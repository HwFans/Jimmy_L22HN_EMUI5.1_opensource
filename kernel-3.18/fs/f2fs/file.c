/*
 * fs/f2fs/file.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/fs.h>
#include <linux/f2fs_fs.h>
#include <linux/stat.h>
#include <linux/buffer_head.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/falloc.h>
#include <linux/types.h>
#include <linux/compat.h>
#include <linux/uaccess.h>
#include <linux/mount.h>
#include <linux/pagevec.h>
#include <linux/random.h>
#include <linux/aio.h>

#include "f2fs.h"
#include "node.h"
#include "segment.h"
#include "xattr.h"
#include "acl.h"
#include "gc.h"
#include "trace.h"
#include <trace/events/f2fs.h>

static int f2fs_vm_page_mkwrite(struct vm_area_struct *vma,
						struct vm_fault *vmf)
{
	struct page *page = vmf->page;
	struct inode *inode = file_inode(vma->vm_file);
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	struct dnode_of_data dn;
	int err;

	sb_start_pagefault(inode->i_sb);

	f2fs_bug_on(sbi, f2fs_has_inline_data(inode));

	/* block allocation */
	f2fs_lock_op(sbi);
	set_new_dnode(&dn, inode, NULL, NULL, 0);
	err = f2fs_reserve_block(&dn, page->index);
	if (err) {
		f2fs_unlock_op(sbi);
		goto out;
	}
	f2fs_put_dnode(&dn);
	f2fs_unlock_op(sbi);

	f2fs_balance_fs(sbi, dn.node_changed);

	file_update_time(vma->vm_file);
	lock_page(page);
	if (unlikely(page->mapping != inode->i_mapping ||
			page_offset(page) > i_size_read(inode) ||
			!PageUptodate(page))) {
		unlock_page(page);
		err = -EFAULT;
		goto out;
	}

	/*
	 * check to see if the page is mapped already (no holes)
	 */
	if (PageMappedToDisk(page))
		goto mapped;

	/* page is wholly or partially inside EOF */
	if (((loff_t)(page->index + 1) << PAGE_CACHE_SHIFT) >
						i_size_read(inode)) {
		unsigned offset;
		offset = i_size_read(inode) & ~PAGE_CACHE_MASK;
		zero_user_segment(page, offset, PAGE_CACHE_SIZE);
	}
	set_page_dirty(page);
	SetPageUptodate(page);

	trace_f2fs_vm_page_mkwrite(page, DATA);
mapped:
	/* fill the page */
	f2fs_wait_on_page_writeback(page, DATA, false);

	/* wait for GCed encrypted page writeback */
	if (f2fs_encrypted_inode(inode) && S_ISREG(inode->i_mode))
		f2fs_wait_on_encrypted_page_writeback(sbi, dn.data_blkaddr);

	/* if gced page is attached, don't write to cold segment */
	clear_cold_data(page);
out:
	sb_end_pagefault(inode->i_sb);
	f2fs_update_time(sbi, REQ_TIME);
	return block_page_mkwrite_return(err);
}

static const struct vm_operations_struct f2fs_file_vm_ops = {
	.fault		= filemap_fault,
	.map_pages	= filemap_map_pages,
	.page_mkwrite	= f2fs_vm_page_mkwrite,
};

static int get_parent_ino(struct inode *inode, nid_t *pino)
{
	struct dentry *dentry;

	inode = igrab(inode);
	dentry = d_find_any_alias(inode);
	iput(inode);
	if (!dentry)
		return 0;

	if (update_dent_inode(inode, inode, &dentry->d_name)) {
		dput(dentry);
		return 0;
	}

	*pino = parent_ino(dentry);
	dput(dentry);
	return 1;
}

static inline bool need_do_checkpoint(struct inode *inode)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	bool need_cp = false;

	if (!S_ISREG(inode->i_mode) || inode->i_nlink != 1)
		need_cp = true;
	else if (is_sbi_flag_set(sbi, SBI_NEED_CP))
		need_cp = true;
	else if (file_wrong_pino(inode))
		need_cp = true;
	else if (!space_for_roll_forward(sbi))
		need_cp = true;
	else if (!is_checkpointed_node(sbi, F2FS_I(inode)->i_pino))
		need_cp = true;
	else if (F2FS_I(inode)->xattr_ver == cur_cp_version(F2FS_CKPT(sbi)))
		need_cp = true;
	else if (test_opt(sbi, FASTBOOT))
		need_cp = true;
	else if (sbi->active_logs == 2)
		need_cp = true;

	return need_cp;
}

static bool need_inode_page_update(struct f2fs_sb_info *sbi, nid_t ino)
{
	struct page *i = find_get_page(NODE_MAPPING(sbi), ino);
	bool ret = false;
	/* But we need to avoid that there are some inode updates */
	if ((i && PageDirty(i)) || need_inode_block_update(sbi, ino))
		ret = true;
	f2fs_put_page(i, 0);
	return ret;
}

static void try_to_fix_pino(struct inode *inode)
{
	struct f2fs_inode_info *fi = F2FS_I(inode);
	nid_t pino;

	down_write(&fi->i_sem);
	fi->xattr_ver = 0;
	if (file_wrong_pino(inode) && inode->i_nlink == 1 &&
			get_parent_ino(inode, &pino)) {
		fi->i_pino = pino;
		file_got_pino(inode);
		up_write(&fi->i_sem);

		mark_inode_dirty_sync(inode);
		f2fs_write_inode(inode, NULL);
	} else {
		up_write(&fi->i_sem);
	}
}

int f2fs_sync_file(struct file *file, loff_t start, loff_t end, int datasync)
{
	struct inode *inode = file->f_mapping->host;
	struct f2fs_inode_info *fi = F2FS_I(inode);
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	nid_t ino = inode->i_ino;
	int ret = 0;
	bool need_cp = false;
	struct writeback_control wbc = {
		.sync_mode = WB_SYNC_ALL,
		.nr_to_write = LONG_MAX,
		.for_reclaim = 0,
	};

	if (unlikely(f2fs_readonly(inode->i_sb)))
		return 0;

	trace_f2fs_sync_file_enter(inode);

	/* if fdatasync is triggered, let's do in-place-update */
	if (datasync || get_dirty_pages(inode) <= SM_I(sbi)->min_fsync_blocks)
		set_inode_flag(fi, FI_NEED_IPU);
	ret = filemap_write_and_wait_range(inode->i_mapping, start, end);
	clear_inode_flag(fi, FI_NEED_IPU);

	if (ret) {
		trace_f2fs_sync_file_exit(inode, need_cp, datasync, ret);
		return ret;
	}

	inode_lock(inode);

	/* if the inode is dirty, let's recover all the time */
	if (!datasync || is_inode_flag_set(fi, FI_ISIZE_CHANGED)) {
		f2fs_write_inode(inode, NULL);
		goto go_write;
	}

	/*
	 * if there is no written data, don't waste time to write recovery info.
	 */
	/*lint -save -e527*/
	if (!is_inode_flag_set(fi, FI_APPEND_WRITE) &&
			!exist_written_data(sbi, ino, APPEND_INO)) {

		/* it may call write_inode just prior to fsync */
		if (need_inode_page_update(sbi, ino))
			goto go_write;

		if (is_inode_flag_set(fi, FI_UPDATE_WRITE) ||
				exist_written_data(sbi, ino, UPDATE_INO))
			goto flush_out;
		goto out;
	}
	/*lint -restore*/
go_write:
	/*
	 * Both of fdatasync() and fsync() are able to be recovered from
	 * sudden-power-off.
	 */
	down_read(&fi->i_sem);
	need_cp = need_do_checkpoint(inode);
	up_read(&fi->i_sem);

	if (need_cp) {
		/* all the dirty node pages should be flushed for POR */
		ret = f2fs_sync_fs(inode->i_sb, 1);

		/*
		 * We've secured consistency through sync_fs. Following pino
		 * will be used only for fsynced inodes after checkpoint.
		 */
		try_to_fix_pino(inode);
		clear_inode_flag(fi, FI_APPEND_WRITE);
		clear_inode_flag(fi, FI_UPDATE_WRITE);
		goto out;
	}
sync_nodes:
	sync_node_pages(sbi, ino, &wbc);

	/* if cp_error was enabled, we should avoid infinite loop */
	if (unlikely(f2fs_cp_error(sbi))) {
		ret = -EIO;
		goto out;
	}

	if (need_inode_block_update(sbi, ino)) {
		mark_inode_dirty_sync(inode);
		f2fs_write_inode(inode, NULL);
		goto sync_nodes;
	}

	ret = wait_on_node_pages_writeback(sbi, ino);
	if (ret)
		goto out;

	/* once recovery info is written, don't need to tack this */
	remove_ino_entry(sbi, ino, APPEND_INO);
	clear_inode_flag(fi, FI_APPEND_WRITE);

	clear_inode_flag(fi, FI_ISIZE_CHANGED);
flush_out:
	remove_ino_entry(sbi, ino, UPDATE_INO);
	clear_inode_flag(fi, FI_UPDATE_WRITE);
	ret = f2fs_issue_flush(sbi);
	f2fs_update_time(sbi, REQ_TIME);
out:
	inode_unlock(inode);
	trace_f2fs_sync_file_exit(inode, need_cp, datasync, ret);
	f2fs_trace_ios(NULL, 1);
	return ret;
}

static pgoff_t __get_first_dirty_index(struct address_space *mapping,
						pgoff_t pgofs, int whence)
{
	struct pagevec pvec;
	int nr_pages;

	if (whence != SEEK_DATA)
		return 0;

	/* find first dirty page index */
	pagevec_init(&pvec, 0);
	nr_pages = pagevec_lookup_tag(&pvec, mapping, &pgofs,
					PAGECACHE_TAG_DIRTY, 1);
	pgofs = nr_pages ? pvec.pages[0]->index : ULONG_MAX;
	pagevec_release(&pvec);
	return pgofs;
}

static bool __found_offset(block_t blkaddr, pgoff_t dirty, pgoff_t pgofs,
							int whence)
{
	switch (whence) {
	case SEEK_DATA:
		if ((blkaddr == NEW_ADDR && dirty == pgofs) ||
			(blkaddr != NEW_ADDR && blkaddr != NULL_ADDR))
			return true;
		break;
	case SEEK_HOLE:
		if (blkaddr == NULL_ADDR)
			return true;
		break;
	}
	return false;
}

static loff_t f2fs_seek_block(struct file *file, loff_t offset, int whence)
{
	struct inode *inode = file->f_mapping->host;
	loff_t maxbytes = inode->i_sb->s_maxbytes;
	struct dnode_of_data dn;
	pgoff_t pgofs, end_offset, dirty;
	loff_t data_ofs = offset;
	loff_t isize;
	int err = 0;

	mutex_lock(&inode->i_mutex);

	isize = i_size_read(inode);
	if (offset >= isize)
		goto fail;

	/* handle inline data case */
	if (f2fs_has_inline_data(inode) || f2fs_has_inline_dentry(inode)) {
		if (whence == SEEK_HOLE)
			data_ofs = isize;
		goto found;
	}

	pgofs = (pgoff_t)(offset >> PAGE_CACHE_SHIFT);

	dirty = __get_first_dirty_index(inode->i_mapping, pgofs, whence);

	for (; data_ofs < isize; data_ofs = (loff_t)pgofs << PAGE_CACHE_SHIFT) {
		set_new_dnode(&dn, inode, NULL, NULL, 0);
		err = get_dnode_of_data(&dn, pgofs, LOOKUP_NODE_RA);
		if (err && err != -ENOENT) {
			goto fail;
		} else if (err == -ENOENT) {
			/* direct node does not exists */
			if (whence == SEEK_DATA) {
				pgofs = get_next_page_offset(&dn, pgofs);
				continue;
			} else {
				goto found;
			}
		}

		end_offset = ADDRS_PER_PAGE(dn.node_page, inode);

		/* find data/hole in dnode block */
		for (; dn.ofs_in_node < end_offset;
				dn.ofs_in_node++, pgofs++,
				data_ofs = (loff_t)pgofs << PAGE_CACHE_SHIFT) {
			block_t blkaddr;
			blkaddr = datablock_addr(dn.node_page, dn.ofs_in_node);

			if (__found_offset(blkaddr, dirty, pgofs, whence)) {
				f2fs_put_dnode(&dn);
				goto found;
			}
		}
		f2fs_put_dnode(&dn);
	}

	if (whence == SEEK_DATA)
		goto fail;
found:
	if (whence == SEEK_HOLE && data_ofs > isize)
		data_ofs = isize;
	mutex_unlock(&inode->i_mutex);
	return vfs_setpos(file, data_ofs, maxbytes);
fail:
	mutex_unlock(&inode->i_mutex);
	return -ENXIO;
}

static loff_t f2fs_llseek(struct file *file, loff_t offset, int whence)
{
	struct inode *inode = file->f_mapping->host;
	loff_t maxbytes = inode->i_sb->s_maxbytes;

	switch (whence) {
	case SEEK_SET:
	case SEEK_CUR:
	case SEEK_END:
		return generic_file_llseek_size(file, offset, whence,
						maxbytes, i_size_read(inode));
	case SEEK_DATA:
	case SEEK_HOLE:
		if (offset < 0)
			return -ENXIO;
		return f2fs_seek_block(file, offset, whence);
	}

	return -EINVAL;
}

static int f2fs_file_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct inode *inode = file_inode(file);
	int err;

	if (f2fs_encrypted_inode(inode)) {
		err = fscrypt_get_encryption_info(inode);
		if (err)
			return 0;
		if (!f2fs_encrypted_inode(inode))
			return -ENOKEY;
	}

	/* we don't need to use inline_data strictly */
	err = f2fs_convert_inline_inode(inode);
	if (err)
		return err;

	file_accessed(file);
	vma->vm_ops = &f2fs_file_vm_ops;
	return 0;
}

static int f2fs_file_open(struct inode *inode, struct file *filp)
{
	int ret = generic_file_open(inode, filp);
	struct inode *dir = filp->f_path.dentry->d_parent->d_inode;

	if (!ret && f2fs_encrypted_inode(inode)) {
		ret = fscrypt_get_encryption_info(inode);
		if (ret)
			return -EACCES;
		if (!fscrypt_has_encryption_key(inode))
			return -ENOKEY;
	}
	if (f2fs_encrypted_inode(dir) &&
			!fscrypt_has_permitted_context(dir, inode))
		return -EPERM;
	return ret;
}

int truncate_data_blocks_range(struct dnode_of_data *dn, int count)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(dn->inode);
	struct f2fs_node *raw_node;
	int nr_free = 0, ofs = dn->ofs_in_node, len = count;
	__le32 *addr;

	raw_node = F2FS_NODE(dn->node_page);
	addr = blkaddr_in_node(raw_node) + ofs;

	for (; count > 0; count--, addr++, dn->ofs_in_node++) {
		block_t blkaddr = le32_to_cpu(*addr);
		if (blkaddr == NULL_ADDR)
			continue;

		dn->data_blkaddr = NULL_ADDR;
		set_data_blkaddr(dn);
		invalidate_blocks(sbi, blkaddr);
		if (dn->ofs_in_node == 0 && IS_INODE(dn->node_page))
			clear_inode_flag(F2FS_I(dn->inode),
						FI_FIRST_BLOCK_WRITTEN);
		nr_free++;
	}

	if (nr_free) {
		pgoff_t fofs;
		/*
		 * once we invalidate valid blkaddr in range [ofs, ofs + count],
		 * we will invalidate all blkaddr in the whole range.
		 */
		fofs = start_bidx_of_node(ofs_of_node(dn->node_page),
							dn->inode) + ofs;
		f2fs_update_extent_cache_range(dn, fofs, 0, len);
		dec_valid_block_count(sbi, dn->inode, nr_free);
		sync_inode_page(dn);
	}
	dn->ofs_in_node = ofs;

	f2fs_update_time(sbi, REQ_TIME);
	trace_f2fs_truncate_data_blocks_range(dn->inode, dn->nid,
					 dn->ofs_in_node, nr_free);
	return nr_free;
}

void truncate_data_blocks(struct dnode_of_data *dn)
{
	truncate_data_blocks_range(dn, ADDRS_PER_BLOCK);
}

static int truncate_partial_data_page(struct inode *inode, u64 from,
								bool cache_only)
{
	unsigned offset = from & (PAGE_CACHE_SIZE - 1);
	pgoff_t index = from >> PAGE_CACHE_SHIFT;
	struct address_space *mapping = inode->i_mapping;
	struct page *page;

	if (!offset && !cache_only)
		return 0;

	if (cache_only) {
		page = f2fs_grab_cache_page(mapping, index, false);
		if (page && PageUptodate(page))
			goto truncate_out;
		f2fs_put_page(page, 1);
		return 0;
	}

	page = get_lock_data_page(inode, index, true);
	if (IS_ERR(page))
		return 0;
truncate_out:
	f2fs_wait_on_page_writeback(page, DATA, true);
	zero_user(page, offset, PAGE_CACHE_SIZE - offset);
	if (!cache_only || !f2fs_encrypted_inode(inode) ||
					!S_ISREG(inode->i_mode))
		set_page_dirty(page);
	f2fs_put_page(page, 1);
	return 0;
}

int truncate_blocks(struct inode *inode, u64 from, bool lock)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	unsigned int blocksize = inode->i_sb->s_blocksize;
	struct dnode_of_data dn;
	pgoff_t free_from;
	int count = 0, err = 0;
	struct page *ipage;
	bool truncate_page = false;

	trace_f2fs_truncate_blocks_enter(inode, from);

	free_from = (pgoff_t)F2FS_BYTES_TO_BLK(from + blocksize - 1);

	if (lock)
		f2fs_lock_op(sbi);

	ipage = get_node_page(sbi, inode->i_ino);
	if (IS_ERR(ipage)) {
		err = PTR_ERR(ipage);
		goto out;
	}

	if (f2fs_has_inline_data(inode)) {
		if (truncate_inline_inode(ipage, from))
			set_page_dirty(ipage);
		f2fs_put_page(ipage, 1);
		truncate_page = true;
		goto out;
	}

	set_new_dnode(&dn, inode, ipage, NULL, 0);
	err = get_dnode_of_data(&dn, free_from, LOOKUP_NODE);
	if (err) {
		if (err == -ENOENT)
			goto free_next;
		goto out;
	}

	count = ADDRS_PER_PAGE(dn.node_page, inode);

	count -= dn.ofs_in_node;
	f2fs_bug_on(sbi, count < 0);

	if (dn.ofs_in_node || IS_INODE(dn.node_page)) {
		truncate_data_blocks_range(&dn, count);
		free_from += count;
	}

	f2fs_put_dnode(&dn);
free_next:
	err = truncate_inode_blocks(inode, free_from);
out:
	if (lock)
		f2fs_unlock_op(sbi);

	/* lastly zero out the first data page */
	if (!err)
		err = truncate_partial_data_page(inode, from, truncate_page);

	trace_f2fs_truncate_blocks_exit(inode, err);
	return err;
}

int f2fs_truncate(struct inode *inode, bool lock)
{
	int err;

	if (!(S_ISREG(inode->i_mode) || S_ISDIR(inode->i_mode) ||
				S_ISLNK(inode->i_mode)))
		return 0;

	trace_f2fs_truncate(inode);

	/* we should check inline_data size */
	if (!f2fs_may_inline_data(inode)) {
		err = f2fs_convert_inline_inode(inode);
		if (err)
			return err;
	}

	err = truncate_blocks(inode, i_size_read(inode), lock);
	if (err)
		return err;

	inode->i_mtime = inode->i_ctime = CURRENT_TIME;
	mark_inode_dirty(inode);
	return 0;
}

int f2fs_getattr(struct vfsmount *mnt,
			 struct dentry *dentry, struct kstat *stat)
{
	struct inode *inode = dentry->d_inode;
	generic_fillattr(inode, stat);
	stat->blocks <<= 3;
	return 0;
}

#ifdef CONFIG_F2FS_FS_POSIX_ACL
static void __setattr_copy(struct inode *inode, const struct iattr *attr)
{
	struct f2fs_inode_info *fi = F2FS_I(inode);
	unsigned int ia_valid = attr->ia_valid;

	if (ia_valid & ATTR_UID)
		inode->i_uid = attr->ia_uid;
	if (ia_valid & ATTR_GID)
		inode->i_gid = attr->ia_gid;
	if (ia_valid & ATTR_ATIME)
		inode->i_atime = timespec_trunc(attr->ia_atime,
						inode->i_sb->s_time_gran);
	if (ia_valid & ATTR_MTIME)
		inode->i_mtime = timespec_trunc(attr->ia_mtime,
						inode->i_sb->s_time_gran);
	if (ia_valid & ATTR_CTIME)
		inode->i_ctime = timespec_trunc(attr->ia_ctime,
						inode->i_sb->s_time_gran);
	if (ia_valid & ATTR_MODE) {
		umode_t mode = attr->ia_mode;

		if (!in_group_p(inode->i_gid) && !capable(CAP_FSETID))
			mode &= ~S_ISGID;
		set_acl_inode(fi, mode);
	}
}
#else
#define __setattr_copy setattr_copy
#endif

int f2fs_setattr(struct dentry *dentry, struct iattr *attr)
{
	struct inode *inode = dentry->d_inode;
	struct f2fs_inode_info *fi = F2FS_I(inode);
	int err;

	err = inode_change_ok(inode, attr);
	if (err)
		return err;

	if (attr->ia_valid & ATTR_SIZE) {
		if (f2fs_encrypted_inode(inode) &&
				fscrypt_get_encryption_info(inode))
			return -EACCES;

		if (attr->ia_size <= i_size_read(inode)) {
			truncate_setsize(inode, attr->ia_size);
			err = f2fs_truncate(inode, true);
			if (err)
				return err;
			f2fs_balance_fs(F2FS_I_SB(inode), true);
		} else {
			/*
			 * do not trim all blocks after i_size if target size is
			 * larger than i_size.
			 */
			truncate_setsize(inode, attr->ia_size);

			/* should convert inline inode here */
			if (!f2fs_may_inline_data(inode)) {
				err = f2fs_convert_inline_inode(inode);
				if (err)
					return err;
			}
			inode->i_mtime = inode->i_ctime = CURRENT_TIME;
		}
	}

	__setattr_copy(inode, attr);

	if (attr->ia_valid & ATTR_MODE) {
		err = posix_acl_chmod(inode, get_inode_mode(inode));
		if (err || is_inode_flag_set(fi, FI_ACL_MODE)) {
			inode->i_mode = fi->i_acl_mode;
			clear_inode_flag(fi, FI_ACL_MODE);
		}
	}

	mark_inode_dirty(inode);
	return err;
}

const struct inode_operations f2fs_file_inode_operations = {
	.getattr	= f2fs_getattr,
	.setattr	= f2fs_setattr,
	.get_acl	= f2fs_get_acl,
	.set_acl	= f2fs_set_acl,
#ifdef CONFIG_F2FS_FS_XATTR
	.setxattr	= generic_setxattr,
	.getxattr	= generic_getxattr,
	.listxattr	= f2fs_listxattr,
	.removexattr	= generic_removexattr,
#endif
	.fiemap		= f2fs_fiemap,
};

static int fill_zero(struct inode *inode, pgoff_t index,
					loff_t start, loff_t len)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	struct page *page;

	if (!len)
		return 0;

	f2fs_balance_fs(sbi, true);

	f2fs_lock_op(sbi);
	page = get_new_data_page(inode, NULL, index, false);
	f2fs_unlock_op(sbi);

	if (IS_ERR(page))
		return PTR_ERR(page);

	f2fs_wait_on_page_writeback(page, DATA, true);
	zero_user(page, start, len);
	set_page_dirty(page);
	f2fs_put_page(page, 1);
	return 0;
}

int truncate_hole(struct inode *inode, pgoff_t pg_start, pgoff_t pg_end)
{
	int err;

	while (pg_start < pg_end) {
		struct dnode_of_data dn;
		pgoff_t end_offset, count;

		set_new_dnode(&dn, inode, NULL, NULL, 0);
		err = get_dnode_of_data(&dn, pg_start, LOOKUP_NODE);
		if (err) {
			if (err == -ENOENT) {
				pg_start++;
				continue;
			}
			return err;
		}

		end_offset = ADDRS_PER_PAGE(dn.node_page, inode);
		count = min(end_offset - dn.ofs_in_node, pg_end - pg_start);

		f2fs_bug_on(F2FS_I_SB(inode), count == 0 || count > end_offset);

		truncate_data_blocks_range(&dn, count);
		f2fs_put_dnode(&dn);

		pg_start += count;
	}
	return 0;
}

static int punch_hole(struct inode *inode, loff_t offset, loff_t len)
{
	pgoff_t pg_start, pg_end;
	loff_t off_start, off_end;
	int ret;

	ret = f2fs_convert_inline_inode(inode);
	if (ret)
		return ret;

	pg_start = ((unsigned long long) offset) >> PAGE_CACHE_SHIFT;
	pg_end = ((unsigned long long) offset + len) >> PAGE_CACHE_SHIFT;

	off_start = offset & (PAGE_CACHE_SIZE - 1);
	off_end = (offset + len) & (PAGE_CACHE_SIZE - 1);

	if (pg_start == pg_end) {
		ret = fill_zero(inode, pg_start, off_start,
						off_end - off_start);
		if (ret)
			return ret;
	} else {
		if (off_start) {
			ret = fill_zero(inode, pg_start++, off_start,
						PAGE_CACHE_SIZE - off_start);
			if (ret)
				return ret;
		}
		if (off_end) {
			ret = fill_zero(inode, pg_end, 0, off_end);
			if (ret)
				return ret;
		}

		if (pg_start < pg_end) {
			struct address_space *mapping = inode->i_mapping;
			loff_t blk_start, blk_end;
			struct f2fs_sb_info *sbi = F2FS_I_SB(inode);

			f2fs_balance_fs(sbi, true);

			blk_start = (loff_t)pg_start << PAGE_CACHE_SHIFT;
			blk_end = (loff_t)pg_end << PAGE_CACHE_SHIFT;
			truncate_inode_pages_range(mapping, blk_start,
					blk_end - 1);

			f2fs_lock_op(sbi);
			ret = truncate_hole(inode, pg_start, pg_end);
			f2fs_unlock_op(sbi);
		}
	}

	return ret;
}

static int __exchange_data_block(struct inode *inode, pgoff_t src,
					pgoff_t dst, bool full)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	struct dnode_of_data dn;
	block_t new_addr;
	bool do_replace = false;
	int ret;

	set_new_dnode(&dn, inode, NULL, NULL, 0);
	ret = get_dnode_of_data(&dn, src, LOOKUP_NODE_RA);
	if (ret && ret != -ENOENT) {
		return ret;
	} else if (ret == -ENOENT) {
		new_addr = NULL_ADDR;
	} else {
		new_addr = dn.data_blkaddr;
		if (!is_checkpointed_data(sbi, new_addr)) {
			/* do not invalidate this block address */
			f2fs_update_data_blkaddr(&dn, NULL_ADDR);
			do_replace = true;
		}
		f2fs_put_dnode(&dn);
	}

	if (new_addr == NULL_ADDR)
		return full ? truncate_hole(inode, dst, dst + 1) : 0;

	if (do_replace) {
		struct page *ipage = get_node_page(sbi, inode->i_ino);
		struct node_info ni;

		if (IS_ERR(ipage)) {
			ret = PTR_ERR(ipage);
			goto err_out;
		}

		set_new_dnode(&dn, inode, ipage, NULL, 0);
		ret = f2fs_reserve_block(&dn, dst);
		if (ret)
			goto err_out;

		truncate_data_blocks_range(&dn, 1);

		get_node_info(sbi, dn.nid, &ni);
		f2fs_replace_block(sbi, &dn, dn.data_blkaddr, new_addr,
				ni.version, true, false);
		f2fs_put_dnode(&dn);
	} else {
		struct page *psrc, *pdst;

		psrc = get_lock_data_page(inode, src, true);
		if (IS_ERR(psrc))
			return PTR_ERR(psrc);
		pdst = get_new_data_page(inode, NULL, dst, true);
		if (IS_ERR(pdst)) {
			f2fs_put_page(psrc, 1);
			return PTR_ERR(pdst);
		}
		f2fs_copy_page(psrc, pdst);
		set_page_dirty(pdst);
		f2fs_put_page(pdst, 1);
		f2fs_put_page(psrc, 1);

		return truncate_hole(inode, src, src + 1);
	}
	return 0;

err_out:
	if (!get_dnode_of_data(&dn, src, LOOKUP_NODE)) {
		f2fs_update_data_blkaddr(&dn, new_addr);
		f2fs_put_dnode(&dn);
	}
	return ret;
}

static int f2fs_do_collapse(struct inode *inode, pgoff_t start, pgoff_t end)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	pgoff_t nrpages = (i_size_read(inode) + PAGE_SIZE - 1) / PAGE_SIZE;
	int ret = 0;

	for (; end < nrpages; start++, end++) {
		f2fs_balance_fs(sbi, true);
		f2fs_lock_op(sbi);
		ret = __exchange_data_block(inode, end, start, true);
		f2fs_unlock_op(sbi);
		if (ret)
			break;
	}
	return ret;
}

static int f2fs_collapse_range(struct inode *inode, loff_t offset, loff_t len)
{
	pgoff_t pg_start, pg_end;
	loff_t new_size;
	int ret;

	if (offset + len >= i_size_read(inode))
		return -EINVAL;

	/* collapse range should be aligned to block size of f2fs. */
	if (offset & (F2FS_BLKSIZE - 1) || len & (F2FS_BLKSIZE - 1))
		return -EINVAL;

	ret = f2fs_convert_inline_inode(inode);
	if (ret)
		return ret;

	pg_start = offset >> PAGE_CACHE_SHIFT;
	pg_end = (offset + len) >> PAGE_CACHE_SHIFT;

	/* write out all dirty pages from offset */
	ret = filemap_write_and_wait_range(inode->i_mapping, offset, LLONG_MAX);
	if (ret)
		return ret;

	truncate_pagecache(inode, offset);

	ret = f2fs_do_collapse(inode, pg_start, pg_end);
	if (ret)
		return ret;

	/* write out all moved pages, if possible */
	filemap_write_and_wait_range(inode->i_mapping, offset, LLONG_MAX);
	truncate_pagecache(inode, offset);

	new_size = i_size_read(inode) - len;
	truncate_pagecache(inode, new_size);

	ret = truncate_blocks(inode, new_size, true);
	if (!ret) {
		i_size_write(inode, new_size);
		set_inode_flag(F2FS_I(inode), FI_ISIZE_CHANGED);
	}

	return ret;
}

static int f2fs_zero_range(struct inode *inode, loff_t offset, loff_t len,
								int mode)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	struct address_space *mapping = inode->i_mapping;
	pgoff_t index, pg_start, pg_end;
	loff_t new_size = i_size_read(inode);
	loff_t off_start, off_end;
	int ret = 0;

	ret = inode_newsize_ok(inode, (len + offset));
	if (ret)
		return ret;

	ret = f2fs_convert_inline_inode(inode);
	if (ret)
		return ret;

	ret = filemap_write_and_wait_range(mapping, offset, offset + len - 1);
	if (ret)
		return ret;

	truncate_pagecache_range(inode, offset, offset + len - 1);

	pg_start = ((unsigned long long) offset) >> PAGE_CACHE_SHIFT;
	pg_end = ((unsigned long long) offset + len) >> PAGE_CACHE_SHIFT;

	off_start = offset & (PAGE_CACHE_SIZE - 1);
	off_end = (offset + len) & (PAGE_CACHE_SIZE - 1);

	if (pg_start == pg_end) {
		ret = fill_zero(inode, pg_start, off_start,
						off_end - off_start);
		if (ret)
			return ret;

		if (offset + len > new_size)
			new_size = offset + len;
		new_size = max_t(loff_t, new_size, offset + len);
	} else {
		if (off_start) {
			ret = fill_zero(inode, pg_start++, off_start,
						PAGE_CACHE_SIZE - off_start);
			if (ret)
				return ret;

			new_size = max_t(loff_t, new_size,
					(loff_t)pg_start << PAGE_CACHE_SHIFT);
		}

		for (index = pg_start; index < pg_end; index++) {
			struct dnode_of_data dn;
			struct page *ipage;

			f2fs_lock_op(sbi);

			ipage = get_node_page(sbi, inode->i_ino);
			if (IS_ERR(ipage)) {
				ret = PTR_ERR(ipage);
				f2fs_unlock_op(sbi);
				goto out;
			}

			set_new_dnode(&dn, inode, ipage, NULL, 0);
			ret = f2fs_reserve_block(&dn, index);
			if (ret) {
				f2fs_unlock_op(sbi);
				goto out;
			}

			if (dn.data_blkaddr != NEW_ADDR) {
				invalidate_blocks(sbi, dn.data_blkaddr);
				f2fs_update_data_blkaddr(&dn, NEW_ADDR);
			}
			f2fs_put_dnode(&dn);
			f2fs_unlock_op(sbi);

			new_size = max_t(loff_t, new_size,
				(loff_t)(index + 1) << PAGE_CACHE_SHIFT);
		}

		if (off_end) {
			ret = fill_zero(inode, pg_end, 0, off_end);
			if (ret)
				goto out;

			new_size = max_t(loff_t, new_size, offset + len);
		}
	}

out:
	if (!(mode & FALLOC_FL_KEEP_SIZE) && i_size_read(inode) < new_size) {
		i_size_write(inode, new_size);
		set_inode_flag(F2FS_I(inode), FI_ISIZE_CHANGED);
		mark_inode_dirty(inode);
		update_inode_page(inode);
	}

	return ret;
}

static int f2fs_insert_range(struct inode *inode, loff_t offset, loff_t len)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	pgoff_t pg_start, pg_end, delta, nrpages, idx;
	loff_t new_size;
	int ret = 0;

	new_size = i_size_read(inode) + len;
	if (new_size > inode->i_sb->s_maxbytes)
		return -EFBIG;

	if (offset >= i_size_read(inode))
		return -EINVAL;

	/* insert range should be aligned to block size of f2fs. */
	if (offset & (F2FS_BLKSIZE - 1) || len & (F2FS_BLKSIZE - 1))
		return -EINVAL;

	ret = f2fs_convert_inline_inode(inode);
	if (ret)
		return ret;

	f2fs_balance_fs(sbi, true);

	ret = truncate_blocks(inode, i_size_read(inode), true);
	if (ret)
		return ret;

	/* write out all dirty pages from offset */
	ret = filemap_write_and_wait_range(inode->i_mapping, offset, LLONG_MAX);
	if (ret)
		return ret;

	truncate_pagecache(inode, offset);

	pg_start = offset >> PAGE_CACHE_SHIFT;
	pg_end = (offset + len) >> PAGE_CACHE_SHIFT;
	delta = pg_end - pg_start;
	nrpages = (i_size_read(inode) + PAGE_SIZE - 1) / PAGE_SIZE;

	for (idx = nrpages - 1; idx >= pg_start && idx != -1; idx--) {
		f2fs_lock_op(sbi);
		ret = __exchange_data_block(inode, idx, idx + delta, false);
		f2fs_unlock_op(sbi);
		if (ret)
			break;
	}

	/* write out all moved pages, if possible */
	filemap_write_and_wait_range(inode->i_mapping, offset, LLONG_MAX);
	truncate_pagecache(inode, offset);

	if (!ret) {
		i_size_write(inode, new_size);
		set_inode_flag(F2FS_I(inode), FI_ISIZE_CHANGED);
	}
	return ret;
}

static int expand_inode_data(struct inode *inode, loff_t offset,
					loff_t len, int mode)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	pgoff_t index, pg_start, pg_end;
	loff_t new_size = i_size_read(inode);
	loff_t off_start, off_end;
	int ret = 0;

	ret = inode_newsize_ok(inode, (len + offset));
	if (ret)
		return ret;

	ret = f2fs_convert_inline_inode(inode);
	if (ret)
		return ret;

	f2fs_balance_fs(sbi, true);

	pg_start = ((unsigned long long) offset) >> PAGE_CACHE_SHIFT;
	pg_end = ((unsigned long long) offset + len) >> PAGE_CACHE_SHIFT;

	off_start = offset & (PAGE_CACHE_SIZE - 1);
	off_end = (offset + len) & (PAGE_CACHE_SIZE - 1);

	f2fs_lock_op(sbi);

	for (index = pg_start; index <= pg_end; index++) {
		struct dnode_of_data dn;

		if (index == pg_end && !off_end)
			goto noalloc;

		set_new_dnode(&dn, inode, NULL, NULL, 0);
		ret = f2fs_reserve_block(&dn, index);
		if (ret)
			break;
noalloc:
		if (pg_start == pg_end)
			new_size = offset + len;
		else if (index == pg_start && off_start)
			new_size = (loff_t)(index + 1) << PAGE_CACHE_SHIFT;
		else if (index == pg_end)
			new_size = ((loff_t)index << PAGE_CACHE_SHIFT) +
								off_end;
		else
			new_size += PAGE_CACHE_SIZE;
	}

	if (!(mode & FALLOC_FL_KEEP_SIZE) &&
		i_size_read(inode) < new_size) {
		i_size_write(inode, new_size);
		set_inode_flag(F2FS_I(inode), FI_ISIZE_CHANGED);
		mark_inode_dirty(inode);
		update_inode_page(inode);
	}
	f2fs_unlock_op(sbi);

	return ret;
}

#define FALLOC_FL_INSERT_RANGE		0X20

static long f2fs_fallocate(struct file *file, int mode,
				loff_t offset, loff_t len)
{
	struct inode *inode = file_inode(file);
	long ret = 0;

	/* f2fs only support ->fallocate for regular file */
	if (!S_ISREG(inode->i_mode))
		return -EINVAL;

	if (f2fs_encrypted_inode(inode) &&
		(mode & (FALLOC_FL_COLLAPSE_RANGE | FALLOC_FL_INSERT_RANGE)))
		return -EOPNOTSUPP;

	if (mode & ~(FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE |
			FALLOC_FL_COLLAPSE_RANGE | FALLOC_FL_ZERO_RANGE |
			FALLOC_FL_INSERT_RANGE))
		return -EOPNOTSUPP;

	mutex_lock(&inode->i_mutex);

	if (mode & FALLOC_FL_PUNCH_HOLE) {
		if (offset >= inode->i_size)
			goto out;

		ret = punch_hole(inode, offset, len);
	} else if (mode & FALLOC_FL_COLLAPSE_RANGE) {
		ret = f2fs_collapse_range(inode, offset, len);
	} else if (mode & FALLOC_FL_ZERO_RANGE) {
		ret = f2fs_zero_range(inode, offset, len, mode);
	} else if (mode & FALLOC_FL_INSERT_RANGE) {
		ret = f2fs_insert_range(inode, offset, len);
	} else {
		ret = expand_inode_data(inode, offset, len, mode);
	}

	if (!ret) {
		inode->i_mtime = inode->i_ctime = CURRENT_TIME;
		mark_inode_dirty(inode);
		f2fs_update_time(F2FS_I_SB(inode), REQ_TIME);
	}

out:
	mutex_unlock(&inode->i_mutex);

	trace_f2fs_fallocate(inode, mode, offset, len, ret);
	return ret;
}

static int f2fs_release_file(struct inode *inode, struct file *filp)
{
	/*
	 * f2fs_relase_file is called at every close calls. So we should
	 * not drop any inmemory pages by close called by other process.
	 */
	if (!(filp->f_mode & FMODE_WRITE) ||
			atomic_read(&inode->i_writecount) != 1)
		return 0;

	/* some remained atomic pages should discarded */
	if (f2fs_is_atomic_file(inode))
		drop_inmem_pages(inode);
	if (f2fs_is_volatile_file(inode)) {
		clear_inode_flag(F2FS_I(inode), FI_VOLATILE_FILE);
		set_inode_flag(F2FS_I(inode), FI_DROP_CACHE);
		filemap_fdatawrite(inode->i_mapping);
		clear_inode_flag(F2FS_I(inode), FI_DROP_CACHE);
	}
	return 0;
}

#define F2FS_REG_FLMASK		(~(FS_DIRSYNC_FL | FS_TOPDIR_FL))
#define F2FS_OTHER_FLMASK	(FS_NODUMP_FL | FS_NOATIME_FL)

static inline __u32 f2fs_mask_flags(umode_t mode, __u32 flags)
{
	if (S_ISDIR(mode))
		return flags;
	else if (S_ISREG(mode))
		return flags & F2FS_REG_FLMASK;
	else
		return flags & F2FS_OTHER_FLMASK;
}

static int f2fs_ioc_getflags(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct f2fs_inode_info *fi = F2FS_I(inode);
	unsigned int flags = fi->i_flags & FS_FL_USER_VISIBLE;
	return put_user(flags, (int __user *)arg);
}

static int f2fs_ioc_setflags(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct f2fs_inode_info *fi = F2FS_I(inode);
	unsigned int flags = fi->i_flags & FS_FL_USER_VISIBLE;
	unsigned int oldflags;
	int ret;

	ret = mnt_want_write_file(filp);
	if (ret)
		return ret;

	if (!inode_owner_or_capable(inode)) {
		ret = -EACCES;
		goto out;
	}

	if (get_user(flags, (int __user *)arg)) {
		ret = -EFAULT;
		goto out;
	}

	flags = f2fs_mask_flags(inode->i_mode, flags);

	mutex_lock(&inode->i_mutex);

	oldflags = fi->i_flags;

	if ((flags ^ oldflags) & (FS_APPEND_FL | FS_IMMUTABLE_FL)) {
		if (!capable(CAP_LINUX_IMMUTABLE)) {
			mutex_unlock(&inode->i_mutex);
			ret = -EPERM;
			goto out;
		}
	}

	flags = flags & FS_FL_USER_MODIFIABLE;
	flags |= oldflags & ~FS_FL_USER_MODIFIABLE;
	fi->i_flags = flags;
	mutex_unlock(&inode->i_mutex);

	f2fs_set_inode_flags(inode);
	inode->i_ctime = CURRENT_TIME;
	mark_inode_dirty(inode);
out:
	mnt_drop_write_file(filp);
	return ret;
}

static int f2fs_ioc_getversion(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);

	return put_user(inode->i_generation, (int __user *)arg);
}

static int f2fs_ioc_start_atomic_write(struct file *filp)
{
	struct inode *inode = file_inode(filp);
	int ret;

	if (!inode_owner_or_capable(inode))
		return -EACCES;

	if (!S_ISREG(inode->i_mode))
		return -EINVAL;

	if (f2fs_is_atomic_file(inode))
		return 0;

	ret = f2fs_convert_inline_inode(inode);
	if (ret)
		return ret;

	set_inode_flag(F2FS_I(inode), FI_ATOMIC_FILE);
	f2fs_update_time(F2FS_I_SB(inode), REQ_TIME);

	if (!get_dirty_pages(inode))
		return 0;

	f2fs_msg(F2FS_I_SB(inode)->sb, KERN_WARNING,
		"Unexpected flush for atomic writes: ino=%lu, npages=%u",
					inode->i_ino, get_dirty_pages(inode));
	ret = filemap_write_and_wait_range(inode->i_mapping, 0, LLONG_MAX);
	if (ret)
		clear_inode_flag(F2FS_I(inode), FI_ATOMIC_FILE);
	return ret;
}

static int f2fs_ioc_commit_atomic_write(struct file *filp)
{
	struct inode *inode = file_inode(filp);
	int ret;

	if (!inode_owner_or_capable(inode))
		return -EACCES;

	if (f2fs_is_volatile_file(inode))
		return 0;

	ret = mnt_want_write_file(filp);
	if (ret)
		return ret;

	if (f2fs_is_atomic_file(inode)) {
		clear_inode_flag(F2FS_I(inode), FI_ATOMIC_FILE);
		ret = commit_inmem_pages(inode);
		if (ret) {
			set_inode_flag(F2FS_I(inode), FI_ATOMIC_FILE);
			goto err_out;
		}
	}

	ret = f2fs_sync_file(filp, 0, LLONG_MAX, 0);
err_out:
	mnt_drop_write_file(filp);
	return ret;
}

static int f2fs_ioc_start_volatile_write(struct file *filp)
{
	struct inode *inode = file_inode(filp);
	int ret;

	if (!inode_owner_or_capable(inode))
		return -EACCES;

	if (!S_ISREG(inode->i_mode))
		return -EINVAL;

	if (f2fs_is_volatile_file(inode))
		return 0;

	ret = f2fs_convert_inline_inode(inode);
	if (ret)
		return ret;

	set_inode_flag(F2FS_I(inode), FI_VOLATILE_FILE);
	f2fs_update_time(F2FS_I_SB(inode), REQ_TIME);
	return 0;
}

static int f2fs_ioc_release_volatile_write(struct file *filp)
{
	struct inode *inode = file_inode(filp);

	if (!inode_owner_or_capable(inode))
		return -EACCES;

	if (!f2fs_is_volatile_file(inode))
		return 0;

	if (!f2fs_is_first_block_written(inode))
		return truncate_partial_data_page(inode, 0, true);

	return punch_hole(inode, 0, F2FS_BLKSIZE);
}

static int f2fs_ioc_abort_volatile_write(struct file *filp)
{
	struct inode *inode = file_inode(filp);
	int ret;

	if (!inode_owner_or_capable(inode))
		return -EACCES;

	ret = mnt_want_write_file(filp);
	if (ret)
		return ret;

	if (f2fs_is_atomic_file(inode)) {
		drop_inmem_pages(inode);
	}
	if (f2fs_is_volatile_file(inode)) {
		clear_inode_flag(F2FS_I(inode), FI_VOLATILE_FILE);
		ret = f2fs_sync_file(filp, 0, LLONG_MAX, 0);
	}

	mnt_drop_write_file(filp);
	f2fs_update_time(F2FS_I_SB(inode), REQ_TIME);
	return ret;
}

static int f2fs_ioc_shutdown(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	struct super_block *sb = sbi->sb;
	__u32 in;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (get_user(in, (__u32 __user *)arg))
		return -EFAULT;

	switch (in) {
	case FS_GOING_DOWN_FULLSYNC:
		sb = freeze_bdev(sb->s_bdev);
		if (sb && !IS_ERR(sb)) {
			f2fs_stop_checkpoint(sbi);
			thaw_bdev(sb->s_bdev, sb);
		}
		break;
	case FS_GOING_DOWN_METASYNC:
		/* do checkpoint only */
		f2fs_sync_fs(sb, 1);
		f2fs_stop_checkpoint(sbi);
		break;
	case FS_GOING_DOWN_NOSYNC:
		f2fs_stop_checkpoint(sbi);
		break;
	case FS_GOING_DOWN_METAFLUSH:
		sync_meta_pages(sbi, META, LONG_MAX);
		f2fs_stop_checkpoint(sbi);
		break;
	default:
		return -EINVAL;
	}
	f2fs_update_time(sbi, REQ_TIME);
	return 0;
}

static int f2fs_ioc_fitrim(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct super_block *sb = inode->i_sb;
	struct request_queue *q = bdev_get_queue(sb->s_bdev);
	struct fstrim_range range;
	int ret;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (!blk_queue_discard(q))
		return -EOPNOTSUPP;

	if (copy_from_user(&range, (struct fstrim_range __user *)arg,
				sizeof(range)))
		return -EFAULT;

	/*
	 * disable aligning minlen to discard granularity of device,
	 * so that smaller discard could be issued from FITRIM interface.
	 */
	#if 0
	range.minlen = max((unsigned int)range.minlen,
				q->limits.discard_granularity);
	#endif

	ret = f2fs_trim_fs(F2FS_SB(sb), &range);
	if (ret < 0)
		return ret;

	if (copy_to_user((struct fstrim_range __user *)arg, &range,
				sizeof(range)))
		return -EFAULT;
	f2fs_update_time(F2FS_I_SB(inode), REQ_TIME);
	return 0;
}

static bool uuid_is_nonzero(__u8 u[16])
{
	int i;

	for (i = 0; i < 16; i++)
		if (u[i])
			return true;
	return false;
}

static int f2fs_ioc_set_encryption_policy(struct file *filp, unsigned long arg)
{
	struct fscrypt_policy policy;
	struct inode *inode = file_inode(filp);

	if (copy_from_user(&policy, (struct fscrypt_policy __user *)arg,
							sizeof(policy)))
		return -EFAULT;

	f2fs_update_time(F2FS_I_SB(inode), REQ_TIME);
	return fscrypt_process_policy(inode, &policy);
}

static int f2fs_ioc_get_encryption_policy(struct file *filp, unsigned long arg)
{
	struct fscrypt_policy policy;
	struct inode *inode = file_inode(filp);
	int err;

	err = fscrypt_get_policy(inode, &policy);
	if (err)
		return err;

	if (copy_to_user((struct fscrypt_policy __user *)arg, &policy, sizeof(policy)))
		return -EFAULT;
	return 0;
}

static int f2fs_ioc_get_encryption_pwsalt(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	int err;

	if (!f2fs_sb_has_crypto(inode->i_sb))
		return -EOPNOTSUPP;

	if (uuid_is_nonzero(sbi->raw_super->encrypt_pw_salt))
		goto got_it;

	err = mnt_want_write_file(filp);
	if (err)
		return err;

	/* update superblock with uuid */
	generate_random_uuid(sbi->raw_super->encrypt_pw_salt);

	err = f2fs_commit_super(sbi, false);
	if (err) {
		/* undo new data */
		memset(sbi->raw_super->encrypt_pw_salt, 0, 16);
		mnt_drop_write_file(filp);
		return err;
	}
	mnt_drop_write_file(filp);
got_it:
	if (copy_to_user((__u8 __user *)arg, sbi->raw_super->encrypt_pw_salt,
									16))
		return -EFAULT;
	return 0;
}

static int f2fs_ioc_gc(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	__u32 sync;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (get_user(sync, (__u32 __user *)arg))
		return -EFAULT;

	if (f2fs_readonly(sbi->sb))
		return -EROFS;

	if (!sync) {
		if (!mutex_trylock(&sbi->gc_mutex))
			return -EBUSY;
	} else {
		mutex_lock(&sbi->gc_mutex);
	}

	return f2fs_gc(sbi, sync, true);
}

static int f2fs_ioc_write_checkpoint(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (f2fs_readonly(sbi->sb))
		return -EROFS;

	return f2fs_sync_fs(sbi->sb, 1);
}

static int f2fs_defragment_range(struct f2fs_sb_info *sbi,
					struct file *filp,
					struct f2fs_defragment *range)
{
	struct inode *inode = file_inode(filp);
	struct f2fs_map_blocks map = { .m_next_pgofs = NULL };
	struct extent_info ei;
	pgoff_t pg_start, pg_end;
	unsigned int blk_per_seg = sbi->blocks_per_seg;
	unsigned int total = 0, sec_num;
	unsigned int pages_per_sec = sbi->segs_per_sec * blk_per_seg;
	block_t blk_end = 0;
	bool fragmented = false;
	int err;

	/* if in-place-update policy is enabled, don't waste time here */
	if (need_inplace_update(inode))
		return -EINVAL;

	pg_start = range->start >> PAGE_CACHE_SHIFT;
	pg_end = (range->start + range->len) >> PAGE_CACHE_SHIFT;

	f2fs_balance_fs(sbi, true);

	mutex_lock(&inode->i_mutex);

	/* writeback all dirty pages in the range */
	err = filemap_write_and_wait_range(inode->i_mapping, range->start,
						range->start + range->len - 1);
	if (err)
		goto out;

	/*
	 * lookup mapping info in extent cache, skip defragmenting if physical
	 * block addresses are continuous.
	 */
	if (f2fs_lookup_extent_cache(inode, pg_start, &ei)) {
		if (ei.fofs + ei.len >= pg_end)
			goto out;
	}

	map.m_lblk = pg_start;

	/*
	 * lookup mapping info in dnode page cache, skip defragmenting if all
	 * physical block addresses are continuous even if there are hole(s)
	 * in logical blocks.
	 */
	while (map.m_lblk < pg_end) {
		map.m_len = pg_end - map.m_lblk;
		err = f2fs_map_blocks(inode, &map, 0, F2FS_GET_BLOCK_READ);
		if (err)
			goto out;

		if (!(map.m_flags & F2FS_MAP_FLAGS)) {
			map.m_lblk++;
			continue;
		}

		if (blk_end && blk_end != map.m_pblk) {
			fragmented = true;
			break;
		}
		blk_end = map.m_pblk + map.m_len;

		map.m_lblk += map.m_len;
	}

	if (!fragmented)
		goto out;

	map.m_lblk = pg_start;
	map.m_len = pg_end - pg_start;

	sec_num = (map.m_len + pages_per_sec - 1) / pages_per_sec;

	/*
	 * make sure there are enough free section for LFS allocation, this can
	 * avoid defragment running in SSR mode when free section are allocated
	 * intensively
	 */
	if (has_not_enough_free_secs(sbi, sec_num)) {
		err = -EAGAIN;
		goto out;
	}

	while (map.m_lblk < pg_end) {
		pgoff_t idx;
		int cnt = 0;

do_map:
		map.m_len = pg_end - map.m_lblk;
		err = f2fs_map_blocks(inode, &map, 0, F2FS_GET_BLOCK_READ);
		if (err)
			goto clear_out;

		if (!(map.m_flags & F2FS_MAP_FLAGS)) {
			map.m_lblk++;
			continue;
		}

		set_inode_flag(F2FS_I(inode), FI_DO_DEFRAG);

		idx = map.m_lblk;
		while (idx < map.m_lblk + map.m_len && cnt < blk_per_seg) {
			struct page *page;

			page = get_lock_data_page(inode, idx, true);
			if (IS_ERR(page)) {
				err = PTR_ERR(page);
				goto clear_out;
			}

			set_page_dirty(page);
			f2fs_put_page(page, 1);

			idx++;
			cnt++;
			total++;
		}

		map.m_lblk = idx;

		if (idx < pg_end && cnt < blk_per_seg)
			goto do_map;

		clear_inode_flag(F2FS_I(inode), FI_DO_DEFRAG);

		err = filemap_fdatawrite(inode->i_mapping);
		if (err)
			goto out;
	}
clear_out:
	clear_inode_flag(F2FS_I(inode), FI_DO_DEFRAG);
out:
	mutex_unlock(&inode->i_mutex);
	if (!err)
		range->len = (u64)total << PAGE_CACHE_SHIFT;
	return err;
}

static int f2fs_ioc_defragment(struct file *filp, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	struct f2fs_sb_info *sbi = F2FS_I_SB(inode);
	struct f2fs_defragment range;
	int err;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (!S_ISREG(inode->i_mode))
		return -EINVAL;

	err = mnt_want_write_file(filp);
	if (err)
		return err;

	if (f2fs_readonly(sbi->sb)) {
		err = -EROFS;
		goto out;
	}

	if (copy_from_user(&range, (struct f2fs_defragment __user *)arg,
							sizeof(range))) {
		err = -EFAULT;
		goto out;
	}

	/* verify alignment of offset & size */
	if (range.start & (F2FS_BLKSIZE - 1) ||
		range.len & (F2FS_BLKSIZE - 1)) {
		err = -EINVAL;
		goto out;
	}

	err = f2fs_defragment_range(sbi, filp, &range);
	f2fs_update_time(sbi, REQ_TIME);
	if (err < 0)
		goto out;

	if (copy_to_user((struct f2fs_defragment __user *)arg, &range,
							sizeof(range)))
		err = -EFAULT;
out:
	mnt_drop_write_file(filp);
	return err;
}

long f2fs_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case F2FS_IOC_GETFLAGS:
		return f2fs_ioc_getflags(filp, arg);
	case F2FS_IOC_SETFLAGS:
		return f2fs_ioc_setflags(filp, arg);
	case F2FS_IOC_GETVERSION:
		return f2fs_ioc_getversion(filp, arg);
	case F2FS_IOC_START_ATOMIC_WRITE:
		return f2fs_ioc_start_atomic_write(filp);
	case F2FS_IOC_COMMIT_ATOMIC_WRITE:
		return f2fs_ioc_commit_atomic_write(filp);
	case F2FS_IOC_START_VOLATILE_WRITE:
		return f2fs_ioc_start_volatile_write(filp);
	case F2FS_IOC_RELEASE_VOLATILE_WRITE:
		return f2fs_ioc_release_volatile_write(filp);
	case F2FS_IOC_ABORT_VOLATILE_WRITE:
		return f2fs_ioc_abort_volatile_write(filp);
	case FS_IOC_SHUTDOWN:
		return f2fs_ioc_shutdown(filp, arg);
	case FITRIM:
		return f2fs_ioc_fitrim(filp, arg);
	case F2FS_IOC_SET_ENCRYPTION_POLICY:
		return f2fs_ioc_set_encryption_policy(filp, arg);
	case F2FS_IOC_GET_ENCRYPTION_POLICY:
		return f2fs_ioc_get_encryption_policy(filp, arg);
	case F2FS_IOC_GET_ENCRYPTION_PWSALT:
		return f2fs_ioc_get_encryption_pwsalt(filp, arg);
	case F2FS_IOC_GARBAGE_COLLECT:
		return f2fs_ioc_gc(filp, arg);
	case F2FS_IOC_WRITE_CHECKPOINT:
		return f2fs_ioc_write_checkpoint(filp, arg);
	case F2FS_IOC_DEFRAGMENT:
		return f2fs_ioc_defragment(filp, arg);
	
	/*lint -save -e30 -e 142*/
	case F2FS_IOC_NAT_FIEMAP:
		return f2fs_nat_fiemap(filp, arg);
	/*lint restore*/
	default:
		return -ENOTTY;
	}
}

static ssize_t f2fs_file_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
	struct file *file = iocb->ki_filp;
	struct inode *inode = file_inode(file);
	size_t count = iov_iter_count(from);
	loff_t pos = iocb->ki_pos;
	ssize_t ret;

	if (f2fs_encrypted_inode(inode) &&
				!fscrypt_has_encryption_key(inode) &&
				fscrypt_get_encryption_info(inode))
		return -EACCES;

	inode_lock(inode);
	ret = generic_write_checks(file, &pos, &count, S_ISBLK(inode->i_mode));
	if (!ret) {
		ret = f2fs_preallocate_blocks(inode, pos, count,
				file->f_flags & O_DIRECT);
		if (!ret)
			ret = __generic_file_write_iter(iocb, from);
	}
	inode_unlock(inode);

	if (ret > 0) {
		ssize_t err;

		err = generic_write_sync(file, iocb->ki_pos - ret, ret);
		if (err < 0)
			ret = err;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
long f2fs_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case F2FS_IOC32_GETFLAGS:
		cmd = F2FS_IOC_GETFLAGS;
		break;
	case F2FS_IOC32_SETFLAGS:
		cmd = F2FS_IOC_SETFLAGS;
		break;
	case F2FS_IOC32_GETVERSION:
		cmd = F2FS_IOC_GETVERSION;
		break;
	case F2FS_IOC_START_ATOMIC_WRITE:
	case F2FS_IOC_COMMIT_ATOMIC_WRITE:
	case F2FS_IOC_START_VOLATILE_WRITE:
	case F2FS_IOC_RELEASE_VOLATILE_WRITE:
	case F2FS_IOC_ABORT_VOLATILE_WRITE:
	case FS_IOC_SHUTDOWN:
	case F2FS_IOC_SET_ENCRYPTION_POLICY:
	case F2FS_IOC_GET_ENCRYPTION_PWSALT:
	case F2FS_IOC_GET_ENCRYPTION_POLICY:
	case F2FS_IOC_GARBAGE_COLLECT:
	case F2FS_IOC_WRITE_CHECKPOINT:
	case F2FS_IOC_DEFRAGMENT:
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return f2fs_ioctl(file, cmd, (unsigned long) compat_ptr(arg));
}
#endif

const struct file_operations f2fs_file_operations = {
	.llseek		= f2fs_llseek,
	.read_iter	= generic_file_read_iter,
	.write_iter	= f2fs_file_write_iter,
	.open		= f2fs_file_open,
	.release	= f2fs_release_file,
	.mmap		= f2fs_file_mmap,
	.fsync		= f2fs_sync_file,
	.fallocate	= f2fs_fallocate,
	.unlocked_ioctl	= f2fs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= f2fs_compat_ioctl,
#endif
	.splice_read	= generic_file_splice_read,
	.splice_write	= iter_file_splice_write,
};
