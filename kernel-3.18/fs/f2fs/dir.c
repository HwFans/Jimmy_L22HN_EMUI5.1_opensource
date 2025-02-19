/*
 * fs/f2fs/dir.c
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
#include <linux/namei.h>
#include <linux/ctype.h>
#include "f2fs.h"
#include "node.h"
#include "acl.h"
#include "xattr.h"

static unsigned long dir_blocks(struct inode *inode)
{
	return ((unsigned long long) (i_size_read(inode) + PAGE_CACHE_SIZE - 1))
							>> PAGE_CACHE_SHIFT;
}

static unsigned int dir_buckets(unsigned int level, int dir_level)
{
	if (level + dir_level < MAX_DIR_HASH_DEPTH / 2)
		return 1 << (level + dir_level);
	else
		return MAX_DIR_BUCKETS;
}

static unsigned int bucket_blocks(unsigned int level)
{
	if (level < MAX_DIR_HASH_DEPTH / 2)
		return 2;
	else
		return 4;
}

unsigned char f2fs_filetype_table[F2FS_FT_MAX] = {
	[F2FS_FT_UNKNOWN]	= DT_UNKNOWN,
	[F2FS_FT_REG_FILE]	= DT_REG,
	[F2FS_FT_DIR]		= DT_DIR,
	[F2FS_FT_CHRDEV]	= DT_CHR,
	[F2FS_FT_BLKDEV]	= DT_BLK,
	[F2FS_FT_FIFO]		= DT_FIFO,
	[F2FS_FT_SOCK]		= DT_SOCK,
	[F2FS_FT_SYMLINK]	= DT_LNK,
};

#define S_SHIFT 12
static unsigned char f2fs_type_by_mode[S_IFMT >> S_SHIFT] = {
	[S_IFREG >> S_SHIFT]	= F2FS_FT_REG_FILE,
	[S_IFDIR >> S_SHIFT]	= F2FS_FT_DIR,
	[S_IFCHR >> S_SHIFT]	= F2FS_FT_CHRDEV,
	[S_IFBLK >> S_SHIFT]	= F2FS_FT_BLKDEV,
	[S_IFIFO >> S_SHIFT]	= F2FS_FT_FIFO,
	[S_IFSOCK >> S_SHIFT]	= F2FS_FT_SOCK,
	[S_IFLNK >> S_SHIFT]	= F2FS_FT_SYMLINK,
};

void set_de_type(struct f2fs_dir_entry *de, umode_t mode)
{
	de->file_type = f2fs_type_by_mode[(mode & S_IFMT) >> S_SHIFT];
}

static unsigned long dir_block_index(unsigned int level,
				int dir_level, unsigned int idx)
{
	unsigned long i;
	unsigned long bidx = 0;

	for (i = 0; i < level; i++)
		bidx += dir_buckets(i, dir_level) * bucket_blocks(i);
	bidx += idx * bucket_blocks(level);
	return bidx;
}

static struct f2fs_dir_entry *find_in_block(struct inode *dir,
				struct page *dentry_page,
				struct fscrypt_name *fname,
				f2fs_hash_t namehash,
				int *max_slots,
				struct page **res_page,
				struct fscrypt_str *fstr)
{
	struct f2fs_dentry_block *dentry_blk;
	struct f2fs_dir_entry *de;
	struct f2fs_dentry_ptr d;

	dentry_blk = (struct f2fs_dentry_block *)kmap(dentry_page);

	make_dentry_ptr(NULL, &d, (void *)dentry_blk, 1);
	de = find_target_dentry(dir, fname, namehash, max_slots, &d, fstr);
	if (de)
		*res_page = dentry_page;
	else
		kunmap(dentry_page);

	return de;
}

struct f2fs_dir_entry *find_target_dentry(struct inode *inode,
			struct fscrypt_name *fname,
			f2fs_hash_t namehash, int *max_slots,
			struct f2fs_dentry_ptr *d, struct fscrypt_str *fstr)
{
	struct f2fs_dir_entry *de;
	unsigned long bit_pos = 0;
	int max_len = 0;
	struct fscrypt_str de_name = FSTR_INIT(NULL, 0);
	struct fscrypt_str *name = &fname->disk_name;
	struct fscrypt_str tmp = FSTR_INIT(NULL, 0);
	int err;

	if (max_slots)
		*max_slots = 0;

	/* if fstr is not NULL, we are doing case-insensitive lookup */
	if (fstr && f2fs_encrypted_inode(inode)) {
		err = fscrypt_fname_alloc_buffer(inode, F2FS_NAME_LEN, &tmp);
		if (err < 0)
			return ERR_PTR((long)err);
	} else if (fstr) {
		fstr->name = kmalloc((size_t)name->len + 1, GFP_NOFS);
		if (!fstr->name)
			return ERR_PTR((long)(-ENOMEM));
		fstr->len = name->len + 1;
	}

	while (bit_pos < d->max) {
		if (!test_bit_le(bit_pos, d->bitmap)) {
			bit_pos++;
			max_len++;
			continue;
		}

		de = &d->dentry[bit_pos];

		if (unlikely(!de->name_len)) {
			bit_pos++;
			continue;
		}

		/* encrypted case */
		de_name.name = d->filename[bit_pos];
		de_name.len = le16_to_cpu(de->name_len);

		/* show encrypted name */
		/*
		 * If we find the dentry before we check if it is case-
		 * insensitive lookup, we free the tmp and set tmp.name
		 * as NULL. f2fs_lookup will use dentry->d_name as ci_name
		 */
		if (fname->hash) {
			if (de->hash_code == fname->hash)
				goto found_free;
		} else if (de_name.len == name->len &&
			de->hash_code == namehash &&
			!memcmp(de_name.name, name->name, name->len)) {
			goto found_free;
		} else if (fstr && de_name.len == name->len) {
			if (f2fs_encrypted_inode(inode)) {
				u32 save_len = tmp.len;
				de_name.name = kmalloc(de_name.len, GFP_NOFS);
				if (!de_name.name)
					return ERR_PTR((long)-ENOMEM);
                /*lint -e409*/
                memcpy(de_name.name, d->filename[bit_pos], de_name.len);
                /*lint -restore*/
				memset(tmp.name, 0, (size_t)save_len);
				err = fscrypt_fname_disk_to_usr(inode, (u32)de->hash_code, 0, &de_name, &tmp);
				kfree(de_name.name);
				if (err < 0) {
					fscrypt_fname_free_buffer(&tmp);
					return ERR_PTR((long)err);
				}

				de_name.name = tmp.name;
				de_name.len = tmp.len;

				tmp.len = save_len;
			}

			if (!strncasecmp(de_name.name, fname->usr_fname->name, fname->usr_fname->len) &&
			    de_name.len == fname->usr_fname->len) {
				/* do not free tmp if it hits */
				if (!fstr->name)
					fstr->name = de_name.name;
				else
					memcpy(fstr->name, de_name.name, (size_t)de_name.len);
				fstr->len = de_name.len;
				goto found;
			}
		}

		if (max_slots && max_len > *max_slots)
			*max_slots = max_len;
		max_len = 0;

		bit_pos += GET_DENTRY_SLOTS(le16_to_cpu(de->name_len));
	}

	de = NULL;
found_free:
	if (fstr && f2fs_encrypted_inode(inode))
		fscrypt_fname_free_buffer(&tmp);
	else if (fstr) {
		kfree(fstr->name);
		fstr->name = NULL;
		fstr->len = 0;
	}

found:
	if (max_slots && max_len > *max_slots)
		*max_slots = max_len;
	return de;
}

static struct f2fs_dir_entry *find_in_level(struct inode *dir,
					unsigned int level,
					struct fscrypt_name *fname,
					struct page **res_page,
					struct fscrypt_str *fstr)
{
	struct qstr name = FSTR_TO_QSTR(&fname->disk_name);
	int s = GET_DENTRY_SLOTS(name.len);
	unsigned int nbucket, nblock;
	unsigned int bidx, end_block;
	struct page *dentry_page;
	struct f2fs_dir_entry *de = NULL;
	bool room = false;
	int max_slots;
	f2fs_hash_t namehash;

	if(fname->hash)
		namehash = cpu_to_le32(fname->hash);
	else
		namehash = f2fs_dentry_hash(&name);

	nbucket = dir_buckets(level, F2FS_I(dir)->i_dir_level);
	nblock = bucket_blocks(level);

	bidx = dir_block_index(level, F2FS_I(dir)->i_dir_level,
					le32_to_cpu(namehash) % nbucket);
	end_block = bidx + nblock;

	for (; bidx < end_block; bidx++) {
		/* no need to allocate new dentry pages to all the indices */
		dentry_page = find_data_page(dir, bidx);
		if (IS_ERR(dentry_page)) {
			room = true;
			continue;
		}

		de = find_in_block(dir, dentry_page, fname, namehash, &max_slots,
						res_page, fstr);
		if (de)
			break;

		if (max_slots >= s)
			room = true;
		f2fs_put_page(dentry_page, 0);
	}

	if (!de && room && F2FS_I(dir)->chash != namehash) {
		F2FS_I(dir)->chash = namehash;
		F2FS_I(dir)->clevel = level;
	}

	return de;
}

struct f2fs_dir_entry *__f2fs_find_entry(struct inode *dir,
			struct fscrypt_name *fname, struct page **res_page,
			struct fscrypt_str *fstr)
{
	unsigned long npages = dir_blocks(dir);
	struct f2fs_dir_entry *de = NULL;
	unsigned int max_depth;
	unsigned int level;

	*res_page = NULL;

	if (f2fs_has_inline_dentry(dir)) {
		de = find_in_inline_dir(dir, fname, res_page, fstr);
		goto out;
	}

	if (npages == 0)
		goto out;

	max_depth = F2FS_I(dir)->i_current_depth;
	if (unlikely(max_depth > MAX_DIR_HASH_DEPTH)) {
		f2fs_msg(F2FS_I_SB(dir)->sb, KERN_WARNING,
				"Corrupted max_depth of %lu: %u",
				dir->i_ino, max_depth);
		max_depth = MAX_DIR_HASH_DEPTH;
		F2FS_I(dir)->i_current_depth = max_depth;
		mark_inode_dirty(dir);
	}

	for (level = 0; level < max_depth; level++) {
		de = find_in_level(dir, level, fname, res_page, fstr);
		if (de)
			break;
	}
out:
	return de;
}

/*
 * Find an entry in the specified directory with the wanted name.
 * It returns the page where the entry was found (as a parameter - res_page),
 * and the entry itself. Page is returned mapped and unlocked.
 * Entry is guaranteed to be valid.
 */
struct f2fs_dir_entry *f2fs_find_entry(struct inode *dir,
			struct qstr *child, struct page **res_page,
			struct fscrypt_str *fstr)
{
	struct f2fs_dir_entry *de;
	struct fscrypt_name fname;
	int err;

	err = fscrypt_setup_filename(dir, child, 1, &fname);
	if (err)
		return NULL;

	de = __f2fs_find_entry(dir, &fname, res_page, fstr);

	fscrypt_free_filename(&fname);
	return de;
}

struct f2fs_dir_entry *f2fs_parent_dir(struct inode *dir, struct page **p)
{
	struct page *page;
	struct f2fs_dir_entry *de;
	struct f2fs_dentry_block *dentry_blk;

	if (f2fs_has_inline_dentry(dir))
		return f2fs_parent_inline_dir(dir, p);

	page = get_lock_data_page(dir, 0, false);
	if (IS_ERR(page))
		return NULL;

	dentry_blk = kmap(page);
	de = &dentry_blk->dentry[1];
	*p = page;
	unlock_page(page);
	return de;
}

ino_t f2fs_inode_by_name(struct inode *dir, struct qstr *qstr)
{
	ino_t res = 0;
	struct f2fs_dir_entry *de;
	struct page *page;

	de = f2fs_find_entry(dir, qstr, &page, NULL);
	if (de) {
		res = le32_to_cpu(de->ino);
		f2fs_dentry_kunmap(dir, page);
		f2fs_put_page(page, 0);
	}

	return res;
}

void f2fs_set_link(struct inode *dir, struct f2fs_dir_entry *de,
		struct page *page, struct inode *inode)
{
	enum page_type type = f2fs_has_inline_dentry(dir) ? NODE : DATA;
	lock_page(page);
	f2fs_wait_on_page_writeback(page, type, true);
	de->ino = cpu_to_le32(inode->i_ino);
	set_de_type(de, inode->i_mode);
	f2fs_dentry_kunmap(dir, page);
	set_page_dirty(page);
	dir->i_mtime = dir->i_ctime = CURRENT_TIME;
	mark_inode_dirty(dir);

	f2fs_put_page(page, 1);
}

static void init_dent_inode(const struct qstr *name, struct page *ipage)
{
	struct f2fs_inode *ri;

	f2fs_wait_on_page_writeback(ipage, NODE, true);

	/* copy name info. to this inode page */
	ri = F2FS_INODE(ipage);
	ri->i_namelen = cpu_to_le32(name->len);
	memcpy(ri->i_name, name->name, name->len);
	set_page_dirty(ipage);
}

int update_dent_inode(struct inode *inode, struct inode *to,
					const struct qstr *name)
{
	struct page *page;

	if (file_enc_name(to))
		return 0;

	page = get_node_page(F2FS_I_SB(inode), inode->i_ino);
	if (IS_ERR(page))
		return PTR_ERR(page);

	init_dent_inode(name, page);
	f2fs_put_page(page, 1);

	return 0;
}

void do_make_empty_dir(struct inode *inode, struct inode *parent,
					struct f2fs_dentry_ptr *d)
{
	struct qstr dot = QSTR_INIT(".", 1);
	struct qstr dotdot = QSTR_INIT("..", 2);

	/* update dirent of "." */
	f2fs_update_dentry(inode->i_ino, inode->i_mode, d, &dot, 0, 0);

	/* update dirent of ".." */
	f2fs_update_dentry(parent->i_ino, parent->i_mode, d, &dotdot, 0, 1);
}

static int make_empty_dir(struct inode *inode,
		struct inode *parent, struct page *page)
{
	struct page *dentry_page;
	struct f2fs_dentry_block *dentry_blk;
	struct f2fs_dentry_ptr d;

	if (f2fs_has_inline_dentry(inode))
		return make_empty_inline_dir(inode, parent, page);

	dentry_page = get_new_data_page(inode, page, 0, true);
	if (IS_ERR(dentry_page))
		return PTR_ERR(dentry_page);

	dentry_blk = kmap_atomic(dentry_page);

	make_dentry_ptr(NULL, &d, (void *)dentry_blk, 1);
	do_make_empty_dir(inode, parent, &d);

	kunmap_atomic(dentry_blk);

	set_page_dirty(dentry_page);
	f2fs_put_page(dentry_page, 1);
	return 0;
}

struct page *init_inode_metadata(struct inode *inode, struct inode *dir,
			const struct qstr *new_name, const struct qstr *orig_name,
			struct page *dpage)
{
	struct page *page;
	int err;

	if (is_inode_flag_set(F2FS_I(inode), FI_NEW_INODE)) {
		page = new_inode_page(inode);
		if (IS_ERR(page))
			return page;

		if (S_ISDIR(inode->i_mode)) {
			err = make_empty_dir(inode, dir, page);
			if (err)
				goto error;
		}

		err = f2fs_init_acl(inode, dir, page, dpage);
		if (err)
			goto put_error;

		err = f2fs_init_security(inode, dir, orig_name, page);
		if (err)
			goto put_error;

		if (f2fs_encrypted_inode(dir) && f2fs_may_encrypt(inode)) {
			err = fscrypt_inherit_context(dir, inode, page, false);
			if (err)
				goto put_error;
		}
	} else {
		page = get_node_page(F2FS_I_SB(dir), inode->i_ino);
		if (IS_ERR(page))
			return page;

		set_cold_node(inode, page);
	}

	if (new_name)
		init_dent_inode(new_name, page);

	/*
	 * This file should be checkpointed during fsync.
	 * We lost i_pino from now on.
	 */
	if (is_inode_flag_set(F2FS_I(inode), FI_INC_LINK)) {
		file_lost_pino(inode);
		/*
		 * If link the tmpfile to alias through linkat path,
		 * we should remove this inode from orphan list.
		 */
		if (inode->i_nlink == 0)
			remove_orphan_inode(F2FS_I_SB(dir), inode->i_ino);
		inc_nlink(inode);
	}
	return page;

put_error:
	f2fs_put_page(page, 1);
error:
	/* once the failed inode becomes a bad inode, i_mode is S_IFREG */
	truncate_inode_pages(&inode->i_data, 0);
	truncate_blocks(inode, 0, false);
	remove_dirty_inode(inode, false);
	remove_inode_page(inode);
	return ERR_PTR(err);
}

void update_parent_metadata(struct inode *dir, struct inode *inode,
						unsigned int current_depth)
{
	if (inode && is_inode_flag_set(F2FS_I(inode), FI_NEW_INODE)) {
		if (S_ISDIR(inode->i_mode)) {
			inc_nlink(dir);
			set_inode_flag(F2FS_I(dir), FI_UPDATE_DIR);
		}
		clear_inode_flag(F2FS_I(inode), FI_NEW_INODE);
	}
	dir->i_mtime = dir->i_ctime = CURRENT_TIME;
	mark_inode_dirty(dir);

	if (F2FS_I(dir)->i_current_depth != current_depth) {
		F2FS_I(dir)->i_current_depth = current_depth;
		set_inode_flag(F2FS_I(dir), FI_UPDATE_DIR);
	}

	if (inode && is_inode_flag_set(F2FS_I(inode), FI_INC_LINK))
		clear_inode_flag(F2FS_I(inode), FI_INC_LINK);
}

int room_for_filename(const void *bitmap, int slots, int max_slots)
{
	int bit_start = 0;
	int zero_start, zero_end;
next:
	zero_start = find_next_zero_bit_le(bitmap, max_slots, bit_start);
	if (zero_start >= max_slots)
		return max_slots;

	zero_end = find_next_bit_le(bitmap, max_slots, zero_start);
	if (zero_end - zero_start >= slots)
		return zero_start;

	bit_start = zero_end + 1;

	if (zero_end + 1 >= max_slots)
		return max_slots;
	goto next;
}

void f2fs_update_dentry(nid_t ino, umode_t mode, struct f2fs_dentry_ptr *d,
				const struct qstr *name, f2fs_hash_t name_hash,
				unsigned int bit_pos)
{
	struct f2fs_dir_entry *de;
	int slots = GET_DENTRY_SLOTS(name->len);
	int i;

	de = &d->dentry[bit_pos];
	de->hash_code = name_hash;
	de->name_len = cpu_to_le16(name->len);
	memcpy(d->filename[bit_pos], name->name, name->len);
	de->ino = cpu_to_le32(ino);
	set_de_type(de, mode);
	for (i = 0; i < slots; i++) {
		test_and_set_bit_le(bit_pos + i, (void *)d->bitmap);
		/* avoid wrong garbage data for readdir */
		if (i)
			(de + i)->name_len = 0;
	}
}

int __f2fs_do_add_link(struct inode *dir, struct fscrypt_name *fname,
				struct inode *inode, nid_t ino, umode_t mode)
{
	unsigned int bit_pos;
	unsigned int level;
	unsigned int current_depth;
	unsigned long bidx, block;
	f2fs_hash_t dentry_hash;
	unsigned int nbucket, nblock;
	struct page *dentry_page = NULL;
	struct f2fs_dentry_block *dentry_blk = NULL;
	struct f2fs_dentry_ptr d;
	struct page *page = NULL;
	struct qstr new_name;
	int slots;
	int err = 0;

	new_name.name = fname_name(fname);
	new_name.len = fname_len(fname);

	if (f2fs_has_inline_dentry(dir)) {
		err = f2fs_add_inline_entry(dir, &new_name, fname->usr_fname,
						inode, ino, mode);
		if (!err || err != -EAGAIN)
			goto out;
		else
			err = 0;
	}

	level = 0;
	slots = GET_DENTRY_SLOTS(new_name.len);
	dentry_hash = f2fs_dentry_hash(&new_name);

	current_depth = F2FS_I(dir)->i_current_depth;
	if (F2FS_I(dir)->chash == dentry_hash) {
		level = F2FS_I(dir)->clevel;
		F2FS_I(dir)->chash = 0;
	}

start:
	if (unlikely(current_depth == MAX_DIR_HASH_DEPTH)) {
		err = -ENOSPC;
		goto out;
	}

	/* Increase the depth, if required */
	if (level == current_depth)
		++current_depth;

	nbucket = dir_buckets(level, F2FS_I(dir)->i_dir_level);
	nblock = bucket_blocks(level);

	bidx = dir_block_index(level, F2FS_I(dir)->i_dir_level,
				(le32_to_cpu(dentry_hash) % nbucket));

	for (block = bidx; block <= (bidx + nblock - 1); block++) {
		dentry_page = get_new_data_page(dir, NULL, block, true);
		if (IS_ERR(dentry_page)) {
			err = PTR_ERR(dentry_page);
			goto out;
		}

		dentry_blk = kmap(dentry_page);
		bit_pos = room_for_filename(&dentry_blk->dentry_bitmap,
						slots, NR_DENTRY_IN_BLOCK);
		if (bit_pos < NR_DENTRY_IN_BLOCK)
			goto add_dentry;

		kunmap(dentry_page);
		f2fs_put_page(dentry_page, 1);
	}

	/* Move to next level to find the empty slot for new dentry */
	++level;
	goto start;
add_dentry:
	f2fs_wait_on_page_writeback(dentry_page, DATA, true);

	if (inode) {
		down_write(&F2FS_I(inode)->i_sem);
		page = init_inode_metadata(inode, dir, &new_name,
							fname->usr_fname, NULL);
		if (IS_ERR(page)) {
			err = PTR_ERR(page);
			goto fail;
		}
		if (f2fs_encrypted_inode(dir))
			file_set_enc_name(inode);
	}

	make_dentry_ptr(NULL, &d, (void *)dentry_blk, 1);
	f2fs_update_dentry(ino, mode, &d, &new_name, dentry_hash, bit_pos);

	set_page_dirty(dentry_page);

	if (inode) {
		/* we don't need to mark_inode_dirty now */
		F2FS_I(inode)->i_pino = dir->i_ino;
		update_inode(inode, page);
		f2fs_put_page(page, 1);
	}

	update_parent_metadata(dir, inode, current_depth);
fail:
	if (inode)
		up_write(&F2FS_I(inode)->i_sem);

	if (is_inode_flag_set(F2FS_I(dir), FI_UPDATE_DIR)) {
		update_inode_page(dir);
		clear_inode_flag(F2FS_I(dir), FI_UPDATE_DIR);
	}
	kunmap(dentry_page);
	f2fs_put_page(dentry_page, 1);
out:
	return err;
}

/*
 * Caller should grab and release a rwsem by calling f2fs_lock_op() and
 * f2fs_unlock_op().
 */
int __f2fs_add_link(struct inode *dir, const struct qstr *name,
				struct inode *inode, nid_t ino, umode_t mode)
{
	struct fscrypt_name fname;
	int err;

	err = fscrypt_setup_filename(dir, name, 0, &fname);
	if (err)
		return err;

	err = __f2fs_do_add_link(dir, &fname, inode, ino, mode);

	fscrypt_free_filename(&fname);
	f2fs_update_time(F2FS_I_SB(dir), REQ_TIME);
	return err;
}


int f2fs_do_tmpfile(struct inode *inode, struct inode *dir)
{
	struct page *page;
	int err = 0;

	down_write(&F2FS_I(inode)->i_sem);
	page = init_inode_metadata(inode, dir, NULL, NULL, NULL);
	if (IS_ERR(page)) {
		err = PTR_ERR(page);
		goto fail;
	}
	/* we don't need to mark_inode_dirty now */
	update_inode(inode, page);
	f2fs_put_page(page, 1);

	clear_inode_flag(F2FS_I(inode), FI_NEW_INODE);
fail:
	up_write(&F2FS_I(inode)->i_sem);
	f2fs_update_time(F2FS_I_SB(inode), REQ_TIME);
	return err;
}

void f2fs_drop_nlink(struct inode *dir, struct inode *inode, struct page *page)
{
	struct f2fs_sb_info *sbi = F2FS_I_SB(dir);

	down_write(&F2FS_I(inode)->i_sem);

	if (S_ISDIR(inode->i_mode)) {
		drop_nlink(dir);
		if (page)
			update_inode(dir, page);
		else
			update_inode_page(dir);
	}
	inode->i_ctime = CURRENT_TIME;

	drop_nlink(inode);
	if (S_ISDIR(inode->i_mode)) {
		drop_nlink(inode);
		i_size_write(inode, 0);
		set_inode_flag(F2FS_I(inode), FI_ISIZE_CHANGED);
	}
	up_write(&F2FS_I(inode)->i_sem);
	update_inode_page(inode);

	if (inode->i_nlink == 0)
		add_orphan_inode(sbi, inode->i_ino);
	else
		release_orphan_inode(sbi);
}

/*
 * It only removes the dentry from the dentry page, corresponding name
 * entry in name page does not need to be touched during deletion.
 */
void f2fs_delete_entry(struct f2fs_dir_entry *dentry, struct page *page,
					struct inode *dir, struct inode *inode)
{
	struct	f2fs_dentry_block *dentry_blk;
	unsigned int bit_pos;
	int slots = GET_DENTRY_SLOTS(le16_to_cpu(dentry->name_len));
	int i;

	f2fs_update_time(F2FS_I_SB(dir), REQ_TIME);

	if (f2fs_has_inline_dentry(dir))
		return f2fs_delete_inline_entry(dentry, page, dir, inode);

	lock_page(page);
	f2fs_wait_on_page_writeback(page, DATA, true);

	dentry_blk = page_address(page);
	bit_pos = dentry - dentry_blk->dentry;
	for (i = 0; i < slots; i++)
		test_and_clear_bit_le(bit_pos + i, &dentry_blk->dentry_bitmap);

	/* Let's check and deallocate this dentry page */
	bit_pos = find_next_bit_le(&dentry_blk->dentry_bitmap,
			NR_DENTRY_IN_BLOCK,
			0);
	kunmap(page); /* kunmap - pair of f2fs_find_entry */
	set_page_dirty(page);

	dir->i_ctime = dir->i_mtime = CURRENT_TIME;

	if (inode)
		f2fs_drop_nlink(dir, inode, NULL);

	if (bit_pos == NR_DENTRY_IN_BLOCK &&
			!truncate_hole(dir, page->index, page->index + 1)) {
		clear_page_dirty_for_io(page);
		ClearPagePrivate(page);
		ClearPageUptodate(page);
		inode_dec_dirty_pages(dir);
	}
	f2fs_put_page(page, 1);
}

bool f2fs_empty_dir(struct inode *dir)
{
	unsigned long bidx;
	struct page *dentry_page;
	unsigned int bit_pos;
	struct f2fs_dentry_block *dentry_blk;
	unsigned long nblock = dir_blocks(dir);

	if (f2fs_has_inline_dentry(dir))
		return f2fs_empty_inline_dir(dir);

	for (bidx = 0; bidx < nblock; bidx++) {
		dentry_page = get_lock_data_page(dir, bidx, false);
		if (IS_ERR(dentry_page)) {
			if (PTR_ERR(dentry_page) == -ENOENT)
				continue;
			else
				return false;
		}

		dentry_blk = kmap_atomic(dentry_page);
		if (bidx == 0)
			bit_pos = 2;
		else
			bit_pos = 0;
		bit_pos = find_next_bit_le(&dentry_blk->dentry_bitmap,
						NR_DENTRY_IN_BLOCK,
						bit_pos);
		kunmap_atomic(dentry_blk);

		f2fs_put_page(dentry_page, 1);

		if (bit_pos < NR_DENTRY_IN_BLOCK)
			return false;
	}
	return true;
}

bool f2fs_fill_dentries(struct dir_context *ctx, struct f2fs_dentry_ptr *d,
			unsigned int start_pos, struct fscrypt_str *fstr)
{
	unsigned char d_type = DT_UNKNOWN;
	unsigned int bit_pos;
	struct f2fs_dir_entry *de = NULL;
	struct fscrypt_str de_name = FSTR_INIT(NULL, 0);

	bit_pos = ((unsigned long)ctx->pos % d->max);

	while (bit_pos < d->max) {
		bit_pos = find_next_bit_le(d->bitmap, d->max, bit_pos);
		if (bit_pos >= d->max)
			break;

		de = &d->dentry[bit_pos];
		if (de->name_len == 0) {
			bit_pos++;
			ctx->pos = start_pos + bit_pos;
			continue;
		}

		if (de->file_type < F2FS_FT_MAX)
			d_type = f2fs_filetype_table[de->file_type];
		else
			d_type = DT_UNKNOWN;

		de_name.name = d->filename[bit_pos];
		de_name.len = le16_to_cpu(de->name_len);

		if (f2fs_encrypted_inode(d->inode)) {
			int save_len = fstr->len;
			int ret;

			de_name.name = kmalloc(de_name.len, GFP_NOFS);
			if (!de_name.name)
				return false;

			memcpy(de_name.name, d->filename[bit_pos], de_name.len);

			ret = fscrypt_fname_disk_to_usr(d->inode,
						(u32)de->hash_code, 0,
						&de_name, fstr);
			kfree(de_name.name);
			if (ret < 0)
				return true;

			de_name = *fstr;
			fstr->len = save_len;
		}

		if (!dir_emit(ctx, de_name.name, de_name.len,
					le32_to_cpu(de->ino), d_type))
			return true;

		bit_pos += GET_DENTRY_SLOTS(le16_to_cpu(de->name_len));
		ctx->pos = start_pos + bit_pos;
	}
	return false;
}

static int f2fs_readdir(struct file *file, struct dir_context *ctx)
{
	struct inode *inode = file_inode(file);
	unsigned long npages = dir_blocks(inode);
	struct f2fs_dentry_block *dentry_blk = NULL;
	struct page *dentry_page = NULL;
	struct file_ra_state *ra = &file->f_ra;
	unsigned int n = ((unsigned long)ctx->pos / NR_DENTRY_IN_BLOCK);
	struct f2fs_dentry_ptr d;
	struct fscrypt_str fstr = FSTR_INIT(NULL, 0);
	int err = 0;

	if (f2fs_encrypted_inode(inode)) {
		err = fscrypt_get_encryption_info(inode);
		if (err && err != -ENOKEY)
			return err;

		err = fscrypt_fname_alloc_buffer(inode, F2FS_NAME_LEN, &fstr);
		if (err < 0)
			return err;
	}

	if (f2fs_has_inline_dentry(inode)) {
		err = f2fs_read_inline_dir(file, ctx, &fstr);
		goto out;
	}

	/* readahead for multi pages of dir */
	if (npages - n > 1 && !ra_has_index(ra, n))
		page_cache_sync_readahead(inode->i_mapping, ra, file, n,
				min(npages - n, (pgoff_t)MAX_DIR_RA_PAGES));

	for (; n < npages; n++) {
		dentry_page = get_lock_data_page(inode, n, false);
		if (IS_ERR(dentry_page)) {
			err = PTR_ERR(dentry_page);
			if (err == -ENOENT)
				continue;
			else
				goto out;
		}

		dentry_blk = kmap(dentry_page);

		make_dentry_ptr(inode, &d, (void *)dentry_blk, 1);

		if (f2fs_fill_dentries(ctx, &d, n * NR_DENTRY_IN_BLOCK, &fstr)) {
			kunmap(dentry_page);
			f2fs_put_page(dentry_page, 1);
			break;
		}

		ctx->pos = (n + 1) * NR_DENTRY_IN_BLOCK;
		kunmap(dentry_page);
		f2fs_put_page(dentry_page, 1);
	}
	err = 0;
out:
	fscrypt_fname_free_buffer(&fstr);
	return err;
}

static int f2fs_dir_open(struct inode *inode, struct file *filp)
{
	if (f2fs_encrypted_inode(inode))
		return fscrypt_get_encryption_info(inode) ? -EACCES : 0;
	return 0;
}

const struct file_operations f2fs_dir_operations = {
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.iterate	= f2fs_readdir,
	.fsync		= f2fs_sync_file,
	.open		= f2fs_dir_open,
	.unlocked_ioctl	= f2fs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = f2fs_compat_ioctl,
#endif
};

static int f2fs_d_revalidate(struct dentry *dentry, unsigned int flags)
{
	if ((flags & LOOKUP_CASE_INSENSITIVE) && !d_inode_rcu(dentry))
		return 0;
	return 1;
}

#ifdef CONFIG_F2FS_FS_ENCRYPTION
static int f2fs_fscrypt_d_revalidate(struct dentry *dentry, unsigned int flags)
{
	struct dentry *dir;
	struct fscrypt_info *ci;
	int dir_has_key, cached_with_key;

	if (flags & LOOKUP_RCU)
		return -ECHILD;

	if ((flags & LOOKUP_CASE_INSENSITIVE) && !dentry->d_inode)
		return 0;

	/*
	 * Validate dentries for encrypted directories to make sure we aren't
	 * potentially caching stale data after a key has been added or
	 * removed.
	 *
	 * The following code is copied from
	 *    fs/crypto/crypto.c:fscrypt_d_revalidate
	 */
	dir = dget_parent(dentry);
	if (!d_inode(dir)->i_sb->s_cop->is_encrypted(d_inode(dir))) {
		dput(dir);
		return 0;
	}

	ci = d_inode(dir)->i_crypt_info;
	if (ci && ci->ci_keyring_key &&
	    (ci->ci_keyring_key->flags & ((1 << KEY_FLAG_INVALIDATED) |
					  (1 << KEY_FLAG_REVOKED) |
					  (1 << KEY_FLAG_DEAD))))
		ci = NULL;

	/* this should eventually be an flag in d_flags */
	spin_lock(&dentry->d_lock);
	cached_with_key = dentry->d_flags & DCACHE_ENCRYPTED_WITH_KEY;
	spin_unlock(&dentry->d_lock);
	dir_has_key = (ci != NULL);
	dput(dir);

	/*
	 * If the dentry was cached without the key, and it is a
	 * negative dentry, it might be a valid name.  We can't check
	 * if the key has since been made available due to locking
	 * reasons, so we fail the validation so ext4_lookup() can do
	 * this check.
	 *
	 * We also fail the validation if the dentry was created with
	 * the key present, but we no longer have the key, or vice versa.
	 */
	if ((!cached_with_key && d_is_negative(dentry)) ||
			(!cached_with_key && dir_has_key) ||
			(cached_with_key && !dir_has_key))
		return 0;
	return 1;
}

/*lint -save -e785 */
const struct dentry_operations f2fs_fscrypt_dops = {
	.d_revalidate	= f2fs_fscrypt_d_revalidate,
};
/*lint -restore*/
#endif

const struct dentry_operations f2fs_ci_dops = {
	.d_revalidate	= f2fs_d_revalidate,
};
