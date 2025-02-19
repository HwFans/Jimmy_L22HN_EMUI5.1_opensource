/*
 * trace_output.c
 *
 * Copyright (C) 2008 Red Hat Inc, Steven Rostedt <srostedt@redhat.com>
 *
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/ftrace.h>

#include "trace_output.h"

/* must be a power of 2 */
#define EVENT_HASHSIZE	128

DECLARE_RWSEM(trace_event_sem);

static struct hlist_head event_hash[EVENT_HASHSIZE] __read_mostly;

static int next_event_type = __TRACE_LAST_TYPE + 1;

enum print_line_t trace_print_bputs_msg_only(struct trace_iterator *iter)
{
	struct trace_seq *s = &iter->seq;
	struct trace_entry *entry = iter->ent;
	struct bputs_entry *field;
	int ret;

	trace_assign_type(field, entry);

	ret = trace_seq_puts(s, field->str);
	if (!ret)
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

enum print_line_t trace_print_bprintk_msg_only(struct trace_iterator *iter)
{
	struct trace_seq *s = &iter->seq;
	struct trace_entry *entry = iter->ent;
	struct bprint_entry *field;
	int ret;

	trace_assign_type(field, entry);

	ret = trace_seq_bprintf(s, field->fmt, field->buf);
	if (!ret)
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

enum print_line_t trace_print_printk_msg_only(struct trace_iterator *iter)
{
	struct trace_seq *s = &iter->seq;
	struct trace_entry *entry = iter->ent;
	struct print_entry *field;
	int ret;

	trace_assign_type(field, entry);

	ret = trace_seq_puts(s, field->buf);
	if (!ret)
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

const char *
ftrace_print_flags_seq(struct trace_seq *p, const char *delim,
		       unsigned long flags,
		       const struct trace_print_flags *flag_array)
{
	unsigned long mask;
	const char *str;
	const char *ret = trace_seq_buffer_ptr(p);
	int i, first = 1;

	for (i = 0;  flag_array[i].name && flags; i++) {

		mask = flag_array[i].mask;
		if ((flags & mask) != mask)
			continue;

		str = flag_array[i].name;
		flags &= ~mask;
		if (!first && delim)
			trace_seq_puts(p, delim);
		else
			first = 0;
		trace_seq_puts(p, str);
	}

	/* check for left over flags */
	if (flags) {
		if (!first && delim)
			trace_seq_puts(p, delim);
		trace_seq_printf(p, "0x%lx", flags);
	}

	trace_seq_putc(p, 0);

	return ret;
}
EXPORT_SYMBOL(ftrace_print_flags_seq);

const char *
ftrace_print_symbols_seq(struct trace_seq *p, unsigned long val,
			 const struct trace_print_flags *symbol_array)
{
	int i;
	const char *ret = trace_seq_buffer_ptr(p);

	for (i = 0;  symbol_array[i].name; i++) {

		if (val != symbol_array[i].mask)
			continue;

		trace_seq_puts(p, symbol_array[i].name);
		break;
	}

	if (ret == (const char *)(trace_seq_buffer_ptr(p)))
		trace_seq_printf(p, "0x%lx", val);
		
	trace_seq_putc(p, 0);

	return ret;
}
EXPORT_SYMBOL(ftrace_print_symbols_seq);

#if BITS_PER_LONG == 32
const char *
ftrace_print_symbols_seq_u64(struct trace_seq *p, unsigned long long val,
			 const struct trace_print_flags_u64 *symbol_array)
{
	int i;
	const char *ret = trace_seq_buffer_ptr(p);

	for (i = 0;  symbol_array[i].name; i++) {

		if (val != symbol_array[i].mask)
			continue;

		trace_seq_puts(p, symbol_array[i].name);
		break;
	}

	if (ret == (const char *)(trace_seq_buffer_ptr(p)))
		trace_seq_printf(p, "0x%llx", val);

	trace_seq_putc(p, 0);

	return ret;
}
EXPORT_SYMBOL(ftrace_print_symbols_seq_u64);
#endif

const char *
ftrace_print_bitmask_seq(struct trace_seq *p, void *bitmask_ptr,
			 unsigned int bitmask_size)
{
	const char *ret = trace_seq_buffer_ptr(p);

	trace_seq_bitmask(p, bitmask_ptr, bitmask_size * 8);
	trace_seq_putc(p, 0);

	return ret;
}
EXPORT_SYMBOL_GPL(ftrace_print_bitmask_seq);

const char *
ftrace_print_hex_seq(struct trace_seq *p, const unsigned char *buf, int buf_len)
{
	int i;
	const char *ret = trace_seq_buffer_ptr(p);

	for (i = 0; i < buf_len; i++)
		trace_seq_printf(p, "%s%2.2x", i == 0 ? "" : " ", buf[i]);

	trace_seq_putc(p, 0);

	return ret;
}
EXPORT_SYMBOL(ftrace_print_hex_seq);

int ftrace_raw_output_prep(struct trace_iterator *iter,
			   struct trace_event *trace_event)
{
	struct ftrace_event_call *event;
	struct trace_seq *s = &iter->seq;
	struct trace_seq *p = &iter->tmp_seq;
	struct trace_entry *entry;
	int ret;

	event = container_of(trace_event, struct ftrace_event_call, event);
	entry = iter->ent;

	if (entry->type != event->event.type) {
		WARN_ON_ONCE(1);
		return TRACE_TYPE_UNHANDLED;
	}

	trace_seq_init(p);
	ret = trace_seq_printf(s, "%s: ", ftrace_event_name(event));
	if (!ret)
		return TRACE_TYPE_PARTIAL_LINE;

	return 0;
}
EXPORT_SYMBOL(ftrace_raw_output_prep);

static int ftrace_output_raw(struct trace_iterator *iter, char *name,
			     char *fmt, va_list ap)
{
	struct trace_seq *s = &iter->seq;
	int ret;

	ret = trace_seq_printf(s, "%s: ", name);
	if (!ret)
		return TRACE_TYPE_PARTIAL_LINE;

	ret = trace_seq_vprintf(s, fmt, ap);

	if (!ret)
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

int ftrace_output_call(struct trace_iterator *iter, char *name, char *fmt, ...)
{
	va_list ap;
	int ret;

	va_start(ap, fmt);
	ret = ftrace_output_raw(iter, name, fmt, ap);
	va_end(ap);

	return ret;
}
EXPORT_SYMBOL_GPL(ftrace_output_call);

#ifdef CONFIG_KRETPROBES
static inline const char *kretprobed(const char *name)
{
	static const char tramp_name[] = "kretprobe_trampoline";
	int size = sizeof(tramp_name);

	if (strncmp(tramp_name, name, size) == 0)
		return "[unknown/kretprobe'd]";
	return name;
}
#else
static inline const char *kretprobed(const char *name)
{
	return name;
}
#endif /* CONFIG_KRETPROBES */

static int
seq_print_sym_short(struct trace_seq *s, const char *fmt, unsigned long address)
{
#ifdef CONFIG_KALLSYMS
	char str[KSYM_SYMBOL_LEN];
	const char *name;

	kallsyms_lookup(address, NULL, NULL, NULL, str);

	name = kretprobed(str);

	return trace_seq_printf(s, fmt, name);
#endif
	return 1;
}

static int
seq_print_sym_offset(struct trace_seq *s, const char *fmt,
		     unsigned long address)
{
#ifdef CONFIG_KALLSYMS
	char str[KSYM_SYMBOL_LEN];
	const char *name;

	sprint_symbol(str, address);
	name = kretprobed(str);

	return trace_seq_printf(s, fmt, name);
#endif
	return 1;
}

#ifndef CONFIG_64BIT
# define IP_FMT "%08lx"
#else
# define IP_FMT "%016lx"
#endif

int seq_print_user_ip(struct trace_seq *s, struct mm_struct *mm,
		      unsigned long ip, unsigned long sym_flags)
{
	struct file *file = NULL;
	unsigned long vmstart = 0;
	int ret = 1;

	if (s->full)
		return 0;

	if (mm) {
		const struct vm_area_struct *vma;

		down_read(&mm->mmap_sem);
		vma = find_vma(mm, ip);
		if (vma) {
			file = vma->vm_file;
			vmstart = vma->vm_start;
		}
		if (file) {
			ret = trace_seq_path(s, &file->f_path);
			if (ret)
				ret = trace_seq_printf(s, "[+0x%lx]",
						       ip - vmstart);
		}
		up_read(&mm->mmap_sem);
	}
	if (ret && ((sym_flags & TRACE_ITER_SYM_ADDR) || !file))
		ret = trace_seq_printf(s, " <" IP_FMT ">", ip);
	return ret;
}

int
seq_print_userip_objs(const struct userstack_entry *entry, struct trace_seq *s,
		      unsigned long sym_flags)
{
	struct mm_struct *mm = NULL;
	int ret = 1;
	unsigned int i;

	if (trace_flags & TRACE_ITER_SYM_USEROBJ) {
		struct task_struct *task;
		/*
		 * we do the lookup on the thread group leader,
		 * since individual threads might have already quit!
		 */
		rcu_read_lock();
		task = find_task_by_vpid(entry->tgid);
		if (task)
			mm = get_task_mm(task);
		rcu_read_unlock();
	}

	for (i = 0; i < FTRACE_STACK_ENTRIES; i++) {
		unsigned long ip = entry->caller[i];

		if (ip == ULONG_MAX || !ret)
			break;
		if (ret)
			ret = trace_seq_puts(s, " => ");
		if (!ip) {
			if (ret)
				ret = trace_seq_puts(s, "??");
			if (ret)
				ret = trace_seq_putc(s, '\n');
			continue;
		}
		if (!ret)
			break;
		if (ret)
			ret = seq_print_user_ip(s, mm, ip, sym_flags);
		ret = trace_seq_putc(s, '\n');
	}

	if (mm)
		mmput(mm);
	return ret;
}

int
seq_print_ip_sym(struct trace_seq *s, unsigned long ip, unsigned long sym_flags)
{
	int ret;

	if (!ip)
		return trace_seq_putc(s, '0');

	if (sym_flags & TRACE_ITER_SYM_OFFSET)
		ret = seq_print_sym_offset(s, "%s", ip);
	else
		ret = seq_print_sym_short(s, "%s", ip);

	if (!ret)
		return 0;

	if (sym_flags & TRACE_ITER_SYM_ADDR)
		ret = trace_seq_printf(s, " <" IP_FMT ">", ip);
	return ret;
}

/**
 * trace_print_lat_fmt - print the irq, preempt and lockdep fields
 * @s: trace seq struct to write to
 * @entry: The trace entry field from the ring buffer
 *
 * Prints the generic fields of irqs off, in hard or softirq, preempt
 * count.
 */
int trace_print_lat_fmt(struct trace_seq *s, struct trace_entry *entry)
{
	char hardsoft_irq;
	char need_resched;
	char irqs_off;
	int hardirq;
	int softirq;
	int ret;

	hardirq = entry->flags & TRACE_FLAG_HARDIRQ;
	softirq = entry->flags & TRACE_FLAG_SOFTIRQ;

	irqs_off =
		(entry->flags & TRACE_FLAG_IRQS_OFF) ? 'd' :
		(entry->flags & TRACE_FLAG_IRQS_NOSUPPORT) ? 'X' :
		'.';

	switch (entry->flags & (TRACE_FLAG_NEED_RESCHED |
				TRACE_FLAG_PREEMPT_RESCHED)) {
	case TRACE_FLAG_NEED_RESCHED | TRACE_FLAG_PREEMPT_RESCHED:
		need_resched = 'N';
		break;
	case TRACE_FLAG_NEED_RESCHED:
		need_resched = 'n';
		break;
	case TRACE_FLAG_PREEMPT_RESCHED:
		need_resched = 'p';
		break;
	default:
		need_resched = '.';
		break;
	}

	hardsoft_irq =
		(hardirq && softirq) ? 'H' :
		hardirq ? 'h' :
		softirq ? 's' :
		'.';

	if (!trace_seq_printf(s, "%c%c%c",
			      irqs_off, need_resched, hardsoft_irq))
		return 0;

	if (entry->preempt_count)
		ret = trace_seq_printf(s, "%x", entry->preempt_count);
	else
		ret = trace_seq_putc(s, '.');

	return ret;
}

static int
lat_print_generic(struct trace_seq *s, struct trace_entry *entry, int cpu)
{
	char comm[TASK_COMM_LEN];

	trace_find_cmdline(entry->pid, comm);

	if (!trace_seq_printf(s, "%8.8s-%-5d %3d",
			      comm, entry->pid, cpu))
		return 0;

	return trace_print_lat_fmt(s, entry);
}

static unsigned long preempt_mark_thresh_us = 100;

static int
lat_print_timestamp(struct trace_iterator *iter, u64 next_ts)
{
	unsigned long verbose = trace_flags & TRACE_ITER_VERBOSE;
	unsigned long in_ns = iter->iter_flags & TRACE_FILE_TIME_IN_NS;
	unsigned long long abs_ts = iter->ts - iter->trace_buffer->time_start;
	unsigned long long rel_ts = next_ts - iter->ts;
	struct trace_seq *s = &iter->seq;

	if (in_ns) {
		abs_ts = ns2usecs(abs_ts);
		rel_ts = ns2usecs(rel_ts);
	}

	if (verbose && in_ns) {
		unsigned long abs_usec = do_div(abs_ts, USEC_PER_MSEC);
		unsigned long abs_msec = (unsigned long)abs_ts;
		unsigned long rel_usec = do_div(rel_ts, USEC_PER_MSEC);
		unsigned long rel_msec = (unsigned long)rel_ts;

		return trace_seq_printf(
				s, "[%08llx] %ld.%03ldms (+%ld.%03ldms): ",
				ns2usecs(iter->ts),
				abs_msec, abs_usec,
				rel_msec, rel_usec);
	} else if (verbose && !in_ns) {
		return trace_seq_printf(
				s, "[%016llx] %lld (+%lld): ",
				iter->ts, abs_ts, rel_ts);
	} else if (!verbose && in_ns) {
		return trace_seq_printf(
				s, " %4lldus%c: ",
				abs_ts,
				rel_ts > preempt_mark_thresh_us ? '!' :
				  rel_ts > 1 ? '+' : ' ');
	} else { /* !verbose && !in_ns */
		return trace_seq_printf(s, " %4lld: ", abs_ts);
	}
}

int trace_print_context(struct trace_iterator *iter)
{
	struct trace_seq *s = &iter->seq;
	struct trace_entry *entry = iter->ent;
	unsigned long long t;
	unsigned long secs, usec_rem;
	char comm[TASK_COMM_LEN];
	int ret;

	trace_find_cmdline(entry->pid, comm);

	ret = trace_seq_printf(s, "%16s-%-5d [%03d] ",comm, entry->pid, iter->cpu);
	if (!ret)
		return 0;

	if (trace_flags & TRACE_ITER_IRQ_INFO) {
		ret = trace_print_lat_fmt(s, entry);
		if (!ret)
			return 0;
	}

	if (iter->iter_flags & TRACE_FILE_TIME_IN_NS) {
		t = ns2usecs(iter->ts);
		usec_rem = do_div(t, USEC_PER_SEC);
		secs = (unsigned long)t;
		return trace_seq_printf(s, " %5lu.%06lu: ", secs, usec_rem);
	} else
		return trace_seq_printf(s, " %12llu: ", iter->ts);
}

int trace_print_lat_context(struct trace_iterator *iter)
{
	u64 next_ts;
	int ret;
	/* trace_find_next_entry will reset ent_size */
	int ent_size = iter->ent_size;
	struct trace_seq *s = &iter->seq;
	struct trace_entry *entry = iter->ent,
			   *next_entry = trace_find_next_entry(iter, NULL,
							       &next_ts);
	unsigned long verbose = (trace_flags & TRACE_ITER_VERBOSE);

	/* Restore the original ent_size */
	iter->ent_size = ent_size;

	if (!next_entry)
		next_ts = iter->ts;

	if (verbose) {
		char comm[TASK_COMM_LEN];

		trace_find_cmdline(entry->pid, comm);

		ret = trace_seq_printf(
				s, "%16s %5d %3d %d %08x %08lx ",
				comm, entry->pid, iter->cpu, entry->flags,
				entry->preempt_count, iter->idx);
	} else {
		ret = lat_print_generic(s, entry, iter->cpu);
	}

	if (ret)
		ret = lat_print_timestamp(iter, next_ts);

	return ret;
}

static const char state_to_char[] = TASK_STATE_TO_CHAR_STR;

static int task_state_char(unsigned long state)
{
	int bit = state ? __ffs(state) + 1 : 0;

	return bit < sizeof(state_to_char) - 1 ? state_to_char[bit] : '?';
}

/**
 * ftrace_find_event - find a registered event
 * @type: the type of event to look for
 *
 * Returns an event of type @type otherwise NULL
 * Called with trace_event_read_lock() held.
 */
struct trace_event *ftrace_find_event(int type)
{
	struct trace_event *event;
	unsigned key;

	key = type & (EVENT_HASHSIZE - 1);

	hlist_for_each_entry(event, &event_hash[key], node) {
		if (event->type == type)
			return event;
	}

	return NULL;
}

static LIST_HEAD(ftrace_event_list);

static int trace_search_list(struct list_head **list)
{
	struct trace_event *e;
	int last = __TRACE_LAST_TYPE;

	if (list_empty(&ftrace_event_list)) {
		*list = &ftrace_event_list;
		return last + 1;
	}

	/*
	 * We used up all possible max events,
	 * lets see if somebody freed one.
	 */
	list_for_each_entry(e, &ftrace_event_list, list) {
		if (e->type != last + 1)
			break;
		last++;
	}

	/* Did we used up all 65 thousand events??? */
	if ((last + 1) > FTRACE_MAX_EVENT)
		return 0;

	*list = &e->list;
	return last + 1;
}

void trace_event_read_lock(void)
{
	down_read(&trace_event_sem);
}

void trace_event_read_unlock(void)
{
	up_read(&trace_event_sem);
}

/**
 * register_ftrace_event - register output for an event type
 * @event: the event type to register
 *
 * Event types are stored in a hash and this hash is used to
 * find a way to print an event. If the @event->type is set
 * then it will use that type, otherwise it will assign a
 * type to use.
 *
 * If you assign your own type, please make sure it is added
 * to the trace_type enum in trace.h, to avoid collisions
 * with the dynamic types.
 *
 * Returns the event type number or zero on error.
 */
int register_ftrace_event(struct trace_event *event)
{
	unsigned key;
	int ret = 0;

	down_write(&trace_event_sem);

	if (WARN_ON(!event))
		goto out;

	if (WARN_ON(!event->funcs))
		goto out;

	INIT_LIST_HEAD(&event->list);

	if (!event->type) {
		struct list_head *list = NULL;

		if (next_event_type > FTRACE_MAX_EVENT) {

			event->type = trace_search_list(&list);
			if (!event->type)
				goto out;

		} else {
			
			event->type = next_event_type++;
			list = &ftrace_event_list;
		}

		if (WARN_ON(ftrace_find_event(event->type)))
			goto out;

		list_add_tail(&event->list, list);

	} else if (event->type > __TRACE_LAST_TYPE) {
		printk(KERN_WARNING "Need to add type to trace.h\n");
		WARN_ON(1);
		goto out;
	} else {
		/* Is this event already used */
		if (ftrace_find_event(event->type))
			goto out;
	}

	if (event->funcs->trace == NULL)
		event->funcs->trace = trace_nop_print;
	if (event->funcs->raw == NULL)
		event->funcs->raw = trace_nop_print;
	if (event->funcs->hex == NULL)
		event->funcs->hex = trace_nop_print;
	if (event->funcs->binary == NULL)
		event->funcs->binary = trace_nop_print;

	key = event->type & (EVENT_HASHSIZE - 1);

	hlist_add_head(&event->node, &event_hash[key]);

	ret = event->type;
 out:
	up_write(&trace_event_sem);

	return ret;
}
EXPORT_SYMBOL_GPL(register_ftrace_event);

/*
 * Used by module code with the trace_event_sem held for write.
 */
int __unregister_ftrace_event(struct trace_event *event)
{
	hlist_del(&event->node);
	list_del(&event->list);
	return 0;
}

/**
 * unregister_ftrace_event - remove a no longer used event
 * @event: the event to remove
 */
int unregister_ftrace_event(struct trace_event *event)
{
	down_write(&trace_event_sem);
	__unregister_ftrace_event(event);
	up_write(&trace_event_sem);

	return 0;
}
EXPORT_SYMBOL_GPL(unregister_ftrace_event);

/*
 * Standard events
 */

enum print_line_t trace_nop_print(struct trace_iterator *iter, int flags,
				  struct trace_event *event)
{
	if (!trace_seq_printf(&iter->seq, "type: %d\n", iter->ent->type))
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

/* TRACE_FN */
static enum print_line_t trace_fn_trace(struct trace_iterator *iter, int flags,
					struct trace_event *event)
{
	struct ftrace_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	if (!seq_print_ip_sym(s, field->ip, flags))
		goto partial;

	if ((flags & TRACE_ITER_PRINT_PARENT) && field->parent_ip) {
		if (!trace_seq_puts(s, " <-"))
			goto partial;
		if (!seq_print_ip_sym(s,
				      field->parent_ip,
				      flags))
			goto partial;
	}
	if (!trace_seq_putc(s, '\n'))
		goto partial;

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}

static enum print_line_t trace_fn_raw(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_entry *field;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_printf(&iter->seq, "%lx %lx\n",
			      field->ip,
			      field->parent_ip))
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_fn_hex(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	SEQ_PUT_HEX_FIELD_RET(s, field->ip);
	SEQ_PUT_HEX_FIELD_RET(s, field->parent_ip);

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_fn_bin(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	SEQ_PUT_FIELD_RET(s, field->ip);
	SEQ_PUT_FIELD_RET(s, field->parent_ip);

	return TRACE_TYPE_HANDLED;
}

static struct trace_event_functions trace_fn_funcs = {
	.trace		= trace_fn_trace,
	.raw		= trace_fn_raw,
	.hex		= trace_fn_hex,
	.binary		= trace_fn_bin,
};

static struct trace_event trace_fn_event = {
	.type		= TRACE_FN,
	.funcs		= &trace_fn_funcs,
};

/* TRACE_GRAPH_ENT */
static enum print_line_t trace_graph_ent_trace(struct trace_iterator *iter, int flags,
					struct trace_event *event)
{
	struct trace_seq *s = &iter->seq;
	struct ftrace_graph_ent_entry *field;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_puts(s, "graph_ent: func="))
		return TRACE_TYPE_PARTIAL_LINE;

	if (!seq_print_ip_sym(s, field->graph_ent.func, flags))
		return TRACE_TYPE_PARTIAL_LINE;

	if (!trace_seq_puts(s, "\n"))
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_graph_ent_raw(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_graph_ent_entry *field;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_printf(&iter->seq, "%lx %d\n",
			      field->graph_ent.func,
			      field->graph_ent.depth))
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_graph_ent_hex(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_graph_ent_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	SEQ_PUT_HEX_FIELD_RET(s, field->graph_ent.func);
	SEQ_PUT_HEX_FIELD_RET(s, field->graph_ent.depth);

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_graph_ent_bin(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_graph_ent_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	SEQ_PUT_FIELD_RET(s, field->graph_ent.func);
	SEQ_PUT_FIELD_RET(s, field->graph_ent.depth);

	return TRACE_TYPE_HANDLED;
}

static struct trace_event_functions trace_graph_ent_funcs = {
	.trace		= trace_graph_ent_trace,
	.raw		= trace_graph_ent_raw,
	.hex		= trace_graph_ent_hex,
	.binary		= trace_graph_ent_bin,
};

static struct trace_event trace_graph_ent_event = {
	.type		= TRACE_GRAPH_ENT,
	.funcs		= &trace_graph_ent_funcs,
};

/* TRACE_GRAPH_RET */
static enum print_line_t trace_graph_ret_trace(struct trace_iterator *iter, int flags,
					struct trace_event *event)
{
	struct trace_seq *s = &iter->seq;
	struct trace_entry *entry = iter->ent;
	struct ftrace_graph_ret_entry *field;

	trace_assign_type(field, entry);

	if (!trace_seq_puts(s, "graph_ret: func="))
		return TRACE_TYPE_PARTIAL_LINE;

	if (!seq_print_ip_sym(s, field->ret.func, flags))
		return TRACE_TYPE_PARTIAL_LINE;

	if (!trace_seq_puts(s, "\n"))
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_graph_ret_raw(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_graph_ret_entry *field;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_printf(&iter->seq, "%lx %lld %lld %ld %d\n",
			      field->ret.func,
			      field->ret.calltime,
			      field->ret.rettime,
			      field->ret.overrun,
			      field->ret.depth));
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_graph_ret_hex(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_graph_ret_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	SEQ_PUT_HEX_FIELD_RET(s, field->ret.func);
	SEQ_PUT_HEX_FIELD_RET(s, field->ret.calltime);
	SEQ_PUT_HEX_FIELD_RET(s, field->ret.rettime);
	SEQ_PUT_HEX_FIELD_RET(s, field->ret.overrun);
	SEQ_PUT_HEX_FIELD_RET(s, field->ret.depth);

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_graph_ret_bin(struct trace_iterator *iter, int flags,
				      struct trace_event *event)
{
	struct ftrace_graph_ret_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	SEQ_PUT_FIELD_RET(s, field->ret.func);
	SEQ_PUT_FIELD_RET(s, field->ret.calltime);
	SEQ_PUT_FIELD_RET(s, field->ret.rettime);
	SEQ_PUT_FIELD_RET(s, field->ret.overrun);
	SEQ_PUT_FIELD_RET(s, field->ret.depth);

	return TRACE_TYPE_HANDLED;
}

static struct trace_event_functions trace_graph_ret_funcs = {
	.trace		= trace_graph_ret_trace,
	.raw		= trace_graph_ret_raw,
	.hex		= trace_graph_ret_hex,
	.binary		= trace_graph_ret_bin,
};

static struct trace_event trace_graph_ret_event = {
	.type		= TRACE_GRAPH_RET,
	.funcs		= &trace_graph_ret_funcs,
};

/* TRACE_CTX an TRACE_WAKE */
static enum print_line_t trace_ctxwake_print(struct trace_iterator *iter,
					     char *delim)
{
	struct ctx_switch_entry *field;
	char comm[TASK_COMM_LEN];
	int S, T;


	trace_assign_type(field, iter->ent);

	T = task_state_char(field->next_state);
	S = task_state_char(field->prev_state);
	trace_find_cmdline(field->next_pid, comm);
	if (!trace_seq_printf(&iter->seq,
			      " %5d:%3d:%c %s [%03d] %5d:%3d:%c %s\n",
			      field->prev_pid,
			      field->prev_prio,
			      S, delim,
			      field->next_cpu,
			      field->next_pid,
			      field->next_prio,
			      T, comm))
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_ctx_print(struct trace_iterator *iter, int flags,
					 struct trace_event *event)
{
	return trace_ctxwake_print(iter, "==>");
}

static enum print_line_t trace_wake_print(struct trace_iterator *iter,
					  int flags, struct trace_event *event)
{
	return trace_ctxwake_print(iter, "  +");
}

static int trace_ctxwake_raw(struct trace_iterator *iter, char S)
{
	struct ctx_switch_entry *field;
	int T;

	trace_assign_type(field, iter->ent);

	if (!S)
		S = task_state_char(field->prev_state);
	T = task_state_char(field->next_state);
	if (!trace_seq_printf(&iter->seq, "%d %d %c %d %d %d %c\n",
			      field->prev_pid,
			      field->prev_prio,
			      S,
			      field->next_cpu,
			      field->next_pid,
			      field->next_prio,
			      T))
		return TRACE_TYPE_PARTIAL_LINE;

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_ctx_raw(struct trace_iterator *iter, int flags,
				       struct trace_event *event)
{
	return trace_ctxwake_raw(iter, 0);
}

static enum print_line_t trace_wake_raw(struct trace_iterator *iter, int flags,
					struct trace_event *event)
{
	return trace_ctxwake_raw(iter, '+');
}


static int trace_ctxwake_hex(struct trace_iterator *iter, char S)
{
	struct ctx_switch_entry *field;
	struct trace_seq *s = &iter->seq;
	int T;

	trace_assign_type(field, iter->ent);

	if (!S)
		S = task_state_char(field->prev_state);
	T = task_state_char(field->next_state);

	SEQ_PUT_HEX_FIELD_RET(s, field->prev_pid);
	SEQ_PUT_HEX_FIELD_RET(s, field->prev_prio);
	SEQ_PUT_HEX_FIELD_RET(s, S);
	SEQ_PUT_HEX_FIELD_RET(s, field->next_cpu);
	SEQ_PUT_HEX_FIELD_RET(s, field->next_pid);
	SEQ_PUT_HEX_FIELD_RET(s, field->next_prio);
	SEQ_PUT_HEX_FIELD_RET(s, T);

	return TRACE_TYPE_HANDLED;
}

static enum print_line_t trace_ctx_hex(struct trace_iterator *iter, int flags,
				       struct trace_event *event)
{
	return trace_ctxwake_hex(iter, 0);
}

static enum print_line_t trace_wake_hex(struct trace_iterator *iter, int flags,
					struct trace_event *event)
{
	return trace_ctxwake_hex(iter, '+');
}

static enum print_line_t trace_ctxwake_bin(struct trace_iterator *iter,
					   int flags, struct trace_event *event)
{
	struct ctx_switch_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	SEQ_PUT_FIELD_RET(s, field->prev_pid);
	SEQ_PUT_FIELD_RET(s, field->prev_prio);
	SEQ_PUT_FIELD_RET(s, field->prev_state);
	SEQ_PUT_FIELD_RET(s, field->next_pid);
	SEQ_PUT_FIELD_RET(s, field->next_prio);
	SEQ_PUT_FIELD_RET(s, field->next_state);

	return TRACE_TYPE_HANDLED;
}

static struct trace_event_functions trace_ctx_funcs = {
	.trace		= trace_ctx_print,
	.raw		= trace_ctx_raw,
	.hex		= trace_ctx_hex,
	.binary		= trace_ctxwake_bin,
};

static struct trace_event trace_ctx_event = {
	.type		= TRACE_CTX,
	.funcs		= &trace_ctx_funcs,
};

static struct trace_event_functions trace_wake_funcs = {
	.trace		= trace_wake_print,
	.raw		= trace_wake_raw,
	.hex		= trace_wake_hex,
	.binary		= trace_ctxwake_bin,
};

static struct trace_event trace_wake_event = {
	.type		= TRACE_WAKE,
	.funcs		= &trace_wake_funcs,
};

/* TRACE_STACK */

static enum print_line_t trace_stack_print(struct trace_iterator *iter,
					   int flags, struct trace_event *event)
{
	struct stack_entry *field;
	struct trace_seq *s = &iter->seq;
	unsigned long *p;
	unsigned long *end;

	trace_assign_type(field, iter->ent);
	end = (unsigned long *)((long)iter->ent + iter->ent_size);

	if (!trace_seq_puts(s, "<stack trace>\n"))
		goto partial;

	for (p = field->caller; p && *p != ULONG_MAX && p < end; p++) {
		if (!trace_seq_puts(s, " => "))
			goto partial;

		if (!seq_print_ip_sym(s, *p, flags))
			goto partial;
		if (!trace_seq_putc(s, '\n'))
			goto partial;
	}

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}

static struct trace_event_functions trace_stack_funcs = {
	.trace		= trace_stack_print,
};

static struct trace_event trace_stack_event = {
	.type		= TRACE_STACK,
	.funcs		= &trace_stack_funcs,
};

/* TRACE_USER_STACK */
static enum print_line_t trace_user_stack_print(struct trace_iterator *iter,
						int flags, struct trace_event *event)
{
	struct userstack_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_puts(s, "<user stack trace>\n"))
		goto partial;

	if (!seq_print_userip_objs(field, s, flags))
		goto partial;

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}

static struct trace_event_functions trace_user_stack_funcs = {
	.trace		= trace_user_stack_print,
};

static struct trace_event trace_user_stack_event = {
	.type		= TRACE_USER_STACK,
	.funcs		= &trace_user_stack_funcs,
};

/* TRACE_BPUTS */
static enum print_line_t
trace_bputs_print(struct trace_iterator *iter, int flags,
		   struct trace_event *event)
{
	struct trace_entry *entry = iter->ent;
	struct trace_seq *s = &iter->seq;
	struct bputs_entry *field;

	trace_assign_type(field, entry);

	if (!seq_print_ip_sym(s, field->ip, flags))
		goto partial;

	if (!trace_seq_puts(s, ": "))
		goto partial;

	if (!trace_seq_puts(s, field->str))
		goto partial;

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}


static enum print_line_t
trace_bputs_raw(struct trace_iterator *iter, int flags,
		struct trace_event *event)
{
	struct bputs_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_printf(s, ": %lx : ", field->ip))
		goto partial;

	if (!trace_seq_puts(s, field->str))
		goto partial;

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}

static struct trace_event_functions trace_bputs_funcs = {
	.trace		= trace_bputs_print,
	.raw		= trace_bputs_raw,
};

static struct trace_event trace_bputs_event = {
	.type		= TRACE_BPUTS,
	.funcs		= &trace_bputs_funcs,
};

/* TRACE_BPRINT */
static enum print_line_t
trace_bprint_print(struct trace_iterator *iter, int flags,
		   struct trace_event *event)
{
	struct trace_entry *entry = iter->ent;
	struct trace_seq *s = &iter->seq;
	struct bprint_entry *field;

	trace_assign_type(field, entry);

	if (!seq_print_ip_sym(s, field->ip, flags))
		goto partial;

	if (!trace_seq_puts(s, ": "))
		goto partial;

	if (!trace_seq_bprintf(s, field->fmt, field->buf))
		goto partial;

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}


static enum print_line_t
trace_bprint_raw(struct trace_iterator *iter, int flags,
		 struct trace_event *event)
{
	struct bprint_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_printf(s, ": %lx : ", field->ip))
		goto partial;

	if (!trace_seq_bprintf(s, field->fmt, field->buf))
		goto partial;

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}

static struct trace_event_functions trace_bprint_funcs = {
	.trace		= trace_bprint_print,
	.raw		= trace_bprint_raw,
};

static struct trace_event trace_bprint_event = {
	.type		= TRACE_BPRINT,
	.funcs		= &trace_bprint_funcs,
};

/* TRACE_PRINT */
static enum print_line_t trace_print_print(struct trace_iterator *iter,
					   int flags, struct trace_event *event)
{
	struct print_entry *field;
	struct trace_seq *s = &iter->seq;

	trace_assign_type(field, iter->ent);

	if (!seq_print_ip_sym(s, field->ip, flags))
		goto partial;

	if (!trace_seq_printf(s, ": %s", field->buf))
		goto partial;

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}

static enum print_line_t trace_print_raw(struct trace_iterator *iter, int flags,
					 struct trace_event *event)
{
	struct print_entry *field;

	trace_assign_type(field, iter->ent);

	if (!trace_seq_printf(&iter->seq, "# %lx %s", field->ip, field->buf))
		goto partial;

	return TRACE_TYPE_HANDLED;

 partial:
	return TRACE_TYPE_PARTIAL_LINE;
}

static struct trace_event_functions trace_print_funcs = {
	.trace		= trace_print_print,
	.raw		= trace_print_raw,
};

static struct trace_event trace_print_event = {
	.type	 	= TRACE_PRINT,
	.funcs		= &trace_print_funcs,
};


static struct trace_event *events[] __initdata = {
	&trace_fn_event,
	&trace_graph_ent_event,
	&trace_graph_ret_event,
	&trace_ctx_event,
	&trace_wake_event,
	&trace_stack_event,
	&trace_user_stack_event,
	&trace_bputs_event,
	&trace_bprint_event,
	&trace_print_event,
	NULL
};

__init static int init_events(void)
{
	struct trace_event *event;
	int i, ret;

	for (i = 0; events[i]; i++) {
		event = events[i];

		ret = register_ftrace_event(event);
		if (!ret) {
			printk(KERN_WARNING "event %d failed to register\n",
			       event->type);
			WARN_ON_ONCE(1);
		}
	}

	return 0;
}
early_initcall(init_events);
