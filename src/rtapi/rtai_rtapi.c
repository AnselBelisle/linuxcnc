/** RTAPI is a library providing a uniform API for several real time
    operating systems.  As of ver 2.0, RTLinux and RTAI are supported.
*/

/** This file, 'rtai_rtapi.c', implements the realtime portion of the
    API for the RTAI platform.  The API is defined in rtapi.h, which
    includes documentation for all of the API functions.  The non-
    real-time portion of the API is implemented in rtai_ulapi.c (for
    the RTAI platform).  This implementation attempts to prevent
    kernel panics, 'oops'es, and other nasty stuff that can happen
    when writing and testing realtime code.  Wherever possible,
    common errors are detected and corrected before they can cause a
    crash.  This implementation also includes several /proc filesystem
    entries and numerous debugging print statements.
*/

/** Copyright (C) 2003 John Kasunich
                       <jmkasunich AT users DOT sourceforge DOT net>
    Copyright (C) 2003 Paul Corner
                       <paul_c AT users DOT sourceforge DOT net>
    This library is based on version 1.0, which was released into
    the public domain by its author, Fred Proctor.  Thanks Fred!
*/

/* This library is free software; you can redistribute it and/or
   modify it under the terms of version 2 of the GNU General Public
   License as published by the Free Software Foundation.
   This library is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this library; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111 USA
*/

/** THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the EMC HAL project.  For more
    information, go to www.linuxcnc.org.
*/

#ifndef RTAPI
#error RTAPI must be defined to compile rtai_rtapi.c!
#endif

#include <stdarg.h>		/* va_* */
#include <linux/config.h>	/* need to know about CONFIG_X86_HAS_TSC */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>		/* replaces malloc.h in recent kernels */
#include <linux/ctype.h>	/* isdigit */
#include <linux/delay.h>	/* udelay */
#include <asm/uaccess.h>	/* copy_from_user() */
#include <linux/time.h>		/* timeval & gettimeofday() */

#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#endif
#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif

/* get inb(), outb(), ioperm() */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,17)
#include <asm/io.h>
#else
#include <sys/io.h>
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,0)
/* Kernel is 2.4 or higher, use it's vsnprintf() implementation */
#define vsn_printf vsnprintf
#else
/* 2.2 and older kernels don't have vsnprintf() so we bring in
   our own implementation (vsn_printf) of it here.*/
#include "vsnprintf.h"
#endif

#include <rtai.h>
#include <rtai_sched.h>
#if RTAI > 2
#include <rtai_sem.h>
#endif
#include <rtai_shm.h>
#include <rtai_fifos.h>

#include "rtapi.h"		/* public RTAPI decls */
#include "rtapi_common.h"	/* shared realtime/nonrealtime stuff */

/* resource data unique to kernel space */
static RT_TASK *ostask_array[RTAPI_MAX_TASKS + 1];
static void *shmem_addr_array[RTAPI_MAX_SHMEMS + 1];
static SEM ossem_array[RTAPI_MAX_SEMS + 1];

#define DEFAULT_MAX_DELAY 10000
static long int max_delay = DEFAULT_MAX_DELAY;

/* module parameters */

static int msg_level = RTAPI_MSG_INFO;	/* message printing level */
MODULE_PARM(msg_level, "i");
MODULE_PARM_DESC(msg, "debug message level (default=3)");

/* other module information */
MODULE_AUTHOR("John Kasunich, Fred Proctor, & Paul Corner");
MODULE_DESCRIPTION("Portable Real Time API for RTAI");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

#include "rtapi_proc.h"		/* proc filesystem decls & code */

/* the following are internal functions that do the real work associated
   with deleting tasks, etc.  They do not check the mutex that protects
   the internal data structures.  When someone calls an rtapi_xxx_delete()
   function, the rtapi funct gets the mutex before calling one of these
   internal functions.  When internal code that already has the mutex
   needs to delete something, it calls these functions directly.
*/
static int module_delete(int module_id);
static int task_delete(int task_id);
static int shmem_delete(int shmem_id, int module_id);
static int sem_delete(int sem_id, int module_id);
static int fifo_delete(int fifo_id, int module_id);
static int irq_delete(unsigned int irq_num);

/***********************************************************************
*                   INIT AND SHUTDOWN FUNCTIONS                        *
************************************************************************/

int init_module(void)
{
    int n;

    /* say hello */
    rtapi_print_msg(RTAPI_MSG_INFO, "RTAPI: Init\n");
    /* setup revision string and code, and print opening message */
    setup_revision_info();
    /* get master shared memory block from OS and save its address */
    rtapi_data = rtai_kmalloc(RTAPI_KEY, sizeof(rtapi_data_t));
    if (rtapi_data == NULL) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "RTAPI: ERROR: could not open shared memory\n");
	return RTAPI_NOMEM;
    }
    /* perform a global init if needed */
    init_rtapi_data(rtapi_data);
    /* check revision code */
    if (rtapi_data->rev_code != rev_code) {
	/* mismatch - release master shared memory block */
	rtai_kfree(RTAPI_KEY);
	rtapi_print_msg(RTAPI_MSG_ERR, "RTAPI: ERROR: version mismatch\n");
	return RTAPI_FAIL;
    }
    /* set up local pointers to global data */
    module_array = rtapi_data->module_array;
    task_array = rtapi_data->task_array;
    shmem_array = rtapi_data->shmem_array;
    sem_array = rtapi_data->sem_array;
    fifo_array = rtapi_data->fifo_array;
    irq_array = rtapi_data->irq_array;
    /* perform local init */
    for (n = 0; n <= RTAPI_MAX_TASKS; n++) {
	ostask_array[n] = NULL;
    }
    for (n = 0; n <= RTAPI_MAX_SHMEMS; n++) {
	shmem_addr_array[n] = NULL;
    }
    rtapi_data->timer_running = 0;
    rtapi_data->timer_period = 0;
    max_delay = DEFAULT_MAX_DELAY;
    rt_linux_use_fpu(1);
#ifdef CONFIG_PROC_FS
    /* set up /proc/rtapi */
    if (proc_init() != 0) {
	rtapi_print_msg(RTAPI_MSG_WARN,
	    "RTAPI: WARNING: Could not activate /proc entries\n");
	proc_clean();
    }
#endif
    /* done */
    rtapi_print_msg(RTAPI_MSG_INFO, "RTAPI: Init complete\n");
    return RTAPI_SUCCESS;
}

/* This cleanup code attempts to fix any messes left by modules
that fail to load properly, or fail to clean up after themselves */

void cleanup_module(void)
{
    int n;

    if (rtapi_data == NULL) {
	/* never got inited, nothing to do */
	return;
    }
    /* grab the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    rtapi_print_msg(RTAPI_MSG_INFO, "RTAPI: Exiting\n");

    /* clean up leftover modules (start at 1, we don't use ID 0 */
    for (n = 1; n <= RTAPI_MAX_MODULES; n++) {
	if (module_array[n].state == REALTIME) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' (ID: %02d) did not call rtapi_exit()\n",
		module_array[n].name, n);
	    module_delete(n);
	}
    }
    /* cleaning up modules should clean up everything, if not there has
       probably been an unrecoverable internal error.... */
    for (n = 1; n <= RTAPI_MAX_IRQS; n++) {
	if (irq_array[n].irq_num != 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"RTAPI: ERROR: interrupt handler %02d not deleted (IRQ %d)\n",
		n, irq_array[n].irq_num);
	    /* probably un-recoverable, but try anyway */
	    irq_delete(irq_array[n].irq_num);
	}
    }
    for (n = 1; n <= RTAPI_MAX_FIFOS; n++) {
	if (fifo_array[n].state != UNUSED) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"RTAPI: ERROR: FIFO %02d not deleted\n", n);
	}
    }
    for (n = 1; n <= RTAPI_MAX_SEMS; n++) {
	while (sem_array[n].users > 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"RTAPI: ERROR: semaphore %02d not deleted\n", n);
	}
    }
    for (n = 1; n <= RTAPI_MAX_SHMEMS; n++) {
	if (shmem_array[n].rtusers > 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"RTAPI: ERROR: shared memory block %02d not deleted\n", n);
	}
    }
    for (n = 1; n <= RTAPI_MAX_TASKS; n++) {
	if (task_array[n].state != EMPTY) {
	    rtapi_print_msg(RTAPI_MSG_ERR,
		"RTAPI: ERROR: task %02d not deleted\n", n);
	    /* probably un-recoverable, but try anyway */
	    rtapi_task_pause(n);
	    task_delete(n);
	}
    }
    if (rtapi_data->timer_running != 0) {
	stop_rt_timer();
	rt_free_timer();
	rtapi_data->timer_period = 0;
	rtapi_data->timer_running = 0;
	max_delay = DEFAULT_MAX_DELAY;
    }
    rtapi_mutex_give(&(rtapi_data->mutex));
#ifdef CONFIG_PROC_FS
    proc_clean();
#endif
    /* release master shared memory block */
    rtai_kfree(RTAPI_KEY);
    rtapi_print_msg(RTAPI_MSG_INFO, "RTAPI: Exit complete\n");
    return;
}

/***********************************************************************
*                   GENERAL PURPOSE FUNCTIONS                          *
************************************************************************/

/* all RTAPI init is done when the rtapi kernel module
is insmoded.  The rtapi_init() and rtapi_exit() functions
simply register that another module is using the RTAPI.
For other RTOSes, things might be different, especially
if the RTOS does not use modules. */

int rtapi_init(char *modname)
{
    int n, module_id;
    module_data *module;

    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: initing module %s\n", modname);
    /* get the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    /* find empty spot in module array */
    n = 1;
    while ((n <= RTAPI_MAX_MODULES) && (module_array[n].state != NO_MODULE)) {
	n++;
    }
    if (n > RTAPI_MAX_MODULES) {
	/* no room */
	rtapi_mutex_give(&(rtapi_data->mutex));
	rtapi_print_msg(RTAPI_MSG_ERR, "RTAPI: ERROR: reached module limit\n",
	    n);
	return RTAPI_LIMIT;
    }
    /* we have space for the module */
    module_id = n;
    module = &(module_array[n]);
    /* update module data */
    module->state = REALTIME;
    if (modname != NULL) {
	/* use name supplied by caller, truncating if needed */
	rtapi_snprintf(module->name, RTAPI_NAME_LEN, "%s", modname);
    } else {
	/* make up a name */
	rtapi_snprintf(module->name, RTAPI_NAME_LEN, "RTMOD%03d", module_id);
    }
    rtapi_data->rt_module_count++;
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: module '%s' loaded, ID: %d\n",
	module->name, module_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return module_id;
}

int rtapi_exit(int module_id)
{
    int retval;

    rtapi_mutex_get(&(rtapi_data->mutex));
    retval = module_delete(module_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return retval;
}

static int module_delete(int module_id)
{
    module_data *module;
    char name[RTAPI_NAME_LEN + 1];
    int n;

    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: module %d exiting\n", module_id);
    /* validate module ID */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	return RTAPI_BADID;
    }
    /* point to the module's data */
    module = &(module_array[module_id]);
    /* check module status */
    if (module->state != REALTIME) {
	/* not an active realtime module */
	return RTAPI_INVAL;
    }
    /* clean up any mess left behind by the module */
    for (n = 1; n <= RTAPI_MAX_TASKS; n++) {
	if ((task_array[n].state != EMPTY)
	    && (task_array[n].owner == module_id)) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' failed to delete task %02d\n",
		module->name, n);
	    task_delete(n);
	}
    }
    for (n = 1; n <= RTAPI_MAX_SHMEMS; n++) {
	if (test_bit(module_id, shmem_array[n].bitmap)) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' failed to delete shmem %02d\n",
		module->name, n);
	    shmem_delete(n, module_id);
	}
    }
    for (n = 1; n <= RTAPI_MAX_SEMS; n++) {
	if (test_bit(module_id, sem_array[n].bitmap)) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' failed to delete sem %02d\n",
		module->name, n);
	    sem_delete(n, module_id);
	}
    }
    for (n = 1; n <= RTAPI_MAX_FIFOS; n++) {
	if ((fifo_array[n].reader == module_id) ||
	    (fifo_array[n].writer == module_id)) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' failed to delete fifo %02d\n",
		module->name, n);
	    fifo_delete(n, module_id);
	}
    }
    for (n = 1; n <= RTAPI_MAX_IRQS; n++) {
	if (irq_array[n].owner == module_id) {
	    rtapi_print_msg(RTAPI_MSG_WARN,
		"RTAPI: WARNING: module '%s' failed to delete handler for IRQ %d\n",
		module->name, irq_array[n].irq_num);
	    irq_delete(irq_array[n].irq_num);
	}
    }
    /* use snprintf() to do strncpy(), since we don't have string.h */
    rtapi_snprintf(name, RTAPI_NAME_LEN, "%s", module->name);
    /* update module data */
    module->state = NO_MODULE;
    module->name[0] = '\0';
    rtapi_data->rt_module_count--;
    if (rtapi_data->rt_module_count == 0) {
	if (rtapi_data->timer_running != 0) {
	    stop_rt_timer();
	    rt_free_timer();
	    rtapi_data->timer_period = 0;
	    max_delay = DEFAULT_MAX_DELAY;
	    rtapi_data->timer_running = 0;
	}
    }
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: module %d exited, name: '%s'\n",
	module_id, name);
    return RTAPI_SUCCESS;
}

int rtapi_snprintf(char *buf, unsigned long int size, const char *fmt, ...)
{
    va_list args;
    int i;

    va_start(args, fmt);
    /* call our own vsn_printf(), which is #defined to vsnprintf() if the
       kernel supplies one. */
    i = vsn_printf(buf, size, fmt, args);
    va_end(args);
    return i;
}

#define BUFFERLEN 80

void rtapi_print(const char *fmt, ...)
{
    char buffer[BUFFERLEN];
    va_list args;

    va_start(args, fmt);
    /* call our own vsn_printf(), which is #defined to vsnprintf() if the
       kernel supplies one. */
    vsn_printf(buffer, BUFFERLEN, fmt, args);
    rt_printk(buffer);
    va_end(args);
}

void rtapi_print_msg(int level, const char *fmt, ...)
{
    char buffer[BUFFERLEN];
    va_list args;

    if ((level <= msg_level) && (msg_level != RTAPI_MSG_NONE)) {
	va_start(args, fmt);
	/* call our own vsn_printf(), which is #defined to vsnprintf() if the 
	   kernel supplies one. */
	vsn_printf(buffer, BUFFERLEN, fmt, args);
	rt_printk(buffer);
	va_end(args);
    }
}

int rtapi_set_msg_level(int level)
{
    if ((level < RTAPI_MSG_NONE) || (level > RTAPI_MSG_ALL)) {
	return RTAPI_INVAL;
    }
    msg_level = level;
    return RTAPI_SUCCESS;
}

int rtapi_get_msg_level(void)
{
    return msg_level;
}

/***********************************************************************
*                     CLOCK RELATED FUNCTIONS                          *
************************************************************************/

long int rtapi_clock_set_period(long int nsecs)
{

    if (nsecs == 0) {
	/* it's a query, not a command */
	return rtapi_data->timer_period;
    }
    if (rtapi_data->timer_running) {
	/* already started, can't restart */
	return RTAPI_INVAL;
    }
    /* limit period to 2 micro-seconds min, 1 second max */
    if ((nsecs < 2000) || (nsecs > 1000000000L)) {
	return RTAPI_INVAL;
    }
    rt_set_periodic_mode();
    rtapi_data->timer_period =
	count2nano(start_rt_timer(nano2count((RTIME) nsecs)));

    rtapi_print_msg(RTAPI_MSG_DBG,
	"RTAPI: clock_set_period requested: %ld  actual: %ld\n",
	nsecs, rtapi_data->timer_period);
    rtapi_data->timer_running = 1;
    max_delay = rtapi_data->timer_period / 4;
    return rtapi_data->timer_period;
}

long long int rtapi_get_time(void)
{
#ifdef CONFIG_X86_HAS_TSC
    /* This will only work if the kernel is compiled for a 586TSC
       (Pentium-Classic) or higher. Should return a true time in nSec or zero 
       if no TSC. */
    return rt_get_cpu_time_ns();
#else
    struct timeval tv;
    /* Whilst this one only gives uSec resolution at best and may be blocking 
     */
    do_gettimeofday(&tv);
    return (tv.tv_sec * 1000000000LL + tv.tv_usec * 1000LL);
#endif
}

void rtapi_delay(long int nsec)
{
    if (nsec > max_delay) {
	nsec = max_delay;
    }
    udelay(nsec / 1000);
}

long int rtapi_delay_max(void)
{
    return max_delay;
}

/***********************************************************************
*                     TASK RELATED FUNCTIONS                           *
************************************************************************/

/* Priority functions.  RTAI uses 0 as the highest priority, as the
number increases, the actual priority of the task decreases. */

int rtapi_prio_highest(void)
{
    return 0;
}

int rtapi_prio_lowest(void)
{
    /* RTAI has LOTS of different priorities - RT_LOWEST_PRIORITY is
       0x3FFFFFFF! I don't want such ugly numbers, and we only need a few
       levels, so we use 0xFFF (4095) instead */
    return 0xFFF;
}

int rtapi_prio_next_higher(int prio)
{
    /* return a valid priority for out of range arg */
    if (prio <= rtapi_prio_highest()) {
	return rtapi_prio_highest();
    }
    if (prio > rtapi_prio_lowest()) {
	return rtapi_prio_lowest();
    }

    /* return next higher priority for in-range arg */
    return prio - 1;
}

int rtapi_prio_next_lower(int prio)
{
    /* return a valid priority for out of range arg */
    if (prio >= rtapi_prio_lowest()) {
	return rtapi_prio_lowest();
    }
    if (prio < rtapi_prio_highest()) {
	return rtapi_prio_highest();
    }
    /* return next lower priority for in-range arg */
    return prio + 1;
}

/* We define taskcode as taking a void pointer and returning void, but
   rtai wants it to take an int and return void.
   We solve this with a wrapper function that meets rtai's needs.
   The wrapper functions also properly deals with tasks that return.
   (Most tasks are infinite loops, and don't return.)
*/

static void wrapper(int task_id)
{
    task_data *task;

    /* point to the task data */
    task = &task_array[task_id];
    /* call the task function with the task argument */
    (task->taskcode) (task->arg);
    /* if the task ever returns, we record that fact */
    task->state = ENDED;
    /* and return to end the thread */
    return;
}

int rtapi_task_new(void (*taskcode) (void *), void *arg,
    int prio, int owner, unsigned long int stacksize, int uses_fp)
{
    int n;
    int task_id;
    int retval;
    task_data *task;

    /* get the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    /* validate owner */
    if ((owner < 1) || (owner > RTAPI_MAX_MODULES)) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    if (module_array[owner].state != REALTIME) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    /* find empty spot in task array */
    n = 1;
    while ((n <= RTAPI_MAX_TASKS) && (task_array[n].state != EMPTY)) {
	n++;
    }
    if (n > RTAPI_MAX_TASKS) {
	/* no room */
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_LIMIT;
    }
    /* we have space for the task */
    task_id = n;
    task = &(task_array[n]);
    /* check requested priority */
    if ((prio < rtapi_prio_highest()) || (prio > rtapi_prio_lowest())) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    /* get space for the OS's task data - this is around 900 bytes, */
    /* so we don't want to statically allocate it for unused tasks. */
    ostask_array[task_id] = kmalloc(sizeof(RT_TASK), GFP_USER);
    if (ostask_array[task_id] == NULL) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_NOMEM;
    }
    task->taskcode = taskcode;
    task->arg = arg;
    /* call OS to initialize the task - use CPU 0 (the only CPU if
       uni-processor, but I want predictable behavior under SMP) */
    retval = rt_task_init_cpuid(ostask_array[task_id], wrapper, task_id, stacksize, prio, uses_fp, 0	/* signal 
	 */ , 0 /* cpu id */ );
    if (retval != 0) {
	/* couldn't create task, free task data memory */
	kfree(ostask_array[task_id]);
	rtapi_mutex_give(&(rtapi_data->mutex));
	if (retval == ENOMEM) {
	    /* not enough space for stack */
	    return RTAPI_NOMEM;
	}
	/* unknown error */
	return RTAPI_FAIL;
    }
    /* the task has been created, update data */
    task->state = PAUSED;
    task->prio = prio;
    task->owner = owner;
    task->taskcode = taskcode;
    rtapi_data->task_count++;
    /* announce the birth of a brand new baby task */
    rtapi_print_msg(RTAPI_MSG_DBG,
	"RTAPI: task %02d installed by module %02d, priority %d, code: %p\n",
	task_id, task->owner, task->prio, taskcode);
    /* and return the ID to the proud parent */
    rtapi_mutex_give(&(rtapi_data->mutex));
    return task_id;
}

int rtapi_task_delete(int task_id)
{
    int retval;

    rtapi_mutex_get(&(rtapi_data->mutex));
    retval = task_delete(task_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return retval;
}

static int task_delete(int task_id)
{
    task_data *task;

    /* validate task ID */
    if ((task_id < 1) || (task_id > RTAPI_MAX_TASKS)) {
	return RTAPI_BADID;
    }
    /* point to the task's data */
    task = &(task_array[task_id]);
    /* check task status */
    if (task->state == EMPTY) {
	/* nothing to delete */
	return RTAPI_INVAL;
    }
    if ((task->state == PERIODIC) || (task->state == FREERUN)) {
	/* task is running, need to stop it */
	rtapi_print_msg(RTAPI_MSG_WARN,
	    "RTAPI: WARNING: tried to delete task %02d while running\n",
	    task_id);
	rtapi_task_pause(task_id);
    }
    /* get rid of it */
    rt_task_delete(ostask_array[task_id]);
    /* free kernel memory */
    kfree(ostask_array[task_id]);
    /* update data */
    task->state = EMPTY;
    task->prio = 0;
    task->owner = 0;
    task->taskcode = NULL;
    ostask_array[task_id] = NULL;
    rtapi_data->task_count--;
    /* if no more tasks, stop the timer */
    if (rtapi_data->task_count == 0) {
	if (rtapi_data->timer_running != 0) {
	    stop_rt_timer();
	    rt_free_timer();
	    rtapi_data->timer_period = 0;
	    max_delay = DEFAULT_MAX_DELAY;
	    rtapi_data->timer_running = 0;
	}
    }
    /* done */
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: task %02d deleted\n", task_id);
    return RTAPI_SUCCESS;
}

int rtapi_task_start(int task_id, unsigned long int period_nsec)
{
    int retval;
    unsigned long int quo, rem;
    task_data *task;

    /* validate task ID */
    if ((task_id < 1) || (task_id > RTAPI_MAX_TASKS)) {
	return RTAPI_BADID;
    }
    /* point to the task's data */
    task = &(task_array[task_id]);
    /* is task ready to be started? */
    if (task->state != PAUSED) {
	return RTAPI_INVAL;
    }
    /* can't start periodic tasks if timer isn't running */
    if ((rtapi_data->timer_running == 0) || (rtapi_data->timer_period == 0)) {
	return RTAPI_FAIL;
    }
    /* make period_nsec an integer multiple of timer_period */
    quo = period_nsec / rtapi_data->timer_period;
    rem = period_nsec - (quo * rtapi_data->timer_period);
    /* round to nearest */
    if (rem > (rtapi_data->timer_period / 2)) {
	quo++;
    }
    /* must be at least 1 * timer_period */
    if (quo == 0) {
	quo = 1;
    }
    period_nsec = quo * rtapi_data->timer_period;
    /* start the task */
    retval = rt_task_make_periodic(ostask_array[task_id],
	rt_get_time(), nano2count((RTIME) period_nsec));
    if (retval != 0) {
	return RTAPI_FAIL;
    }
    /* ok, task is started */
    task->state = PERIODIC;
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: start_task id: %02d\n", task_id);
    return retval;
}

void rtapi_wait(void)
{
    rt_task_wait_period();
}

int rtapi_task_resume(int task_id)
{
    int retval;
    task_data *task;

    /* validate task ID */
    if ((task_id < 1) || (task_id > RTAPI_MAX_TASKS)) {
	return RTAPI_BADID;
    }
    /* point to the task's data */
    task = &(task_array[task_id]);
    /* is task ready to be started? */
    if (task->state != PAUSED) {
	return RTAPI_INVAL;
    }
    /* start the task */
    retval = rt_task_resume(ostask_array[task_id]);
    if (retval != 0) {
	return RTAPI_FAIL;
    }
    /* update task data */
    task->state = FREERUN;
    return RTAPI_SUCCESS;
}

int rtapi_task_pause(int task_id)
{
    int retval;
    task_data *task;

    /* validate task ID */
    if ((task_id < 1) || (task_id > RTAPI_MAX_TASKS)) {
	return RTAPI_BADID;
    }
    /* point to the task's data */
    task = &(task_array[task_id]);
    /* is it running? */
    if ((task->state != PERIODIC) && (task->state != FREERUN)) {
	return RTAPI_INVAL;
    }
    /* pause the task */
    retval = rt_task_suspend(ostask_array[task_id]);
    if (retval != 0) {
	return RTAPI_FAIL;
    }
    /* update task data */
    task->state = PAUSED;
    return RTAPI_SUCCESS;
}

int rtapi_task_self(void)
{
    RT_TASK *ptr;
    int n;

    /* ask OS for pointer to its data for the current task */
    ptr = rt_whoami();
    if (ptr == NULL) {
	/* called from outside a task? */
	return RTAPI_FAIL;
    }
    /* find matching entry in task array */
    n = 1;
    while (n <= RTAPI_MAX_TASKS) {
	if (ostask_array[n] == ptr) {
	    /* found a match */
	    return n;
	}
	n++;
    }
    return RTAPI_FAIL;
}

/***********************************************************************
*                  SHARED MEMORY RELATED FUNCTIONS                     *
************************************************************************/

int rtapi_shmem_new(int key, int module_id, unsigned long int size)
{
    int n;
    int shmem_id;
    shmem_data *shmem;

    /* key must be non-zero, and also cannot match the key that RTAPI uses */
    if ((key == 0) || (key == RTAPI_KEY)) {
	return RTAPI_INVAL;
    }
    /* get the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    /* validate module_id */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    if (module_array[module_id].state != REALTIME) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    /* check if a block is already open for this key */
    for (n = 1; n <= RTAPI_MAX_SHMEMS; n++) {
	if (shmem_array[n].key == key) {
	    /* found a match */
	    shmem_id = n;
	    shmem = &(shmem_array[n]);
	    /* is it big enough? */
	    if (shmem->size < size) {
		rtapi_mutex_give(&(rtapi_data->mutex));
		return RTAPI_FAIL;
	    }
	    /* yes, has it been mapped into kernel space? */
	    if (shmem->rtusers == 0) {
		/* no, map it and save the address */
		shmem_addr_array[shmem_id] = rtai_kmalloc(key, shmem->size);
		if (shmem_addr_array[shmem_id] == NULL) {
		    rtapi_mutex_give(&(rtapi_data->mutex));
		    return RTAPI_NOMEM;
		}
	    }
	    /* is this module already using it? */
	    if (test_bit(module_id, shmem->bitmap)) {
		rtapi_mutex_give(&(rtapi_data->mutex));
		return RTAPI_INVAL;
	    }
	    /* update usage data */
	    set_bit(module_id, shmem->bitmap);
	    shmem->rtusers++;
	    /* announce another user for this shmem */
	    rtapi_print_msg(RTAPI_MSG_DBG,
		"RTAPI: shmem %02d opened by module %02d\n",
		shmem_id, module_id);
	    rtapi_mutex_give(&(rtapi_data->mutex));
	    return shmem_id;
	}
    }
    /* find empty spot in shmem array */
    n = 1;
    while ((n <= RTAPI_MAX_SHMEMS) && (shmem_array[n].key != 0)) {
	n++;
    }
    if (n > RTAPI_MAX_SHMEMS) {
	/* no room */
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_LIMIT;
    }
    /* we have space for the block data */
    shmem_id = n;
    shmem = &(shmem_array[n]);
    /* get shared memory block from OS and save its address */
    shmem_addr_array[shmem_id] = rtai_kmalloc(key, size);
    if (shmem_addr_array[shmem_id] == NULL) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_NOMEM;
    }
    /* the block has been created, update data */
    set_bit(module_id, shmem->bitmap);
    shmem->key = key;
    shmem->rtusers = 1;
    shmem->ulusers = 0;
    shmem->size = size;
    rtapi_data->shmem_count++;
    /* announce the birth of a brand new baby shmem */
    rtapi_print_msg(RTAPI_MSG_DBG,
	"RTAPI: shmem %02d created by module %02d, key: %d, size: %lu\n",
	shmem_id, module_id, key, size);
    /* and return the ID to the proud parent */
    rtapi_mutex_give(&(rtapi_data->mutex));
    return shmem_id;
}

int rtapi_shmem_delete(int shmem_id, int module_id)
{
    int retval;

    rtapi_mutex_get(&(rtapi_data->mutex));
    retval = shmem_delete(shmem_id, module_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return retval;
}

static int shmem_delete(int shmem_id, int module_id)
{
    shmem_data *shmem;

    /* validate shmem ID */
    if ((shmem_id < 1) || (shmem_id > RTAPI_MAX_SHMEMS)) {
	return RTAPI_BADID;
    }
    /* point to the shmem's data */
    shmem = &(shmem_array[shmem_id]);
    /* is the block valid? */
    if (shmem->key == 0) {
	return RTAPI_BADID;
    }
    /* validate module_id */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	return RTAPI_INVAL;
    }
    if (module_array[module_id].state != REALTIME) {
	return RTAPI_INVAL;
    }
    /* is this module using the block? */
    if (test_bit(module_id, shmem->bitmap) == 0) {
	return RTAPI_INVAL;
    }
    /* OK, we're no longer using it */
    clear_bit(module_id, shmem->bitmap);
    shmem->rtusers--;
    /* is somebody else still using the block? */
    if (shmem->rtusers > 0) {
	/* yes, we're done for now */
	rtapi_print_msg(RTAPI_MSG_DBG,
	    "RTAPI: shmem %02d closed by module %02d\n", shmem_id, module_id);
	return RTAPI_SUCCESS;
    }
    /* no other realtime users, free the shared memory from kernel space */
    rtai_kfree(shmem->key);
    shmem_addr_array[shmem_id] = NULL;
    shmem->rtusers = 0;
    /* are any user processes using the block? */
    if (shmem->ulusers > 0) {
	/* yes, we're done for now */
	rtapi_print_msg(RTAPI_MSG_DBG,
	    "RTAPI: shmem %02d unmapped by module %02d\n", shmem_id,
	    module_id);
	return RTAPI_SUCCESS;
    }
    /* no other users at all, this ID is now free */
    /* update the data array and usage count */
    shmem->key = 0;
    shmem->size = 0;
    rtapi_data->shmem_count--;
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: shmem %02d freed by module %02d\n",
	shmem_id, module_id);
    return RTAPI_SUCCESS;
}

int rtapi_shmem_getptr(int shmem_id, void **ptr)
{
    /* validate shmem ID */
    if ((shmem_id < 1) || (shmem_id > RTAPI_MAX_SHMEMS)) {
	return RTAPI_BADID;
    }
    /* is the block mapped? */
    if (shmem_addr_array[shmem_id] == NULL) {
	return RTAPI_BADID;
    }
    /* pass memory address back to caller */
    *ptr = shmem_addr_array[shmem_id];
    return RTAPI_SUCCESS;
}

/***********************************************************************
*                    SEMAPHORE RELATED FUNCTIONS                       *
************************************************************************/

int rtapi_sem_new(int key, int module_id)
{
    int n;
    int sem_id;
    sem_data *sem;

    /* key must be non-zero */
    if (key == 0) {
	return RTAPI_INVAL;
    }
    /* get the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    /* validate module_id */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    if (module_array[module_id].state != REALTIME) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    /* check if a semaphore already exists for this key */
    for (n = 1; n <= RTAPI_MAX_SEMS; n++) {
	if ((sem_array[n].users > 0) && (sem_array[n].key == key)) {
	    /* found a match */
	    sem_id = n;
	    sem = &(sem_array[n]);
	    /* is this module already using it? */
	    if (test_bit(module_id, sem->bitmap)) {
		/* yes, can't open it again */
		rtapi_mutex_give(&(rtapi_data->mutex));
		return RTAPI_INVAL;
	    }
	    /* update usage data */
	    set_bit(module_id, sem->bitmap);
	    sem->users++;
	    /* announce another user for this semaphore */
	    rtapi_print_msg(RTAPI_MSG_DBG,
		"RTAPI: sem %02d opened by module %02d\n", sem_id, module_id);
	    rtapi_mutex_give(&(rtapi_data->mutex));
	    return sem_id;
	}
    }
    /* find empty spot in sem array */
    n = 1;
    while ((n <= RTAPI_MAX_SEMS) && (sem_array[n].users != 0)) {
	n++;
    }
    if (n > RTAPI_MAX_SEMS) {
	/* no room */
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_LIMIT;
    }
    /* we have space for the semaphore */
    sem_id = n;
    sem = &(sem_array[n]);
    /* ask the OS to initialize the semaphore */
    rt_sem_init(&(ossem_array[sem_id]), 0);
    /* the semaphore has been created, update data */
    set_bit(module_id, sem->bitmap);
    sem->users = 1;
    sem->key = key;
    rtapi_data->sem_count++;
    /* announce the birth of a brand new baby semaphore */
    rtapi_print_msg(RTAPI_MSG_DBG,
	"RTAPI: sem %02d created by module %02d, key: %d\n",
	sem_id, module_id, key);
    /* and return the ID to the proud parent */
    rtapi_mutex_give(&(rtapi_data->mutex));
    return sem_id;
}

int rtapi_sem_delete(int sem_id, int module_id)
{
    int retval;

    rtapi_mutex_get(&(rtapi_data->mutex));
    retval = sem_delete(sem_id, module_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return retval;
}

static int sem_delete(int sem_id, int module_id)
{
    sem_data *sem;

    /* validate sem ID */
    if ((sem_id < 1) || (sem_id > RTAPI_MAX_SEMS)) {
	return RTAPI_BADID;
    }
    /* point to the semaphores's data */
    sem = &(sem_array[sem_id]);
    /* is the semaphore valid? */
    if (sem->users == 0) {
	return RTAPI_BADID;
    }
    /* validate module_id */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	return RTAPI_INVAL;
    }
    if (module_array[module_id].state != REALTIME) {
	return RTAPI_INVAL;
    }
    /* is this module using the semaphore? */
    if (test_bit(module_id, sem->bitmap) == 0) {
	return RTAPI_INVAL;
    }
    /* OK, we're no longer using it */
    clear_bit(module_id, sem->bitmap);
    sem->users--;
    /* is somebody else still using the semaphore */
    if (sem->users > 0) {
	/* yes, we're done for now */
	rtapi_print_msg(RTAPI_MSG_DBG,
	    "RTAPI: sem %02d closed by module %02d\n", sem_id, module_id);
	return RTAPI_SUCCESS;
    }
    /* no other users, ask the OS to shut down the semaphore */
    rt_sem_delete(&(ossem_array[sem_id]));
    /* update the data array and usage count */
    sem->users = 0;
    sem->key = 0;
    rtapi_data->sem_count--;
    rtapi_print_msg(RTAPI_MSG_DBG, "RTAPI: sem %02d deleted by module %02d\n",
	sem_id, module_id);
    return RTAPI_SUCCESS;
}

int rtapi_sem_give(int sem_id)
{
    sem_data *sem;

    /* validate sem ID */
    if ((sem_id < 1) || (sem_id > RTAPI_MAX_SEMS)) {
	return RTAPI_BADID;
    }
    /* point to the semaphores's data */
    sem = &(sem_array[sem_id]);
    /* is the semaphore valid? */
    if (sem->users == 0) {
	return RTAPI_BADID;
    }
    /* give up the semaphore */
    rt_sem_signal(&(ossem_array[sem_id]));
    return RTAPI_SUCCESS;
}

int rtapi_sem_take(int sem_id)
{
    sem_data *sem;

    /* validate sem ID */
    if ((sem_id < 1) || (sem_id > RTAPI_MAX_SEMS)) {
	return RTAPI_BADID;
    }
    /* point to the semaphores's data */
    sem = &(sem_array[sem_id]);
    /* is the semaphore valid? */
    if (sem->users == 0) {
	return RTAPI_BADID;
    }
    /* get the semaphore */
    rt_sem_wait(&(ossem_array[sem_id]));
    return RTAPI_SUCCESS;
}

int rtapi_sem_try(int sem_id)
{
    sem_data *sem;

    /* validate sem ID */
    if ((sem_id < 1) || (sem_id > RTAPI_MAX_SEMS)) {
	return RTAPI_BADID;
    }
    /* point to the semaphores's data */
    sem = &(sem_array[sem_id]);
    /* is the semaphore valid? */
    if (sem->users == 0) {
	return RTAPI_BADID;
    }
    /* try the semaphore */
    if (rt_sem_wait_if(&(ossem_array[sem_id])) <= 0) {
	return RTAPI_BUSY;
    }
    return RTAPI_SUCCESS;
}

/***********************************************************************
*                        FIFO RELATED FUNCTIONS                        *
************************************************************************/

int rtapi_fifo_new(int key, int module_id, unsigned long int size, char mode)
{
    int n, retval;
    int fifo_id;
    fifo_data *fifo;

    /* key must be non-zero */
    if (key == 0) {
	return RTAPI_INVAL;
    }
    /* mode must be "R" or "W" */
    if ((mode != 'R') && (mode != 'W')) {
	return RTAPI_INVAL;
    }
    /* get the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    /* validate module_id */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    if (module_array[module_id].state != REALTIME) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    /* check if a fifo already exists for this key */
    for (n = 1; n <= RTAPI_MAX_FIFOS; n++) {
	if ((fifo_array[n].state != UNUSED) && (fifo_array[n].key == key)) {
	    /* found a match */
	    fifo_id = n;
	    fifo = &(fifo_array[n]);
	    /* is the desired mode available */
	    if (mode == 'R') {
		if (fifo->state & HAS_READER) {
		    rtapi_mutex_give(&(rtapi_data->mutex));
		    return RTAPI_BUSY;
		}
		/* available, update status */
		fifo->state |= HAS_READER;
		fifo->reader = module_id;
		/* announce */
		rtapi_print_msg(RTAPI_MSG_DBG,
		    "RTAPI: fifo %02d opened for read by module %02d\n",
		    fifo_id, module_id);
		rtapi_mutex_give(&(rtapi_data->mutex));
		return fifo_id;
	    } else {		/* mode == 'W' */

		if (fifo->state & HAS_WRITER) {
		    rtapi_mutex_give(&(rtapi_data->mutex));
		    return RTAPI_BUSY;
		}
		/* available, update status */
		fifo->state |= HAS_WRITER;
		fifo->writer = module_id;
		/* announce */
		rtapi_print_msg(RTAPI_MSG_DBG,
		    "RTAPI: fifo %02d opened for write by module %02d\n",
		    fifo_id, module_id);
		rtapi_mutex_give(&(rtapi_data->mutex));
		return fifo_id;
	    }
	}
    }
    /* find empty spot in fifo array */
    n = 1;
    while ((n <= RTAPI_MAX_FIFOS) && (fifo_array[n].state != UNUSED)) {
	n++;
    }
    if (n > RTAPI_MAX_FIFOS) {
	/* no room */
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_LIMIT;
    }
    /* we have a free ID for the fifo */
    fifo_id = n;
    fifo = &(fifo_array[n]);
    /* create the fifo */
    retval = rtf_create(fifo_id, size);
    /* rtf_create() returns 0 on success */
    if (retval != 0) {
	/* create failed */
	rtapi_mutex_give(&(rtapi_data->mutex));
	if (retval == ENOMEM) {
	    /* couldn't allocate memory */
	    return RTAPI_NOMEM;
	}
	/* some other failure */
	return RTAPI_FAIL;
    }
    /* the fifo has been created, update data */
    if (mode == 'R') {
	fifo->state = HAS_READER;
	fifo->reader = module_id;
	rtapi_print_msg(RTAPI_MSG_DBG,
	    "RTAPI: fifo %02d created for read by module %02d, key: %d, size: %ld\n",
	    fifo_id, module_id, key, size);
    } else {			/* mode == 'W' */

	fifo->state = HAS_WRITER;
	fifo->writer = module_id;
	rtapi_print_msg(RTAPI_MSG_DBG,
	    "RTAPI: fifo %02d created for write by module %02d, key: %d, size: %ld\n",
	    fifo_id, module_id, key, size);
    }
    fifo->key = key;
    fifo->size = size;
    rtapi_data->fifo_count++;
    /* and return the ID */
    rtapi_mutex_give(&(rtapi_data->mutex));
    return fifo_id;
}

int rtapi_fifo_delete(int fifo_id, int module_id)
{
    int retval;

    rtapi_mutex_get(&(rtapi_data->mutex));
    retval = fifo_delete(fifo_id, module_id);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return retval;
}

static int fifo_delete(int fifo_id, int module_id)
{
    fifo_data *fifo;

    /* validate fifo ID */
    if ((fifo_id < 1) || (fifo_id > RTAPI_MAX_FIFOS)) {
	return RTAPI_BADID;
    }
    /* point to the fifo's data */
    fifo = &(fifo_array[fifo_id]);
    /* is the fifo valid? */
    if (fifo->state == UNUSED) {
	return RTAPI_BADID;
    }
    /* validate module_id */
    if ((module_id < 1) || (module_id > RTAPI_MAX_MODULES)) {
	return RTAPI_INVAL;
    }
    if (module_array[module_id].state != REALTIME) {
	return RTAPI_INVAL;
    }
    /* is this module using the fifo? */
    if ((fifo->reader != module_id) && (fifo->writer != module_id)) {
	return RTAPI_INVAL;
    }
    /* update fifo state */
    if (fifo->reader == module_id) {
	fifo->state &= ~HAS_READER;
	fifo->reader = 0;
    }
    if (fifo->writer == module_id) {
	fifo->state &= ~HAS_WRITER;
	fifo->writer = 0;
    }
    /* is somebody else still using the fifo */
    if (fifo->state != UNUSED) {
	/* yes, done for now */
	rtapi_print_msg(RTAPI_MSG_DBG,
	    "RTAPI: fifo %02d closed by module %02d\n", fifo_id, module_id);
	return RTAPI_SUCCESS;
    }
    /* no other users, call the OS to destroy the fifo */
    /* OS returns open count, loop until truly destroyed */
    while (rtf_destroy(fifo_id) > 0);
    /* update the data array and usage count */
    fifo->state = UNUSED;
    fifo->key = 0;
    fifo->size = 0;
    rtapi_data->fifo_count--;
    rtapi_print_msg(RTAPI_MSG_DBG,
	"RTAPI: fifo %02d deleted by module %02d\n", fifo_id, module_id);
    return RTAPI_SUCCESS;
}

int rtapi_fifo_read(int fifo_id, char *buf, unsigned long int size)
{
    int retval;
    fifo_data *fifo;

    /* validate fifo ID */
    if ((fifo_id < 1) || (fifo_id > RTAPI_MAX_FIFOS)) {
	return RTAPI_BADID;
    }
    /* point to the fifo's data */
    fifo = &(fifo_array[fifo_id]);
    /* is the fifo valid? */
    if ((fifo->state & HAS_READER) == 0) {
	return RTAPI_BADID;
    }
    /* get whatever data is available */
    retval = rtf_get(fifo_id, &buf, size);
    if (retval < 0) {
	return RTAPI_FAIL;
    }
    return retval;
}

int rtapi_fifo_write(int fifo_id, char *buf, unsigned long int size)
{
    int retval;
    fifo_data *fifo;

    /* validate fifo ID */
    if ((fifo_id < 1) || (fifo_id > RTAPI_MAX_FIFOS)) {
	return RTAPI_BADID;
    }
    /* point to the fifo's data */
    fifo = &(fifo_array[fifo_id]);
    /* is the fifo valid? */
    if ((fifo->state & HAS_WRITER) == 0) {
	return RTAPI_BADID;
    }
    /* put as much data as possible */
    retval = rtf_put(fifo_id, buf, size);
    if (retval < 0) {
	return RTAPI_INVAL;
    }
    return retval;
}

/***********************************************************************
*                    INTERRUPT RELATED FUNCTIONS                       *
************************************************************************/

int rtapi_irq_new(unsigned int irq_num, int owner, void (*handler) (void))
{
    int n, retval;
    int irq_id;
    irq_data *irq;

    /* validate irq */
    if ((irq_num < 1) || (irq_num > 255)) {
	return RTAPI_INVAL;
    }
    /* validate handler */
    if (handler == NULL) {
	return RTAPI_INVAL;
    }
    /* get the mutex */
    rtapi_mutex_get(&(rtapi_data->mutex));
    /* validate owner */
    if ((owner < 1) || (owner > RTAPI_MAX_MODULES)) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    if (module_array[owner].state != REALTIME) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_INVAL;
    }
    /* check if a handler already exists for this irq */
    for (n = 1; n <= RTAPI_MAX_IRQS; n++) {
	if (irq_array[n].irq_num == irq_num) {
	    /* found a match */
	    rtapi_mutex_give(&(rtapi_data->mutex));
	    return RTAPI_BUSY;
	}
    }
    /* find empty spot in irq array */
    n = 1;
    while ((n <= RTAPI_MAX_IRQS) && (irq_array[n].irq_num != 0)) {
	n++;
    }
    if (n > RTAPI_MAX_IRQS) {
	/* no room */
	rtapi_mutex_give(&(rtapi_data->mutex));
	return RTAPI_LIMIT;
    }
    /* we have space for the irq */
    irq_id = n;
    irq = &(irq_array[n]);
    /* install the handler */
    retval = rt_request_global_irq(irq_num, handler);
    if (retval != 0) {
	rtapi_mutex_give(&(rtapi_data->mutex));
	if (retval == EBUSY) {
	    return RTAPI_BUSY;
	} else {
	    return RTAPI_FAIL;
	}
    }
    /* update data */
    irq->irq_num = irq_num;
    irq->owner = owner;
    irq->handler = handler;
    rtapi_data->irq_count++;
    /* announce the new interrupt handler */
    rtapi_print_msg(RTAPI_MSG_DBG,
	"RTAPI: handler for IRQ %d installed by module %02d\n",
	irq_num, owner);
    /* and return success */
    rtapi_mutex_give(&(rtapi_data->mutex));
    return RTAPI_SUCCESS;
}

int rtapi_irq_delete(unsigned int irq_num)
{
    int retval;

    rtapi_mutex_get(&(rtapi_data->mutex));
    retval = irq_delete(irq_num);
    rtapi_mutex_give(&(rtapi_data->mutex));
    return retval;
}

static int irq_delete(unsigned int irq_num)
{
    int n, retval;
    int irq_id;
    irq_data *irq;

    /* validate irq */
    if ((irq_num < 1) || (irq_num > 255)) {
	return RTAPI_INVAL;
    }
    /* check if a handler exists for this irq */
    n = 1;
    while ((n <= RTAPI_MAX_IRQS) && (irq_array[n].irq_num != irq_num)) {
	n++;
    }
    if (n > RTAPI_MAX_IRQS) {
	/* not found */
	return RTAPI_INVAL;
    }
    /* found the irq */
    irq_id = n;
    irq = &(irq_array[n]);
    /* get rid of the handler */
    rt_shutdown_irq(irq_num);
    retval = rt_free_global_irq(irq_num);
    if (retval != 0) {
	return RTAPI_FAIL;
    }
    /* update data */
    irq->irq_num = 0;
    irq->owner = 0;
    irq->handler = NULL;
    rtapi_data->irq_count--;
    rtapi_print_msg(RTAPI_MSG_DBG,
	"RTAPI: handler for IRQ %d deleted\n", irq_num);
    return RTAPI_SUCCESS;
}

int rtapi_enable_interrupt(unsigned int irq)
{
    rt_startup_irq(irq);

    return RTAPI_SUCCESS;
}

int rtapi_disable_interrupt(unsigned int irq)
{
    rt_shutdown_irq(irq);

    return RTAPI_SUCCESS;
}

/***********************************************************************
*                        I/O RELATED FUNCTIONS                         *
************************************************************************/

void rtapi_outb(unsigned char byte, unsigned int port)
{
    outb(byte, port);
}

unsigned char rtapi_inb(unsigned int port)
{
    return inb(port);
}
