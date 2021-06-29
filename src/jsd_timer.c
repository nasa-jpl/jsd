/**
 * Copyright (C) 2013, California Institute of Technology.
 * All Rights Reserved. U.S. Government Sponsorship Acknowledged.
 * Any commercial use must be negotiated with the Office of
 * Technology Transfer at the California Institute of Technology.
 */

/**
 * \file  jsd_timer.c
 * \brief jsd_timer implementation

 * \date  02/08/2013

 * \authors
 *   Daniel Helmick (dhelmick@jsd.nasa.gov)
 *   Peter Vieira (pevieira@jsd.nasa.gov)
 * Mobility and Robotic Systems (347), JPL
*/

// Standard C Headers
#define _GNU_SOURCE

#include <assert.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

// Local headers
#include "jsd_timer.h"

// Timer helper macros
/// Number of nanoseconds in a second
#define JSD_NSEC_PER_SEC (1000000000L)

/// Subtract nanoseconds from nanoseconds
#define JSD_DIFF_NS(A, B)                                                 \
  ((int64_t)(((B).tv_sec - (A).tv_sec) * JSD_NSEC_PER_SEC + (B).tv_nsec - \
             (A).tv_nsec))

static struct timespec jsd_timer_timespec_add(struct timespec time1,
                                              struct timespec time2);

static int jsd_timer_setup_process(jsd_timer_t* self, int16_t cpu,
                                   bool use_mlockall);

jsd_timer_t* jsd_timer_alloc(void) {
  jsd_timer_t* self;

  self = (jsd_timer_t*)calloc(1, sizeof(jsd_timer_t));
  assert(self);

  return self;
}

int jsd_timer_init(jsd_timer_t* self, uint32_t loop_period_ns, int16_t cpu,
                   bool use_root) {
  return jsd_timer_init_ex(self, loop_period_ns, cpu, use_root, true);
}

int jsd_timer_init_ex(jsd_timer_t* self, uint32_t loop_period_ns, int16_t cpu,
                      bool use_root, bool use_mlockall) {
  self->loop_period_ns.tv_nsec = loop_period_ns;

  if (use_root) {
    jsd_timer_setup_process(self, cpu, use_mlockall);
  }

  // init loop jsd_timer
  clock_gettime(CLOCK_MONOTONIC, &self->last_loop_time);

  // set default overrun cycles
  self->max_cycle_overruns = JSD_MAX_CYCLE_OVERRUNS_DEFAULT;

  return 0;
}

void jsd_timer_set_max_cycle_overruns(jsd_timer_t* self,
                                      uint8_t      max_cycle_overruns) {
  assert(self != NULL);

  self->max_cycle_overruns = max_cycle_overruns;
}

int jsd_timer_process(jsd_timer_t* self) {
  struct timespec curr_time;
  struct timespec wakeup_time;
  int64_t         ns_left_in_cycle;
  int             status = 0;

  // calculate the time that the next cycle needs to start
  wakeup_time =
      jsd_timer_timespec_add(self->last_loop_time, self->loop_period_ns);

  // make sure we didn't overrun the last period
  clock_gettime(CLOCK_MONOTONIC, &curr_time);
  ns_left_in_cycle = JSD_DIFF_NS(curr_time, wakeup_time);

  if (ns_left_in_cycle < 0) {
    fprintf(stderr, "The jsd_timer loop overran by %f microseconds!!\n",
            (double)(ns_left_in_cycle) / -1.0e3);

    status = -1;

    // if overrun exceeds max number of allowable cycles, set status to
    // ecat_overrun_fault
    if (-1 * ns_left_in_cycle >
        (int64_t)(self->max_cycle_overruns *
                  (self->loop_period_ns.tv_sec * JSD_NSEC_PER_SEC +
                   self->loop_period_ns.tv_nsec))) {
      fprintf(stderr, "jsd_timer has exceeded its trip limit!!\n");
      status = JSD_TIMER_OVERRUN_FAULT;
    }
    if (-1 * ns_left_in_cycle >
        (int64_t)(self->loop_period_ns.tv_sec * JSD_NSEC_PER_SEC +
                  self->loop_period_ns.tv_nsec)) {
      fprintf(stderr,
              "jsd_timer overrun larger than loop_period, reinitializing "
              "last_loop_time\n");
      // We overran by more than the loop_period so in order not to fire off a
      // bunch of processes let's re-init the last_loop_time
      clock_gettime(CLOCK_MONOTONIC, &self->last_loop_time);
      wakeup_time =
          jsd_timer_timespec_add(self->last_loop_time, self->loop_period_ns);
    }
  }

  // set the last loop time for the next cycle
  self->last_loop_time = wakeup_time;

  // sleep until an absolute time using the monotonic clock
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);

  return status;
}

static int jsd_timer_setup_process(jsd_timer_t* self, int16_t cpu,
                                   bool use_mlockall) {
  (void)self;
  // lock this process's virtual address space into RAM
  if (use_mlockall) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
      fprintf(stderr, "mlockall failed\n");
      return -1;
    }
  }

  // make this a realtime task and set its priority
  pid_t              pid = getpid();
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = 99;  // 99 is highest realtime priority
  if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
    fprintf(stderr, "sched_setscheduler failed\n");
    return -1;
  }

  // migrate this process (pid=0) to the specified CPU
  if (cpu != JSD_TIMER_ANY_CPU) {
    cpu_set_t ded_mask;
    CPU_ZERO(&ded_mask);
    CPU_SET(cpu, &ded_mask);
    if (sched_setaffinity(0, sizeof(ded_mask), &ded_mask) == -1) {
      fprintf(stderr, "sched_setaffinity failed\n");
      return -1;
    }
  }

  return 0;
}

static struct timespec jsd_timer_timespec_add(struct timespec time1,
                                              struct timespec time2) {
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= JSD_NSEC_PER_SEC) {
    result.tv_sec  = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - JSD_NSEC_PER_SEC;
  } else {
    result.tv_sec  = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }

  return result;
}

double jsd_timer_get_time_sec() {
  struct timespec sys_time;

  clock_gettime(CLOCK_MONOTONIC, &sys_time);

  return (double)sys_time.tv_sec + (double)sys_time.tv_nsec / 1.0e9;
}

double jsd_timer_get_time_msec() {
  struct timespec sys_time;

  clock_gettime(CLOCK_MONOTONIC, &sys_time);

  return (double)sys_time.tv_sec * 1.0e3 + (double)sys_time.tv_nsec / 1.0e6;
}

double jsd_timer_get_time_usec() {
  struct timespec sys_time;

  clock_gettime(CLOCK_MONOTONIC, &sys_time);

  return (double)sys_time.tv_sec * 1.0e6 + (double)sys_time.tv_nsec / 1.0e3;
}

double jsd_timer_get_time_nsec() {
  struct timespec sys_time;

  clock_gettime(CLOCK_MONOTONIC, &sys_time);

  return (double)sys_time.tv_sec * 1.0e9 + (double)sys_time.tv_nsec;
}

void jsd_timer_free(jsd_timer_t* self) { free(self); }
