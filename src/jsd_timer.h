#ifndef JSD_TIMER_H_
#define JSD_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

// Standard headers
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

// for declaring overrun faults
#define JSD_MAX_CYCLE_OVERRUNS_DEFAULT 25
#define JSD_TIMER_OVERRUN_FAULT -2
#define JSD_TIMER_NSEC_PER_SEC 1000000000L

/**
 * \struct jsd_timer_t
 * \brief Defines the process time attributes
 */
typedef struct {
  struct timespec loop_period_ns;  ///> loop period in nanoseconds
  struct timespec last_loop_time;  ///> loop time from last control cycle
  uint8_t max_cycle_overruns;  ///> how many cycles is tolerable to overrun by
                               /// before decalring a fault
} jsd_timer_t;

/// if cpu arg of jsd_timer_init() is set to this value
/// it let's the OS pick which CPU the process will run on
enum { JSD_TIMER_ANY_CPU = INT16_MAX };

/**
 * \brief Allocates memory for the jsd_timer_t struct, which is "self".
 * Asserts if memory allocation fails
 * return allocated jsd_timer_t struct pointer
 */
jsd_timer_t* jsd_timer_alloc(void);

/**
 * \brief Wrapper to thetimer_initinit_ex function with use_mlockall set to true
 * \param[in,out] self Main jsd_timer_t struct pointer
 * \param[in] loop_period_ns Desired loop time in nanoseconds
 * \param[in] cpu Processor to isolate the calling process to
 * \param[in] use_root If true, the calling process is set to realtime priority
 * and isolated to the desired core \return 0 on success or fail
 */
int jsd_timer_init(jsd_timer_t* self, uint32_t loop_period_ns, int16_t cpu,
                   bool use_root);

/**
 * \brief Sets the loop period, if use_root is true then it locks current and
 * future virtual address space into RAM, sets the realtime priority of the
 * calling process to 99 (requires a soft or hard realtime kernel), sets the
 * scheduler to use FIFO scheduling, and isolates the calling process to the
 * desire core. \param[in,out] self Main jsd_timer_t struct pointer \param[in]
 * loop_period_ns Desired loop time in nanoseconds \param[in] cpu Processor to
 * isolate the calling process to \param[in] use_root If true, the calling
 * process is set to realtime priority and isolated to the desired core
 * \param[in] use_mlockall If true, locks the process's current and future
 * virtual address space into RAM \return 0 on success. -1 on failure
 */
int jsd_timer_init_ex(
    jsd_timer_t* self, uint32_t loop_period_ns, int16_t cpu, bool use_root,
    bool use_mlockall);  // can optionally select whether to use mlockall

/**
 * \brief Sets the max allowable number of consecutive overruns before faulting
 * \param[in,out] self Main jsd_timer_t struct pointer
 * \param[in] max_cycle_overruns Max allowable number of consecutive overruns
 * \return. void. Asserts is self in NULL
 */
void jsd_timer_set_max_cycle_ovveruns(
    jsd_timer_t* self,
    uint8_t      max_cycle_overruns);  // can optionally set the max number of
                                  // allowable cycle overruns before faulting.
                                  // default is set in jsd_timer_init(_ex), so
                                  // call after that if desired

/**
 * \brief Main function that gets called cyclicly by cyclic process.
 * Sleeps for appropriate amount of time in order to maintain the set loop
 * period. Uses CLOCK_MONOTONIC and clock_nanosleep \param[in,out] self Main
 * jsd_timer_t struct pointer \return 0 on success. -1 if the loop period was
 * overrun, -2 if overran more than self->max_cycle_overruns
 */
int jsd_timer_process(jsd_timer_t* self);

/**
 * \brief Get system time in seconds
 * @return system time in seconds as a double
 */
double jsd_timer_get_time_sec();

/**
 * \brief Get system time in milliseconds
 * @return system time in milliseconds as a double
 */
double jsd_timer_get_time_msec();

/**
 * \brief Get system time in microseconds
 * @return system time in microseconds as a double
 */
double jsd_timer_get_time_usec();

/**
 * \brief Get system time in nanoseconds
 * @return system time in nanoseconds as a double
 */
double jsd_timer_get_time_nsec();

/**
 * \brief Frees the previously allocated memory for jsd_timer_t struct (self)
 * \param[in,out] self Main jsd_timer_t struct pointer
 * \return void
 */
void jsd_timer_free(jsd_timer_t* self);

#ifdef __cplusplus
}
#endif

#endif  // JSD_TIMER_H_
