/**
 * <rc/cpu.h>
 *
 * @brief      Control CPU scaling governer
 *
 * Functions to read and set the current CPU scaling function. This is not
 * specific to the beaglebone and should work on any linux system, however since
 * the beaglebone has a single cpu core, this only changes the govenor for one
 * core.
 *
 * @author     James Strawson
 * @date       3/7/2018
 *
 * @addtogroup CPU
 * @{
 */

#ifndef RC_CPU_H
#define RC_CPU_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * available CPU governors
 */
typedef enum rc_governor_t{
	RC_GOV_POWERSAVE,	///< Sets CPU to slowest speed
	RC_GOV_PERFORMANCE,	///< Sets CPU to fastest speed
	RC_GOV_ONDEMAND,	///< Default automatic scaling
	RC_GOV_SCHEDUTIL,	///< Like ONDEMAND but newer algorithm
	RC_GOV_CONSERVATIVE	///< Automatically scales the cpu but still tries to save power
} rc_governor_t;

/**
 * @brief      Sets the CPU governor. See rc_governor_t
 *
 * This is the equivalent to running 'cpufreq-set -g {governor}' from the
 * command line but can be called in your C program instead.
 *
 * @param[in]  gov   Desired governor
 *
 * @return     0 on success, -1 on failure.
 */
int rc_cpu_set_governor(rc_governor_t gov);

/**
 * @brief      Returns the current clock speed of the Beaglebone's Sitara
 * processor in the form of the provided enumerated type. It will never return
 * the FREQ_ONDEMAND value as the intention of this function is to see the clock
 * speed as set by either the user or the ondemand governor itself.
 *
 * @return     frequency in hz
 */
int rc_cpu_get_freq(void);

/**
 * @brief      Prints the current frequency to the screen. For example "600mhz".
 *
 * @return     0 on success or -1 on failure.
 */
int rc_cpu_print_freq(void);



#ifdef __cplusplus
}
#endif

#endif // RC_CPU_H

/** @} end group CPU */