/**
 * <rc/pru.h>
 *
 * @brief      Start and stop the PRU from userspace.
 *
 * This is primarily for the PRU-dependent servo, motor PWM and encoder functions to use,
 * however the user may elect to use their own PRU routines separately from
 * those.
 *
 * @addtogroup PRU
 * @{
 */

#ifndef RC_PRU_H
#define RC_PRU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * Starts a single specified PRU core running a provided firmware name.
 *
 * This function requires special permissions and your custom firmware must exist in
 * "/lib/firmware/". Example default names for the firmwares are "am335x-pru0-fw"
 * and "am335x-pru1-fw", please don't overwrite these if they exist. Name your
 * firmware image something like "am335x-pru0-mycustom-fw".
 *
 * On BeagleBone Black and other AM3358-based systems, there are 2 PRU cores in
 * a single PRU sub-system. The indexes for these cores are 0 and 1.
 *
 * On BeagleBone AI and other systems with multiple PRU sub-systems, the indexes
 * start at 2 and increment by 1 between sub-systems. For example, the second PRU
 * in the second PRU sub-system would have the index 5.
 *
 * @param[in]  ch       PRU core to start
 * @param[in]  fw_name  The firmware image name, e.g. "am335x-pru0-fw", do not
 * include '/lib/firmware' in the path, only the file name.
 *
 * @return     0 on success, -1 on failure.
 */
int rc_pru_start(int ch, const char* fw_name);

/**
 * @brief      fetches a pointer to the beginning of the PRU shared memory.
 *
 * This is done by mapping to /dev/uio/X for the shared memory region of the
 * PRU sub-system and therefore requires special permissions, but provides 
 * extremely low-latency memory access to communicate with the PRU.
 *
 * NOTE: this memory pointer will point to the same memory for all cores within the
 * same PRU sub-system.
 *
 * @param[in]  ch       PRU core to reference
 *
 * @return     memory pointer on success, NULL on failure
 */
volatile uint32_t* rc_pru_shared_mem_ptr(int ch);


/**
 * Unloads PRU binaries
 *
 * @param[in]  ch    PRU core to stop
 *
 * @return     0 on success, -1 on failure.
 */
int rc_pru_stop(int ch);


#ifdef __cplusplus
}
#endif

#endif // RC_PRU_H

/** @} end group PRU */
