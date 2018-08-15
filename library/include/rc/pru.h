/**
 * <rc/pru.h>
 *
 * @brief      Start and stop the PRU from userspace.
 *
 * This is primarily for the PRU-dependent servo and encoder functions to use,
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
 * This function requires root privileges and your custom firmware must exist in
 * "/lib/firmware/". The default name for the two firmwares are "am335x-pru0-fw"
 * and "am335x-pru1-fw", please don't overwrite these if they exist. Name your
 * firmware image something like "am335x-pru0-mycustom-fw".
 *
 * @param[in]  ch       pru core to start (0 or 1)
 * @param[in]  fw_name  The firmware image name, e.g. "am335x-pru0-fw", do not
 * include '/lib/firmware' in the path, only the file name.
 *
 * @return     0 on success, -1 on failure.
 */
int rc_pru_start(int ch, const char* fw_name);

/**
 * @brief      fetches a pointer to the beginning of the PRU shared memory.
 *
 * This is done by mapping to /dev/mem and therefore requires root privileges
 * but provides extremely low-latency memory access to communicate with the PRU.
 *
 * @return     memory pointer on success, NULL on failure
 */
volatile uint32_t* rc_pru_shared_mem_ptr(void);


/**
 * Unloads pru binaries
 *
 * @param[in]  ch    pru core to stop (0 or 1)
 *
 * @return     0 on success, -1 on failure.
 */
int rc_pru_stop(int ch);


#ifdef __cplusplus
}
#endif

#endif // RC_PRU_H

/** @} end group PRU */