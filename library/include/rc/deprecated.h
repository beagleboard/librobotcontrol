/**
 * <rc/deprecated.h>
 *
 * @brief Deprecated functions that only exist for backwards compatability.
 *
 * @author     James Strawson
 * @date       4/26/2018
 *
 * @addtogroup deprecated
 * @{
 */

#ifndef RC_DEPRECATED_H
#define RC_DEPRECATED_H

#ifdef  __cplusplus
extern "C" {
#endif



int rc_initialize() __attribute__ ((deprecated));

int rc_cleanup() __attribute__ ((deprecated));




#ifdef __cplusplus
}
#endif

#endif // RC_DEPRECATED_H
/** @}  end group deprecated*/