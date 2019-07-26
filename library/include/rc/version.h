/**
 * @headerfile version.h <rc/version.h>
 *
 * @brief macros and functions for getting the current version of librobotcontrol
 *
 * @author     James Strawson
 * @date       2/8/2018
 *
 * @addtogroup version
 * @{
 */

#ifndef RC_VERSION_H
#define RC_VERSION_H

#ifdef  __cplusplus
extern "C" {
#endif


#define RC_LIB_VERSION_MAJOR	1
#define RC_LIB_VERSION_MINOR	0
#define RC_LIB_VERSION_PATCH	5
#define RC_LIB_VERSION_HEX	((RC_LIB_VERSION_MAJOR << 16) | \
				 (RC_LIB_VERSION_MINOR <<  8) | \
				 (RC_LIB_VERSION_PATCH))


/**
 * @brief      get an integer representation of the library version
 *
 * 8 bits are used for each component, with the patch number stored in the 8
 * least significant bits. E.g. for version 1.2.3 this would be 0x010203.
 *
 * @return     integer representation of the library version
 */
unsigned int rc_version(void);


/**
 * @brief      gets a string representation of the current library version.
 *
 * @return     const char* string
 */
const char* rc_version_string(void);


/**
 * @brief      prints a string representation of the current library version to
 * stdout with no trailing newline character.
 */
void rc_version_print(void);


#ifdef __cplusplus
}
#endif

#endif //RC_VERSION_H

/** @} end group version*/
