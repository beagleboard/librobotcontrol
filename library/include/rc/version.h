/**
 * @headerfile version.h <rc/version.h>
 *
 * @brief functions for getting the current version of the Robotics Cape Library
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


#define RC_LIB_VERSION_FLOAT	0.41
#define RC_LIB_VERSION_STRING	"0.4.1"


/**
 * @brief      gets a floating point representation of the current Robotics Cape
 *             library version.
 *
 *             This is better than using the RC_LIB_VERSION_FLOAT macro because
 *             the number returned will be from the library .so file instead of
 *             being hard-coded into your program. Therefore it will change the
 *             return value when you update the library without the user needing
 *             to recompile their program.
 *
 * @return     float
 */
float rc_version_float();

/**
 * @brief      gets a string representation of the current Robotics Cape library
 *             version.
 *
 *             This is better than using the RC_LIB_VERSION_STRING macro because
 *             the number returned will be from the library .so file instead of
 *             being hard-coded into your program. Therefore it will change the
 *             return value when you update the library without the user needing
 *             to recompile their program.
 *
 * @return     const char* string
 */
const char* rc_version_string();

/**
 * @brief      prints a string representation of the current Robotics Cape library
 *             version with no trailing newline character.
 *
 *             This is better than using the RC_LIB_VERSION_STRING macro because
 *             the number returned will be from the library .so file instead of
 *             being hard-coded into your program. Therefore it will change the
 *             return value when you update the library without the user needing
 *             to recompile their program.
 *
 * @return     const char* string
 */
void rc_version_print();


#ifdef  __cplusplus
}
#endif

#endif // RC_VERSION_H

/** @}  end group version*/