/**
 * <rc/adc.h>
 *
 * @brief C interface for the Linux IIO ADC driver
 *
 * The Robotics cape and BeagleBone Blue include two voltage dividers for safe
 * measurement of the 2-cell lithium battery voltage and the voltage of any
 * power source connected to the 9-18V DC power jack. These can be read with
 * c_adc_batt() and rc_adc_dc_jack()
 *
 * There is also a 6-pin JST-SH socket on the Cape for connecting up to 4
 * potentiometers or general use analog signals. The pinout of this socket is as
 * follows:
 *
 * - 1 - Ground
 * - 2 - VDD_ADC (1.8V)
 * - 3 - AIN0
 * - 4 - AIN1
 * - 5 - AIN2
 * - 6 - AIN3
 *
 * All 8 ADC channels on the Sitara including the 4 listed above can be read
 * with rc_adc_read_raw(int ch) which returns the raw integer output of the
 * 12-bit ADC. rc_adc_read_volt(int ch) additionally converts this raw value to
 * a voltage.
 *
 * See the rc_test_adc example for sample use case.
 *
 * @addtogroup ADC
 * @ingroup    IO
 * @{
 */

#ifndef RC_ADC_H
#define RC_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief      initializes the analog to digital converter for reading
 *
 * @return     0 on success, -1 on failure.
 */
int rc_adc_init(void);

/**
 * @brief      Cleans up the ADC subsystem.
 *
 *             Call before your program closes down
 *
 * @return     0 on success, -1 on failure.
 */
int rc_adc_cleanup(void);

/**
 * @brief      reads the raw integer ADC value for a particular channel
 *
 * @param[in]  ch    channel 0-7
 *
 * @return     16-bit adc reading on success, -1 on failure.
 */
int rc_adc_read_raw(int ch);

/**
 * @brief      reads an adc voltage.
 *
 * @param[in]  ch    channel 0-7
 *
 * @return     voltage from 0-1.8v or -1 on error
 */
double rc_adc_read_volt(int ch);

/**
 * @brief      reads the voltage of the 2-cell Lithium battery
 *
 * @return     voltage of battery or -1 on failure
 */
double rc_adc_batt(void);

/**
 * @brief      Reads the voltage of the 9-18v DC jack
 *
 * @return     Voltage at DC jack or -1 on failure
 */
double rc_adc_dc_jack(void);



#ifdef __cplusplus
}
#endif

#endif // RC_ADC_H

/** @}  end group ADC*/