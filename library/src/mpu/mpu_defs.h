/*
See  MPU-9250 Register Map and Descriptions, Revision 4.0,
RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
above document; the MPU9250 and MPU9150 are virtually
identical but the latter has a different register map
*/

#ifndef RC_MPU_DEFS
#define RC_MPU_DEFS


// internal DMP sample rate limits
#define DMP_MAX_RATE		200
#define DMP_MIN_RATE		4
#define IMU_POLL_TIMEOUT	300 // milliseconds


/******************************************************************
* register offsets
******************************************************************/
#define SELF_TEST_X_GYRO	0x00
#define SELF_TEST_Y_GYRO	0x01
#define SELF_TEST_Z_GYRO	0x02
#define X_FINE_GAIN		0x03
#define Y_FINE_GAIN		0x04
#define Z_FINE_GAIN		0x05
#define SELF_TEST_X_ACCEL	0x0D
#define SELF_TEST_Y_ACCEL	0x0E
#define SELF_TEST_Z_ACCEL	0x0F
#define SELF_TEST_A		0x10
#define XG_OFFSET_H		0x13
#define XG_OFFSET_L		0x14
#define YG_OFFSET_H		0x15
#define YG_OFFSET_L		0x16
#define ZG_OFFSET_H		0x17
#define ZG_OFFSET_L		0x18
#define SMPLRT_DIV		0x19
#define CONFIG			0x1A
#define GYRO_CONFIG		0x1B
#define ACCEL_CONFIG		0x1C
#define ACCEL_CONFIG_2		0x1D
#define LP_ACCEL_ODR		0x1E
#define WOM_THR			0x1F
#define MOT_DUR			0x20
#define ZMOT_THR		0x21
#define ZRMOT_DUR		0x22
#define FIFO_EN			0x23
#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR		0x2E
#define I2C_SLV3_REG		0x2F
#define I2C_SLV3_CTRL		0x30
#define I2C_SLV4_ADDR		0x31
#define I2C_SLV4_REG		0x32
#define I2C_SLV4_DO		0x33
#define I2C_SLV4_CTRL		0x34
#define I2C_SLV4_DI		0x35
#define I2C_MST_STATUS		0x36
#define INT_PIN_CFG		0x37
#define INT_ENABLE		0x38
#define DMP_INT_STATUS		0x39
#define INT_STATUS		0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H		0x41
#define TEMP_OUT_L		0x42
#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define EXT_SENS_DATA_06	0x4F
#define EXT_SENS_DATA_07	0x50
#define EXT_SENS_DATA_08	0x51
#define EXT_SENS_DATA_09	0x52
#define EXT_SENS_DATA_10	0x53
#define EXT_SENS_DATA_11	0x54
#define EXT_SENS_DATA_12	0x55
#define EXT_SENS_DATA_13	0x56
#define EXT_SENS_DATA_14	0x57
#define EXT_SENS_DATA_15	0x58
#define EXT_SENS_DATA_16	0x59
#define EXT_SENS_DATA_17	0x5A
#define EXT_SENS_DATA_18	0x5B
#define EXT_SENS_DATA_19	0x5C
#define EXT_SENS_DATA_20	0x5D
#define EXT_SENS_DATA_21	0x5E
#define EXT_SENS_DATA_22	0x5F
#define EXT_SENS_DATA_23	0x60
#define MOT_DETECT_STATUS	0x61
#define I2C_SLV0_DO		0x63
#define I2C_SLV1_DO		0x64
#define I2C_SLV2_DO		0x65
#define I2C_SLV3_DO		0x66
#define I2C_MST_DELAY_CTRL	0x67
#define SIGNAL_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL		0x6A
#define PWR_MGMT_1		0x6B
#define PWR_MGMT_2		0x6C
#define DMP_BANK		0x6D
#define DMP_RW_PNT		0x6E
#define DMP_REG			0x6F
#define DMP_REG_1		0x70
#define DMP_REG_2		0x71
#define FIFO_COUNTH		0x72
#define FIFO_COUNTL		0x73
#define FIFO_R_W		0x74
#define WHO_AM_I_MPU9250	0x75 // Should return 0x71
#define XA_OFFSET_H		0x77
#define XA_OFFSET_L		0x78
#define YA_OFFSET_H		0x7A
#define YA_OFFSET_L		0x7B
#define ZA_OFFSET_H		0x7D
#define ZA_OFFSET_L		0x7E




/*******************************************************************
* CONFIG register bits
*******************************************************************/
#define FIFO_MODE_REPLACE_OLD	0
#define FIFO_MODE_KEEP_OLD	0x01<<6
#define EXT_SYNC_SET_DISABLE	0


/*******************************************************************
* GYRO_CONFIG register bits
*******************************************************************/
#define XGYRO_CTEN		0x01<<7
#define YGYRO_CTEN		0x01<<6
#define ZGYRO_CTEN		0x01<<5
#define GYRO_FSR_CFG_250	0x00<<3
#define GYRO_FSR_CFG_500	0x01<<3
#define GYRO_FSR_CFG_1000	0x02<<3
#define GYRO_FSR_CFG_2000	0x03<<3
#define FCHOICE_B_DLPF_EN	0x00
#define FCHOICE_B_DLPF_DISABLE	0x01

/*******************************************************************
* ACCEL_CONFIG register bits
*******************************************************************/
#define AX_ST_EN		0x01<<7
#define AY_ST_EN		0x01<<6
#define AZ_ST_EN		0x01<<5
#define ACCEL_FSR_CFG_2G	0x00<<3
#define ACCEL_FSR_CFG_4G	0x01<<3
#define ACCEL_FSR_CFG_8G	0x02<<3
#define ACCEL_FSR_CFG_16G	0x03<<3

/*******************************************************************
* ACCEL_CONFIG2 register bits
*******************************************************************/
#define ACCEL_FCHOICE_1KHZ	0x00<<3
#define ACCEL_FCHOICE_4KHZ	0x01<<3


/*******************************************************************
* INT_PIN_CFG
*******************************************************************/
#define ACTL_ACTIVE_LOW		0x01<<7
#define ACTL_ACTIVE_HIGH	0
#define INT_OPEN_DRAIN		0
#define INT_PUSH_PULL		0x00<<6
#define LATCH_INT_EN		0x01<<5
#define INT_ANYRD_CLEAR		0x01<<4
#define ACTL_FSYNC_ACTIVE_LOW	0x01<<3
#define ACTL_FSYNC_ACTIVE_HIGH	0x00<<3
#define FSYNC_INT_MODE_EN	0x01<<2
#define FSYNC_INT_MODE_DIS	0x00<<2
#define BYPASS_EN		0x01<<1


/*******************************************************************
*INT_ENABLE register settings
*******************************************************************/
#define WOM_EN			0x01<<6
#define WOM_DIS			0x00<<6
#define FIFO_OVERFLOW_EN	0x01<<4
#define FIFO_OVERFLOW_DIS	0x00<<4
#define FSYNC_INT_EN		0x01<<3
#define FSYNC_INT_DIS		0x00<<3
#define RAW_RDY_EN		0x01
#define RAW_RDY_DIS		0x00

/*******************************************************************
* FIFO_EN register settings
*******************************************************************/
#define FIFO_TEMP_EN		0x01<<7
#define FIFO_GYRO_X_EN		0x01<<6
#define FIFO_GYRO_Y_EN		0x01<<5
#define FIFO_GYRO_Z_EN		0x01<<4
#define FIFO_ACCEL_EN		0x01<<3
#define FIFO_SLV2_EN		0x01<<2
#define FIFO_SLV1_EN		0x01<<1
#define FIFO_SLV0_EN		0x01


/*******************************************************************
* PWR_MGMT_1 register settings
*******************************************************************/
#define H_RESET			0x01<<7
#define MPU_SLEEP		0x01<<6
#define MPU_CYCLE		0x01<<5


/*******************************************************************
* temperature reading constants
*******************************************************************/
#define ROOM_TEMP_OFFSET	0x00
#define TEMP_SENSITIVITY	333.87 // degC/LSB


/*******************************************************************
* USER_CTRL settings bits
*******************************************************************/
#define FIFO_EN_BIT		0x01<<6
#define I2C_MST_EN		0x01<<5
#define I2C_IF_DIS		0x01<<4
#define FIFO_RST		0x01<<2
#define I2C_MST_RST		0x01<<1
#define SIG_COND_RST		0x01

/******************************************************************
* Magnetometer Registers
******************************************************************/
#define AK8963_ADDR		0x0C
#define WHO_AM_I_AK8963		0x00 // should return 0x48
#define INFO			0x01
#define AK8963_ST1		0x02  // data ready status
#define AK8963_XOUT_L		0x03  // data
#define AK8963_XOUT_H		0x04
#define AK8963_YOUT_L		0x05
#define AK8963_YOUT_H		0x06
#define AK8963_ZOUT_L		0x07
#define AK8963_ZOUT_H		0x08
#define AK8963_ST2		0x09
#define AK8963_CNTL		0x0A  // main mode control register
#define AK8963_ASTC		0x0C  // Self test control
#define AK8963_I2CDIS		0x0F  // I2C disable
#define AK8963_ASAX		0x10  // x-axis sensitivity adjustment value
#define AK8963_ASAY		0x11  // y-axis sensitivity adjustment value
#define AK8963_ASAZ		0x12  // z-axis sensitivity adjustment value

/******************************************************************
* Magnetometer AK8963_CNTL register Settings
******************************************************************/
#define MAG_POWER_DN		0x00	// power down magnetometer
#define MAG_SINGLE_MES		0x01	// powers down after 1 measurement
#define MAG_CONT_MES_1		0x02	// 8hz continuous self-sampling
#define MAG_CONT_MES_2		0x06	// 100hz continuous self-sampling
#define MAG_EXT_TRIG		0x04	// external trigger mode
#define MAG_SELF_TEST		0x08	// self test mode
#define MAG_FUSE_ROM		0x0F	// ROM read only mode
#define MSCALE_16		0x01<<4
#define MSCALE_14		0x00

/******************************************************************
* Magnetometer AK8963_ST2 register definitions
******************************************************************/
#define MAGNETOMETER_SATURATION	0x01<<3

/******************************************************************
* Magnetometer AK8963_ST1 register definitions
******************************************************************/
#define MAG_DATA_READY		0x01

/******************************************************************
* Magnetometer sensitivity in micro Teslas to LSB
******************************************************************/
#define MAG_RAW_TO_uT		(4912.0/32760.0)


#define BIT_I2C_MST_VDDIO	(0x80)
#define BIT_FIFO_EN		(0x40)
#define BIT_DMP_EN		(0x80)
#define BIT_FIFO_RST		(0x04)
#define BIT_DMP_RST		(0x08)
#define BIT_FIFO_OVERFLOW	(0x10)
#define BIT_DATA_RDY_EN		(0x01)
#define BIT_DMP_INT_EN		(0x02)
#define BIT_MOT_INT_EN		(0x40)
#define BITS_FSR		(0x18)
#define BITS_LPF		(0x07)
#define BITS_HPF		(0x07)
#define BITS_CLK		(0x07)
#define BIT_FIFO_SIZE_1024	(0x40)
#define BIT_FIFO_SIZE_2048	(0x80)
#define BIT_FIFO_SIZE_4096	(0xC0)
#define BIT_RESET		(0x80)
#define BIT_SLEEP		(0x40)
#define BIT_S0_DELAY_EN		(0x01)
#define BIT_S2_DELAY_EN		(0x04)
#define BITS_SLAVE_LENGTH	(0x0F)
#define BIT_SLAVE_BYTE_SW	(0x40)
#define BIT_SLAVE_GROUP		(0x10)
#define BIT_SLAVE_EN		(0x80)
#define BIT_I2C_READ		(0x80)
#define BITS_I2C_MASTER_DLY	(0x1F)
#define BIT_AUX_IF_EN		(0x20)
#define BIT_ACTL		(0x80)
#define BIT_LATCH_EN		(0x20)
#define BIT_ANY_RD_CLR		(0x10)
#define BIT_BYPASS_EN		(0x02)
#define BITS_WOM_EN		(0xC0)
#define BIT_LPA_CYCLE		(0x20)
#define BIT_STBY_XA		(0x20)
#define BIT_STBY_YA		(0x10)
#define BIT_STBY_ZA		(0x08)
#define BIT_STBY_XG		(0x04)
#define BIT_STBY_YG		(0x02)
#define BIT_STBY_ZG		(0x01)
#define BIT_STBY_XYZA	(BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG	(BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

//#define GYRO_SF		(46850825LL * 200 / DMP_SAMPLE_RATE)
#define GYRO_SF			(46850825LL)

#define DMP_FEATURE_TAP			(0x001)
#define DMP_FEATURE_ANDROID_ORIENT	(0x002)
#define DMP_FEATURE_LP_QUAT		(0x004)
#define DMP_FEATURE_PEDOMETER		(0x008)
#define DMP_FEATURE_6X_LP_QUAT		(0x010)
#define DMP_FEATURE_GYRO_CAL		(0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL	(0x040)
#define DMP_FEATURE_SEND_RAW_GYRO	(0x080)
#define DMP_FEATURE_SEND_CAL_GYRO	(0x100)
#define DMP_FEATURE_SEND_ANY_GYRO	(DMP_FEATURE_SEND_RAW_GYRO | \
					DMP_FEATURE_SEND_CAL_GYRO)

#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)

#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define INV_WXYZ_QUAT       (0x100)



#endif
