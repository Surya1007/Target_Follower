
/*
 * MPU925.h
 *	Original code taken from github profile: desertkun
 *	Modified due to improper device initialization from original author
 *	MPU9250 device driver for STM32:NUCLEO-F412ZG
 *	Can be use for other processors as well by modifying line 17, and including corresponding spi library
 *  Created on: 23th Oct, 2021
 *  Author: Surya Teja
 */

#ifndef MPU925_H_
#define MPU925_H_

#include "main.h"
#include "MPU9250_Config.h"
#include "stm32f4xx_hal_spi.h"
#include "math.h"
#include "local_to_global.h"

#define  READWRITE_CMD  0x80
#define  MULTIPLEBYTE_CMD  0x40
#define  DUMMY_BYTE  0x00

#define  _address  0b11010000
// 400 kHz
const uint32_t _i2cRate = 400000;


// Values for Accel and Gyro
#define  PWR_CYCLE          0x20
#define  PWR_RESET          0x80
#define  CLOCK_SEL_PLL      0x01
#define  SEN_ENABLE          0x00
#define  I2C_MST_EN          0x30
#define  I2C_MST_CLK          0x0D
#define  I2C_SLV0_EN          0x80
#define  I2C_READ_FLAG      0x80

// Register map for Gyro and Accel

    // Self test registers for gyro
#define SELF_TEST_X_GYRO       0x00
#define SELF_TEST_Y_GYRO       0x01
#define SELF_TEST_Z_GYRO       0x02

    // Self test registers for accel
#define SELF_TEST_X_ACCEL      0x0D
#define SELF_TEST_Y_ACCEL      0x0E
#define SELF_TEST_Z_ACCEL      0x0F

    // Offset registers for gyro
#define XG_OFFSET_H            0x13
#define XG_OFFSET_L            0x14
#define YG_OFFSET_H            0x15
#define YG_OFFSET_L            0x16
#define ZG_OFFSET_H            0x17
#define ZG_OFFSET_L            0x18

    // Sample rate divider
#define SMPLRT_DIV             0x19

    // Configuration parameters
#define CONFIG                 0x1A
#define GYRO_CONFIG            0x1B
#define ACCEL_CONFIG           0x1C
#define ACCEL_CONFIG_2         0x1D

    // Some others
#define LP_ACCEL_ODR           0x1E
#define WOM_THR                0x1F
#define FIFO_EN                0x23

    // I2C details
#define I2C_MST_CTRL           0x24
#define I2C_SLV0_ADDR          0x25
#define I2C_SLV0_REG           0x26
#define I2C_SLV0_CTRL          0x27

#define I2C_SLV1_ADDR          0x28
#define I2C_SLV1_REG           0x29
#define I2C_SLV1_CTRL          0x2A

#define I2C_SLV2_ADDR          0x2B
#define I2C_SLV2_REG           0x2C
#define I2C_SLV2_CTRL          0x2D

#define I2C_SLV3_ADDR          0x2E
#define I2C_SLV3_REG           0x2F
#define I2C_SLV3_CTRL          0x30

#define I2C_SLV4_ADDR          0x31
#define I2C_SLV4_REG           0x32
#define I2C_SLV4_DO            0x33
#define I2C_SLV4_CTRL          0x34
#define I2C_SLV4_DI            0x35

#define I2C_MST_STATUS         0x36

    // Interrupt Pin registers
#define INT_PIN_CFG            0x37
#define INT_ENABLE             0x38
#define INT_STATUS             0x3A

    // Accel output registers
#define ACCEL_XOUT_H           0x3B
#define ACCEL_XOUT_L           0x3C
#define ACCEL_YOUT_H           0x3D
#define ACCEL_YOUT_L           0x3E
#define ACCEL_ZOUT_H           0x3F
#define ACCEL_ZOUT_L           0x40

    // Temperature output registers
#define TEMP_OUT_H             0x41
#define TEMP_OUT_L             0x42

    // Gyro output registers
#define GYRO_XOUT_H            0x43
#define GYRO_XOUT_L            0x44
#define GYRO_YOUT_H            0x45
#define GYRO_YOUT_L            0x46
#define GYRO_ZOUT_H            0x47
#define GYRO_ZOUT_L            0x48

    // External sensor output registers
#define EXT_SENS_DATA_00       0x49
#define EXT_SENS_DATA_01       0x4A
#define EXT_SENS_DATA_02       0x4B
#define EXT_SENS_DATA_03       0x4C
#define EXT_SENS_DATA_04       0x4D
#define EXT_SENS_DATA_05       0x4E
#define EXT_SENS_DATA_06       0x4F
#define EXT_SENS_DATA_07       0x50
#define EXT_SENS_DATA_08       0x51
#define EXT_SENS_DATA_09       0x52
#define EXT_SENS_DATA_10       0x53
#define EXT_SENS_DATA_11       0x54
#define EXT_SENS_DATA_12       0x55
#define EXT_SENS_DATA_13       0x56
#define EXT_SENS_DATA_14       0x57
#define EXT_SENS_DATA_15       0x58
#define EXT_SENS_DATA_16       0x59
#define EXT_SENS_DATA_17       0x5A
#define EXT_SENS_DATA_18       0x5B
#define EXT_SENS_DATA_19       0x5C
#define EXT_SENS_DATA_20       0x5D
#define EXT_SENS_DATA_21       0x5E
#define EXT_SENS_DATA_22       0x5F
#define EXT_SENS_DATA_23       0x60

    // Other I2C details
#define I2C_SLV0_DO            0x63
#define I2C_SLV1_DO            0x64
#define I2C_SLV2_DO            0x65
#define I2C_SLV3_DO            0x66
#define I2C_MST_DELAY_CTRL     0x67

    // Some other detials
#define SIGNAL_PATH_RESET      0x68
#define MOT_DETECT_CTRL        0x69
#define USER_CTRL              0x6A

    // Power Management
#define PWR_MGMT_1             0x6B
#define PWR_MGMT_2             0x6C

    // FIFO queues
#define FIFO_COUNTH            0x72
#define FIFO_COUNTL            0x73
#define FIFO_R_W               0x74

#define WHO_AM_I               0x75

    // Offset values for accel
#define XA_OFFSET_H            0x77
#define XA_OFFSET_L            0x78
#define YA_OFFSET_H            0x7A
#define YA_OFFSET_L            0x7B
#define ZA_OFFSET_H            0x7D
#define ZA_OFFSET_L            0x7E


// End register map for Gyro and Accel



// AK8963 registers and values
#define  AK8963_I2C_ADDR        0x0C
#define  AK8963_PWR_DOWN        0x10
#define  AK8963_CNT_MEAS1       0x12
#define  AK8963_CNT_MEAS2       0x16
#define  AK8963_RESET           0x01
#define  AK8963_FUSE_ROM        0x0F



// Register map for AK8963 (Magnetometer)

#define AK_WHO_AM_I            0x00
#define AK_INFO                0x01
#define AK_STATUS_1            0x02
#define AK_MAG_XL              0x03
#define AK_MAG_XH              0x04
#define AK_MAG_YL              0x05
#define AK_MAG_YH              0x06
#define AK_MAG_ZL              0x07
#define AK_MAG_ZH              0x08
#define AK_STATUS_2            0x09
#define AK_CONTROL             0x0A
#define AK_CONTROL_2		   0x0B
#define AK_SELF_TEST           0x0C
#define AK_TEST_1              0x0D
#define AK_TEST_2              0x0E
#define AK_I2C_DISABLE         0x0F
#define AK_SENSX               0x10
#define AK_SENSY               0x11
#define AK_SENSZ               0x12

// End register map for AK8963


static uint8_t _buffer[21];
static uint8_t _mag_adjust[3];
float self_test_result[6]  = {0.f};

float mag_factory_bias[3];

float mag_bias[3]; //= {284.50, 3.50, -60.73};
float mag_scale[3];//= {0.60, 1.46, 1.56};



// Acceleration range selector
#define ACCEL_FS_SEL_2G   	0x00
#define ACCEL_FS_SEL_4G   	0x08
#define ACCEL_FS_SEL_8G   	0x10
#define ACCEL_FS_SEL_16G  	0x18

// Gyro range selector
#define GYRO_FS_SEL_250DPS  0x00
#define GYRO_FS_SEL_500DPS  0x08
#define GYRO_FS_SEL_1000DPS 0x10
#define GYRO_FS_SEL_2000DPS 0x18

// DLPF selector
#define DLPF_184   			0x01
#define DLPF_92   			0x02
#define DLPF_41   			0x03
#define DLPF_20   			0x04
#define DLPF_10   			0x05
#define DLPF_5   			0x06

uint8_t who_mpu, who_ak;

typedef enum GyroRange_ {
	GYRO_RANGE_250DPS = 0,
	GYRO_RANGE_500DPS,
	GYRO_RANGE_1000DPS,
	GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_ {
	ACCEL_RANGE_2G = 0,
	ACCEL_RANGE_4G,
	ACCEL_RANGE_8G,
	ACCEL_RANGE_16G
} AccelRange;

typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ
} SampleRateDivider;

__weak void MPU9250_OnActivate()
{
}

static inline void MPU9250_Activate()
{
    MPU9250_OnActivate();
    HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
    HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
    uint8_t receivedbyte = 0;
    if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
    {
        return -1;
    }
    return receivedbyte;
}

void MPU_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    MPU9250_Activate();
    SPIx_WriteRead(WriteAddr);
    while(NumByteToWrite>=0x01)
    {
        SPIx_WriteRead(*pBuffer);
        NumByteToWrite--;
        pBuffer++;
    }
    MPU9250_Deactivate();
}

void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    MPU9250_Activate();
    uint8_t data = ReadAddr | READWRITE_CMD;
    HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&MPU9250_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
    MPU9250_Deactivate();
}

/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data)
{
    MPU_SPI_Write(&data, subAddress, 1);
    HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    MPU_SPI_Read(dest, subAddress, count);
}

/* writes a register to the AK8963 given a register address and data */
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    // set slave 0 to the AK8963 and set for write
    writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR);

    // set the register to the desired AK8963 sub address
    writeRegister(I2C_SLV0_REG,subAddress);

    // store the data for write
    writeRegister(I2C_SLV0_DO,data);

    // enable I2C and send 1 byte
    writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1);
}

/* reads registers from the AK8963 */
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    // set slave 0 to the AK8963 and set for read
    writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);

    // set the register to the desired AK8963 sub address
    writeRegister(I2C_SLV0_REG,subAddress);

    // enable I2C and request the bytes
    writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count);

    // takes some time for these registers to fill
    HAL_Delay(10);

    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    readRegisters(EXT_SENS_DATA_00,count,dest);
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
static uint8_t whoAmI()
{
    // read the WHO AM I register
    readRegisters(WHO_AM_I,1,_buffer);

    // return the register value
    return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
static int whoAmIAK8963()
{
    // read the WHO AM I register
    readAK8963Registers(AK_WHO_AM_I,1,_buffer);
    // return the register value
    return _buffer[0];
}

/* sets the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range)
{
    writeRegister(ACCEL_CONFIG, range);
}

/* sets the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range)
{
    writeRegister(GYRO_CONFIG, range);
}

/* sets the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth)
{
    writeRegister(ACCEL_CONFIG_2,bandwidth);
    writeRegister(CONFIG,bandwidth);
}

/* starts communication with the MPU-9250 */
uint8_t MPU9250_Init(AccelRange accel_range, GyroRange gyro_range)
{
	printf("Connecting to MPU-9250\n");
	HAL_Delay(10);

    // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
    who_mpu = whoAmI();

    if((who_mpu != 0x71) &&( who_mpu != 0x73))
    {

    }

    // reset the MPU9250
	writeRegister(PWR_MGMT_1,0x80);
	HAL_Delay(100);

	writeRegister(PWR_MGMT_1,0x03);
    HAL_Delay(1);

    // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
    who_mpu = whoAmI();

    if((who_mpu != 0x71) &&( who_mpu != 0x73))
    {

    }
    // select clock source to gyro
    writeRegister(PWR_MGMT_1, CLOCK_SEL_PLL);

    // FIFO register
    writeRegister(FIFO_EN, 0x00);
    HAL_Delay(10);

    // Set power management register to internal clock
    writeRegister(PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    // enable accelerometer and gyro
    writeRegister(PWR_MGMT_2,SEN_ENABLE);
    HAL_Delay(10);

    // enable I2C master mode and setting to spi mode
    writeRegister(USER_CTRL, 0x20);
    HAL_Delay(10);

    // set the I2C bus speed to 400 kHz
    writeRegister(I2C_MST_CTRL, 0x09);
    HAL_Delay(10);



    writeRegister(I2C_MST_DELAY_CTRL, 0x81);
    HAL_Delay(10);

    writeRegister(I2C_SLV4_CTRL, 0x01);
    HAL_Delay(10);


    uint8_t spi_value[1];
    readAK8963Registers(I2C_MST_CTRL, 1, spi_value);
    HAL_Delay(10);

    printf("Value is %x", spi_value[0]);

    // Set accel range
    MPU9250_SetAccelRange(accel_range);
    HAL_Delay(10);

    // Set gyro range
    MPU9250_SetGyroRange(gyro_range);
    HAL_Delay(10);

    // setting bandwidth to 184Hz as default
    writeRegister(ACCEL_CONFIG_2,0x00);
    HAL_Delay(10);

    // setting gyro bandwidth to 184Hz
    writeRegister(CONFIG,DLPF_184);
    HAL_Delay(10);

    // setting the sample rate divider to 0 as default
    writeRegister(SMPLRT_DIV,0x00);
    HAL_Delay(10);

    // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
    who_mpu = whoAmI();

    if((who_mpu != 0x71) &&( who_mpu != 0x73))
    {
        return 1;
    }

    printf("\rAccel, and gyro connected\n");
    HAL_Delay(100);
    // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
    who_ak = whoAmIAK8963();

    if( whoAmIAK8963() != 0x48 )
    {
        return 1;
    }

    printf("\rMag connected\n");
    HAL_Delay(100);

    writeRegister(USER_CTRL, 0x34); // Check here if error in scaling factor
    HAL_Delay(10);

    writeRegister(I2C_MST_CTRL, 0x0D);
    HAL_Delay(10);

    // Bypass MPU9250 to get to AK8963
    writeAK8963Register(INT_PIN_CFG, 0x22);
    HAL_Delay(10);

    writeAK8963Register(INT_ENABLE, 0x01);
    HAL_Delay(100);

    return 0;
}


void AK8963_Init()
{
    // Set AK8963 Power down
    writeAK8963Register(AK_CONTROL, AK8963_PWR_DOWN);
    HAL_Delay(10);

    // Fuse ROM
    writeAK8963Register(AK_CONTROL, 0x0F);
    HAL_Delay(100);

    // read the AK8963 ASA registers and compute magnetometer scale factors
    readAK8963Registers(AK_SENSX, 3, _mag_adjust);
    HAL_Delay(10);

    mag_factory_bias[0] = ((float)(_mag_adjust[0] - 128)/256. + 1.);
    mag_factory_bias[1] = ((float)(_mag_adjust[1] - 128)/256. + 1.);
    mag_factory_bias[2] = ((float)(_mag_adjust[2] - 128)/256. + 1.);

    // Set AK8963 Power down
    writeAK8963Register(AK_CONTROL, AK8963_PWR_DOWN);
    HAL_Delay(10);

    // Set AK8963 mode
    writeAK8963Register(AK_CONTROL, 0x12);
    HAL_Delay(20);

    printf("\rMag Factory Calibration Values: \n");
    HAL_Delay(100);
    printf("\rX-Axis sensitivity offset value ");
    printf("%f, \n", mag_factory_bias[0]);
    HAL_Delay(100);
    printf("\rY-Axis sensitivity offset value ");
    printf("%f, \n", mag_factory_bias[1]);
    HAL_Delay(100);
    printf("\rZ-Axis sensitivity offset value ");
    printf("%f, \n", mag_factory_bias[2]);
    HAL_Delay(100);


}

void Get_AK8963_Data(float* mag_data)
{
	uint8_t status1, status2;
	readAK8963Registers(AK_STATUS_1, 1, &status1);
	HAL_Delay(10);
	if (status1 & 0x01)
	{
		uint8_t raw_data[14];
		readAK8963Registers(AK_MAG_XL, 7, &raw_data[0]);
        readAK8963Registers(AK_STATUS_2, 1, &status2);
		//printf("\r\nStatus2 value is %x, \n", status2);
        //printf("\r\n%x, %x, %x, %x, %x, %x, %x", raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4], raw_data[5], raw_data[6]);

		if (!(status2 & 0x08))
		{
            mag_data[0] =  ((float) (int16_t) (((int16_t)raw_data[1] << 8) |  (int16_t) raw_data[0]));
            mag_data[1] =  ((float) (int16_t) (((int16_t)raw_data[3] << 8) |  (int16_t) raw_data[2]));
            mag_data[2] =  ((float) (int16_t) (((int16_t)raw_data[5] << 8) |  (int16_t) raw_data[4]));
            //printf("\r\n[%f, %f, %f], ", mag_data[0], mag_data[1], mag_data[2]);
            //HAL_Delay(10);
            //printf("\r\n%x, %x, %x, %x, %x, %x, %x", raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4], raw_data[5], raw_data[6]);
            //HAL_Delay(100);
		}


	}
}
/* read the data, each argument should point to a array for x, y, and x */
void MPU9250_GetData(int16_t* AccData, int16_t* GyroData)
{
    // grab the data from the MPU9250
    readRegisters(ACCEL_XOUT_H, 14, _buffer);


    // combine into 16 bit values
    AccData[0] = (int16_t) ((((int16_t)_buffer[0]) << 8) |  _buffer[1]);
    AccData[1] = (int16_t) ((((int16_t)_buffer[2]) << 8) |  _buffer[3]);
    AccData[2] = (int16_t) ((((int16_t)_buffer[4]) << 8) |  _buffer[5]);

    GyroData[0] = (int16_t) ((((int16_t)_buffer[8]) << 8) |  _buffer[9]);
    GyroData[1] = (int16_t) ((((int16_t)_buffer[10]) << 8) |  _buffer[11]);
    GyroData[2] = (int16_t) ((((int16_t)_buffer[12]) << 8) |  _buffer[13]);


    //MagData[0] = (int16_t)((float)magx * 0.15 * ((float)(_mag_adjust[0] - 128) / 256.0f + 1.0f));
    //MagData[1] = (int16_t)((float)magy * 0.15 * ((float)(_mag_adjust[1] - 128) / 256.0f + 1.0f));
    //MagData[2] = (int16_t)((float)magz * 0.15 * ((float)(_mag_adjust[2] - 128) / 256.0f + 1.0f));

}

void Update_MPU9250_Data(float* AData, float* GData, float* MData)
{
	int16_t adata[3], gdata[3];
	float mdata[3];
	MPU9250_GetData(adata, gdata);
	AData[0] = (float) adata[0] * 2 / 36768;
	AData[1] = (float) adata[1] * 2 / 36768;
	AData[2] = (float) adata[2] * 2 / 36768;

	GData[0] = (float) gdata[0] * 250 / 36768;
	GData[1] = (float) gdata[1] * 250 / 36768;
	GData[2] = (float) gdata[2] * 250 / 36768;

    float mag_resolution = 10.0 * 4912.0 / 32760.0;
	Get_AK8963_Data(mdata);

	MData[0] = (float) (((mdata[0] * mag_resolution * mag_factory_bias[0]) - mag_bias[0]) * mag_scale[0]);
	MData[1] = (float) (((mdata[1] * mag_resolution * mag_factory_bias[1]) - mag_bias[1]) * mag_scale[1]);
	MData[2] = (float) (((mdata[2] * mag_resolution * mag_factory_bias[2]) - mag_bias[2]) * mag_scale[2]);

}


void MPU9250SelfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t selfTest[6] = {0, 0, 0, 0, 0, 0};
    int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
    float factoryTrim[6];
    uint8_t FS = 0;
    int16_t Acc_Self_Data[3], Gyro_Self_Data[3];

    writeRegister(SMPLRT_DIV,0x00); // Set gyro sample rate to 1 kHz
    writeRegister(CONFIG,DLPF_92); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz

    MPU9250_SetGyroRange(GYRO_RANGE_250DPS);  // Set full scale range for the gyro to 250 dps
    MPU9250_SetAccelRange(ACCEL_RANGE_2G); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    MPU9250_SetDLPFBandwidth(DLPF_BANDWIDTH_92HZ); // May neeed to check..........................................................

    for (uint16_t num_read = 0; num_read < 200; num_read++)
    {
        MPU9250_GetData((int16_t *)Acc_Self_Data, (int16_t *)Gyro_Self_Data);
        aAvg[0] += Acc_Self_Data[0];
        aAvg[1] += Acc_Self_Data[1];
        aAvg[2] += Acc_Self_Data[2];

        gAvg[0] += Gyro_Self_Data[0];
        gAvg[1] += Gyro_Self_Data[1];
        gAvg[2] += Gyro_Self_Data[2];
    }

    for (uint8_t axis = 0; axis < 3; axis++)
    {
        aAvg[axis] /= 200;
        gAvg[axis] /= 200;
    }

    // Configure accelerometer and gyroscope for self-test
    writeRegister(ACCEL_CONFIG,0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    writeRegister(GYRO_CONFIG,0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    HAL_Delay(50);

    for (uint16_t num_read = 0; num_read < 200; num_read++)
    {
        MPU9250_GetData((int16_t *)Acc_Self_Data, (int16_t *)Gyro_Self_Data);
        aSTAvg[0] += Acc_Self_Data[0];
        aSTAvg[1] += Acc_Self_Data[1];
        aSTAvg[2] += Acc_Self_Data[2];

        gSTAvg[0] += Gyro_Self_Data[0];
        gSTAvg[1] += Gyro_Self_Data[1];
        gSTAvg[2] += Gyro_Self_Data[2];
    }

    for (uint8_t axis = 0; axis < 3; axis++)
    {
        aSTAvg[axis] /= 200;
        gSTAvg[axis] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    writeRegister(ACCEL_CONFIG,0x00);
    writeRegister(GYRO_CONFIG,0x00);
    HAL_Delay(50);

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    readRegisters(SELF_TEST_X_ACCEL, 1, &selfTest[0]);
    readRegisters(SELF_TEST_Y_ACCEL, 1, &selfTest[1]);
    readRegisters(SELF_TEST_Z_ACCEL, 1, &selfTest[2]);

    readRegisters(SELF_TEST_X_GYRO, 1, &selfTest[3]);
    readRegisters(SELF_TEST_Y_GYRO, 1, &selfTest[4]);
    readRegisters(SELF_TEST_Z_GYRO, 1, &selfTest[5]);

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620/1<<FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620/1<<FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620/1<<FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620/1<<FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620/1<<FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620/1<<FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++)
   {
     self_test_result[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
     self_test_result[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
   }

   printf("\rx-axis self test: acceleration trim within : %f %% of factory value\n", self_test_result[0]);
   printf("\ry-axis self test: acceleration trim within : %f %% of factory value\n", self_test_result[1]);
   printf("\rz-axis self test: acceleration trim within : %f %% of factory value\n", self_test_result[2]);

   printf("\rx-axis self test: gyration trim within : %f %% of factory value\n", self_test_result[3]);
   printf("\ry-axis self test: gyration trim within : %f %% of factory value\n", self_test_result[4]);
   printf("\rz-axis self test: gyration trim within : %f %% of factory value\n", self_test_result[5]);

}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * abias, float * gbias)
{
    uint16_t ii, packet_count, _count;

    // reset device
    writeRegister(PWR_MGMT_1, PWR_RESET);
    HAL_Delay(100);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeRegister(PWR_MGMT_1,	0x01);
    writeRegister(PWR_MGMT_2,	0x00);
    HAL_Delay(200);

    // Configure device for bias calculation
    writeRegister(INT_ENABLE,	0x00);        // Disable all interrupts
    writeRegister(FIFO_EN,		0x00);           // Disable FIFO
    writeRegister(PWR_MGMT_1,	0x00);        // Turn on internal clock source
    writeRegister(I2C_MST_CTRL, 0x00);      // Disable I2C master
    writeRegister(USER_CTRL, 	0x00);         // Disable FIFO and I2C master modes
    writeRegister(USER_CTRL, 	0x0C);         // Reset FIFO and DMP
    HAL_Delay(50);


    // Configure MPU9250 gyro and accelerometer for bias calculation

	writeRegister(CONFIG, 		0x01);		// Set low-pass filter to 188 Hz
	writeRegister(SMPLRT_DIV, 	0x00); 		// Set sample rate to 1 kHz
	writeRegister(GYRO_CONFIG, 	0x00);		// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeRegister(ACCEL_CONFIG, 0x00);		// Set accelerometer full-scale to 2 g, maximum sensitivity



	// Sensitivity
    uint16_t  gyrosensitivity  = 262;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 32767;  // = 16384 LSB/g

	int16_t Acc_Dat[3], Gyro_Dat[3];
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    packet_count = 50;
    for (ii = 0; ii < packet_count; ii++)
    {
        MPU9250_GetData((int16_t *)Acc_Dat, (int16_t *)Gyro_Dat);
        for (_count = 0; _count < 3; _count++)
        {
        	accel_bias[_count] += Acc_Dat[_count];
        	gyro_bias[_count] += Gyro_Dat[_count];
        }
    }
    accel_bias[0] /= (float) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (float) packet_count;
    accel_bias[2] /= (float) packet_count;
    gyro_bias[0]  /= (float) packet_count;
    gyro_bias[1]  /= (float) packet_count;
    gyro_bias[2]  /= (float) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (float) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (float) accelsensitivity;}


  abias[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  abias[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  abias[2] = (float) gyro_bias[2] / (float) gyrosensitivity;


// Output scaled accelerometer biases for manual subtraction in the main program
   gbias[0] = (float)accel_bias[0] / (float)accelsensitivity;
   gbias[1] = (float)accel_bias[1] / (float)accelsensitivity;
   gbias[2] = (float)accel_bias[2] / (float)accelsensitivity;


   // Start reading factory values
   uint8_t read_data[2] = {0};
   int16_t acc_bias_reg[3] = {0, 0, 0};

   readRegisters(XA_OFFSET_H, 2, &read_data[0]);
   acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
   readRegisters(YA_OFFSET_H, 2, &read_data[0]);
   acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
   readRegisters(ZA_OFFSET_H, 2, &read_data[0]);
   acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

   uint8_t write_data[6] = {0};
   write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
   write_data[1] = (acc_bias_reg[0]) & 0xFF;
   write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
   write_data[3] = (acc_bias_reg[1]) & 0xFF;
   write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
   write_data[5] = (acc_bias_reg[2]) & 0xFF;

   writeRegister(XA_OFFSET_H,	write_data[0]);
   writeRegister(XA_OFFSET_L,	write_data[1]);
   writeRegister(YA_OFFSET_H,	write_data[2]);
   writeRegister(YA_OFFSET_L,	write_data[3]);
   writeRegister(ZA_OFFSET_H,	write_data[4]);
   writeRegister(ZA_OFFSET_L,	write_data[5]);

   uint8_t gyro_offset_data[6] = {0};
   gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
   gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
   gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
   gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
   gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
   gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;

   writeRegister(XG_OFFSET_H,	gyro_offset_data[0]);
   writeRegister(XG_OFFSET_L,	gyro_offset_data[1]);
   writeRegister(YG_OFFSET_H,	gyro_offset_data[2]);
   writeRegister(YG_OFFSET_L,	gyro_offset_data[3]);
   writeRegister(ZG_OFFSET_H,	gyro_offset_data[4]);
   writeRegister(ZG_OFFSET_L,	gyro_offset_data[5]);

}


void Callibrate_AK8963()
{

	uint16_t sample_count = 700;

    float bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
    float mag_max[3] = {-32767, -32767, -32767};
    float mag_min[3] = {32767, 32767, 32767};
    float mag_temp[3] = {0, 0, 0};
	AK8963_Init();

    for (uint16_t ii = 0; ii < sample_count; ii++)
    {
        Get_AK8963_Data(mag_temp);  // Read the mag data
        printf("[%f, %f, %f], ", mag_temp[0], mag_temp[1], mag_temp[2]);
        for (int jj = 0; jj < 3; jj++) {
            if (mag_temp[jj] > mag_max[jj])
            {
            	mag_max[jj] = mag_temp[jj];
            }
            if (mag_temp[jj] < mag_min[jj])
            {
            	mag_min[jj] = mag_temp[jj];
            }
        }
        HAL_Delay(12);
    }

    printf("mag x min/max: %f, %f", mag_min[0], mag_max[0]);
    HAL_Delay(10);
    printf("mag y min/max: %f, %f", mag_min[1], mag_max[1]);
    HAL_Delay(10);
    printf("mag z min/max: %f, %f", mag_min[2], mag_max[2]);
    HAL_Delay(10);

    bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
    bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
    bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

    float bias_resolution = 10.0 * 4912.0 / 32760.0;
    mag_bias[0] = (float)bias[0] * bias_resolution * mag_factory_bias[0];  // save mag biases in G for main program
    mag_bias[1] = (float)bias[1] * bias_resolution * mag_factory_bias[1];
    mag_bias[2] = (float)bias[2] * bias_resolution * mag_factory_bias[2];

    // Get soft iron correction estimate
    //*** multiplication by mag_bias_factory added in accordance with the following comment
    //*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
    scale[0] = (float)(mag_max[0] - mag_min[0]) * mag_factory_bias[0] / 2;  // get average x axis max chord length in counts
    scale[1] = (float)(mag_max[1] - mag_min[1]) * mag_factory_bias[1] / 2;  // get average y axis max chord length in counts
    scale[2] = (float)(mag_max[2] - mag_min[2]) * mag_factory_bias[2] / 2;  // get average z axis max chord length in counts

    float avg_rad = scale[0] + scale[1] + scale[2];
    avg_rad /= 3.0;

    mag_scale[0] = avg_rad / ((float)scale[0]);
    mag_scale[1] = avg_rad / ((float)scale[1]);
    mag_scale[2] = avg_rad / ((float)scale[2]);


    HAL_Delay(500);
    printf("\r\nMag scale values are %f, %f, and %f ", mag_scale[0], mag_scale[1], mag_scale[2]);
    HAL_Delay(200);
    printf("   Mag bias values are %f, %f, and %f ", mag_bias[0], mag_bias[1], mag_bias[2]);
    HAL_Delay(500);

}






/* sets the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd)
{
    /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
    writeRegister(SMPLRT_DIV,19);

    if(srd > 9)
    {
        // set AK8963 to Power Down
        writeAK8963Register(AK_CONTROL,AK8963_PWR_DOWN);

        // long wait between AK8963 mode changes
        HAL_Delay(100);

        // set AK8963 to 16 bit resolution, 8 Hz update rate
        writeAK8963Register(AK_CONTROL,AK8963_CNT_MEAS1);

        // long wait between AK8963 mode changes
        HAL_Delay(100);

        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK_MAG_XL,7,_buffer);

    }
    else
    {
        // set AK8963 to Power Down
        writeAK8963Register(AK_CONTROL,AK8963_PWR_DOWN);
        // long wait between AK8963 mode changes
        HAL_Delay(100);
        // set AK8963 to 16 bit resolution, 100 Hz update rate
        writeAK8963Register(AK_CONTROL,AK8963_CNT_MEAS2);

        // long wait between AK8963 mode changes
        HAL_Delay(100);

        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK_MAG_XL,7,_buffer);
    }

    writeRegister(SMPLRT_DIV, srd);
}


void convert_loc_to_global()
{
	// Converts local IMU data to global data
	float accel_data[3], gyro_data[3], mag_data[3];
	float gloabl_accel_data[3], global_gyro_data[3], yaw = 0;
	Update_MPU9250_Data(accel_data, gyro_data, mag_data);
	sensor_global_reference(gloabl_accel_data, accel_data, yaw);
	sensor_global_reference(global_gyro_data, gyro_data, yaw);
}





#endif /* MPU925_H_ */





