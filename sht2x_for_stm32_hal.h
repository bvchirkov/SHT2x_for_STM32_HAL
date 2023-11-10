/* An STM32 HAL library written for the SHT2x temperature/humidity sensor series. */
/* Library by @eepj www.github.com/eepj */
#ifndef SHT2X_FOR_STM32_HAL_H
#define SHT2X_FOR_STM32_HAL_H
#include "stm32f1xx_hal.h"
/*----------------------------------------------------------------------------*/
#define SHT2x_I2C_ADDR			    0x40
#define SHT2x_READ_TEMP_HOLD	  0xe3
#define	SHT2x_READ_RH_HOLD		  0xe5
#define SHT2x_READ_TEMP_NOHOLD	0xf3
#define SHT2x_READ_RH_NOHOLD	  0xf5
#define	SHT2x_WRITE_REG			    0xe6
#define SHT2x_READ_REG			    0xe7
#define SHT2x_SOFT_RESET		    0xfe
#define SHT2x_TIMEOUT			      500

#define SHT2x_DIVIDER           (float)(1 << 16)
#define SHT2x_I2C_MODE          ((SHT2x_MasterMode)SHT2x_HOLD_MASTER)
/*----------------------------------------------------------------------------*/
typedef enum SHT2x_Resolution {
	RES_14_12 = 0x00,
	RES_12_8  = 0x01,
	RES_13_10 = 0x80,
	RES_11_11 = 0x81,
} SHT2x_Resolution;

typedef enum SHT2x_Param {
  SHT2x_TEMPERATURE,
  SHT2x_HUMIDITY
} SHT2x_MeasurementType;

typedef enum SHT2x_MasterMode {
  SHT2x_NO_HOLD_MASTER,
  SHT2x_HOLD_MASTER
} SHT2x_MasterMode;

typedef enum SHT2x_NBRequest_Status
{
  SHT2x_REQ_DATA_RECEIVED,
  SHT2x_REQ_DATA_WAIT
} SHT2x_RequestStatus;
/*----------------------------------------------------------------------------*/
void SHT2x_Init(I2C_HandleTypeDef *hi2c);
void SHT2x_SoftReset(void);
void SHT2x_SetResolution(SHT2x_Resolution res);

uint8_t SHT2x_ReadUserReg(void);

uint16_t SHT2x_GetRaw(uint8_t cmd);

float SHT2x_GetTemperature(SHT2x_MasterMode mode);
float SHT2x_GetRelativeHumidity(SHT2x_MasterMode mode);

float SHT2x_CelsiusToFahrenheit(float celsius);
float SHT2x_CelsiusToKelvin(float celsius);

int32_t  SHT2x_GetInteger(float f);
uint32_t SHT2x_GetDecimal(float f, uint8_t digits);

SHT2x_RequestStatus SHT2x_NonBlock_RequestMeasurement(SHT2x_MeasurementType data_type);
float               SHT2x_NonBlock_ReadRelativeHumidity(void);
float               SHT2x_NonBlock_ReadTemperature(void);

#endif
