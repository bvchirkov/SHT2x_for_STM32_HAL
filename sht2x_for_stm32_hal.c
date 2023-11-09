/* An STM32 HAL library written for the SHT2x temperature/humidity sensor series. */
/* Libraries by @eepj www.github.com/eepj */
#include "sht2x_for_stm32_hal.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C"{
#endif

#define BUF_TO_VALUE(BUF) (uint16_t)(((BUF[0] << 8) & 0xFF00) | (BUF[1] & 0x00FF))

static uint32_t SHT2x_Ipow(uint32_t, uint8_t);

I2C_HandleTypeDef*  _sht2x_ui2c  = NULL;
uint8_t             raw_val[3]   = {0};

__IO _Bool is_reseived_data = false;
	
/**
 * @brief Initializes the SHT2x temperature/humidity sensor.
 * @param hi2c User I2C handle pointer.
 */
void SHT2x_Init(I2C_HandleTypeDef *hi2c) {
	_sht2x_ui2c = hi2c;
}

/**
 *  @brief Performs a soft reset.
 */
void SHT2x_SoftReset(void){
	uint8_t cmd = SHT2x_SOFT_RESET;
	HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &cmd, 1, SHT2x_TIMEOUT);
}

/**
 * @brief Gets the value stored in user register.
 * @return 8-bit value stored in user register, 0 to 255.
 */
uint8_t SHT2x_ReadUserReg(void) {
	uint8_t val = 0;
	uint8_t cmd = SHT2x_READ_REG;
	HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &cmd, 1, SHT2x_TIMEOUT);
	HAL_I2C_Master_Receive(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &val, 1, SHT2x_TIMEOUT);
	return val;
}

/**
 * @brief Sends the designated command to sensor and read a 16-bit raw value.
 * @param cmd Command to send to sensor.
 * @return 16-bit raw value, 0 to 65535.
 */
uint16_t SHT2x_GetRaw(uint8_t cmd) {
	uint8_t val[3] = {0};
	HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &cmd, 1, SHT2x_TIMEOUT);
	HAL_I2C_Master_Receive(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, val, 3, SHT2x_TIMEOUT);
	return BUF_TO_VALUE(val);
}

/**
 * @brief Measures and gets the current temperature.
 * @param hold Holding mode, 0 for no hold master, 1 for hold master.
 * @return Floating point temperature value.
 */
float SHT2x_GetTemperature(SHT2x_MasterMode mode) {
	uint8_t cmd = (mode ? SHT2x_READ_TEMP_HOLD : SHT2x_READ_TEMP_NOHOLD);
	return (float)(-46.85 + 175.72 * (SHT2x_GetRaw(cmd) / SHT2x_DIVIDER));
}

/**
 * @brief Measures and gets the current relative humidity.
 * @param hold Holding mode, 0 for no hold master, 1 for hold master.
 * @return Floating point relative humidity value.
 */
float SHT2x_GetRelativeHumidity(SHT2x_MasterMode mode) {
	uint8_t cmd = (mode ? SHT2x_READ_RH_HOLD : SHT2x_READ_RH_NOHOLD);
	return (float)(-6 + 125.00 * (SHT2x_GetRaw(cmd) / SHT2x_DIVIDER));
}

/**
 * @brief Sets the measurement resolution.
 * @param res Enum resolution.
 * @note Available resolutions: RES_14_12, RES_12_8, RES_13_10, RES_11_11.
 * @note RES_14_12 = 14-bit temperature and 12-bit RH resolution, etc.
 */
void SHT2x_SetResolution(SHT2x_Resolution res) {
	uint8_t val = SHT2x_ReadUserReg();
	val = (val & 0x7e) | res;
	uint8_t temp[2] = { SHT2x_WRITE_REG, val };
	HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, temp, sizeof(temp)/sizeof(temp[0]), SHT2x_TIMEOUT);
}

/**
 * @brief Converts degrees Celsius to degrees Fahrenheit.
 * @param celsius Floating point temperature in degrees Celsius.
 * @return Floating point temperature in degrees Fahrenheit.
 */
float SHT2x_CelsiusToFahrenheit(float celsius) {
	return (float)((9.0 / 5.0) * celsius + 32);
}

/**
 * @brief Converts degrees Celsius to Kelvin.
 * @param celsius Floating point temperature in degrees Celsius.
 * @return Floating point temperature in Kelvin.
 */
float SHT2x_CelsiusToKelvin(float celsius) {
	return (float)(celsius + 273.0);
}

/**
 * @brief Gets the integer part of a floating point number.
 * @note Avoids the use of sprinf floating point formatting.
 * @param num Floating point number.
 * @return Integer part of floating point number.
 */
int32_t SHT2x_GetInteger(float num) {
	return (int32_t)(num / 1L);
}

/**
 * @brief Gets the decimal part of a floating point number.
 * @note Avoids the use of sprinf floating point formatting.
 * @param num Floating point number.
 * @return Decimal part of floating point number.
 */
uint32_t SHT2x_GetDecimal(float num, uint8_t digits) {
	float postDec = num - (float)SHT2x_GetInteger(num);
	return (uint32_t)postDec * SHT2x_Ipow(10, digits);
}

/**
 * @brief Integer equivalent of pow() in math.h.
 * @param base Base.
 * @param power Power.
 * @return
 */
static uint32_t SHT2x_Ipow(uint32_t base, uint8_t power) {
	uint32_t temp = base;
	for (uint8_t i = 1; i < power; i++)
		temp *= base;
	return temp;
}

static HAL_StatusTypeDef SHT2x_SendRequestData(SHT2x_ParamType param_type, SHT2x_MasterMode mode) {
  uint8_t cmd = 0;
  if (param_type == SHT2x_TEMPERATURE) {
    cmd = (mode ? SHT2x_READ_TEMP_HOLD : SHT2x_READ_TEMP_NOHOLD);
  } else if (param_type == SHT2x_HUMIDITY) {
    cmd = (mode ? SHT2x_READ_RH_HOLD : SHT2x_READ_RH_NOHOLD);
  }

  HAL_StatusTypeDef transmit_status = HAL_BUSY;
  /* Проверка доступности сенсора здесь, чтобы не попасть в бесконечный
   * цикл I2C_WaitOnFlagUntilTimeout, внутри функции HAL_I2C_Master_Transmit*/
  uint8_t           i2c_line_status = __HAL_I2C_GET_FLAG(_sht2x_ui2c, I2C_FLAG_BUSY);
  if (i2c_line_status != SET) {
    transmit_status = HAL_I2C_Master_Transmit(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, &cmd, sizeof(cmd), SHT2x_TIMEOUT);
  }
  return transmit_status;
}

static HAL_StatusTypeDef SHT2x_NonBlock_StartReceive(void) {
  return HAL_I2C_Master_Receive_IT(_sht2x_ui2c, SHT2x_I2C_ADDR << 1, raw_val, sizeof(raw_val)/sizeof(raw_val[0]));
}

static uint8_t calculate_crc8(uint8_t* data, size_t len)
{
  uint8_t crc = 0x00; // init value (see "CRC Checksum Calculation for SHT2x")

  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc & 0x80) != 0) {
        crc = (uint8_t)((crc << 1) ^ 0x31);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

SHT2x_RequestStatus SHT2x_NonBlock_RequestRaw(SHT2x_ParamType param, SHT2x_MasterMode mode) {
  static _Bool is_sended_request = false;
  static _Bool is_waiting_data   = false;

  SHT2x_RequestStatus result = SHT2x_REQ_DATA_WAIT;

  if (is_sended_request == false) {
    if (SHT2x_SendRequestData(param, mode) == HAL_OK) {
      is_sended_request = true;
      is_waiting_data   = true;
    }
  }

  if (is_waiting_data) {
    if (SHT2x_NonBlock_StartReceive() == HAL_OK) {
      is_waiting_data  = false;
      is_reseived_data = false;
    }
  }

  if (is_reseived_data) { // Меняется в прерывании HAL_I2C_MasterRxCpltCallback
    is_sended_request = false;

    uint8_t crc = calculate_crc8(raw_val, 2);
    if (crc == raw_val[2]) {
      result = SHT2x_REQ_DATA_RECEIVED;
    }
  }

  return result;
}

float SHT2x_NonBlock_ReadRelativeHumidity(void) {
  uint16_t val = BUF_TO_VALUE(raw_val);
  return (float)(-6.0 + 125.0 * (val / SHT2x_DIVIDER));
}

float SHT2x_NonBlock_ReadTemperature(void) {
  uint16_t val = BUF_TO_VALUE(raw_val);
  return (float)(-46.85 + 175.72 * (val / SHT2x_DIVIDER));
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c) {
  if (hi2c == _sht2x_ui2c) {
    is_reseived_data = true;
  }
}

#ifdef __cplusplus
}
#endif
