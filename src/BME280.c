#include "BME280.h"
#include  "main.h"



//read F7 to FE, pressure, humidity and temp
void BME280_full_read(int2_t *adc_T, int32_t *adc_P, int32_t adc_H){
	uint8_t data[8];
	BME280_read_reg(*hi2c2, )
}
int32_t BME280_read_compensation_values(uint8_t *data){

}
int32_t BME280_set_compensation_values(uint8_t *data){

}

void BME280_Init(){
    
	uint8_t humConfig = BME280_CTRL_HUM_CONFIG;
	uint8_t ctrlConfig = BME280_CTRL_MEAS_CONFIG;
	HAL_I2C_Mem_Write(&hi2c2,(BME280_I2C_ADDRESS<<1),BME280_CTRL_HUM,1,&humConfig,1,1000);
	HAL_I2C_Mem_Write(&hi2c2,(BME280_I2C_ADDRESS<<1),BME280_CTRL_MEAS,1,&ctrlConfig,1,1000);
}
int32_t BME280_compensate_T_int32(int32_t adc_T, uint16_t dig_T1, int16_t dig_T2, int16_t dig_T3);
uint32_t BME280_compensate_P_int32(int32_t adc_P, uint16_t dig_P1, int16_t dig_P2, int16_t dig_P3,
 int16_t dig_P4, int16_t dig_P5, int16_t dig_P6, int16_t dig_P7, int16_t dig_P8, int16_t dig_P9);
uint32_t BME280_compensate_H_int32(int32_t adc_H, uint8_t dig_H1, int16_t dig_H2, uint8_t dig_H3,
 int16_t dig_H4, int16_t dig_H5, int8_t dig_H6);