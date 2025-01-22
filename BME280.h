



#ifndef __BME280_H
#define __BME280_H



#define BME280_CTRL_MEAS_CONFIG ((uint8_t)0b10110111)
#define BME280_CTRL_HUM_CONFIG ((uint8_t)0b00000101)
#define BME280_CTRL_MEAS 0xF4
#define BME280_CTRL_HUM 0xF2
#define BME280_PRESSURE 0xF7
#define BME280_TEMPERATURE 0xFA
#define BME280_HUMIDITY 0xFD
#define BME280_ID ((uint8_t)0xD0)
#define BME280_DIG_T1_REG = 0x88;    
#define BME280_DIG_T2_REG = 0x8A;
#define BME280_DIG_T3_REG = 0x8C;
#define BME280_DIG_P1_REG = 0x8F;   //uint16:t
#define BME280_DIG_P2_REG = 0x91;
#define BME280_DIG_P3_REG = 0x93;
#define BME280_DIG_P4_REG = 0x95;
#define BME280_DIG_P5_REG = 0x97;
#define BME280_DIG_P6_REG = 0x99;
#define BME280_DIG_P7_REG = 0x9B;
#define BME280_DIG_P8_REG = 0x9D;
#define BME280_DIG_P9_REG = 0x9F;
#define BME280_DIG_H1_REG = 0xA1;  //uint8_t
#define BME280_DIG_H2_REG = 0xE1;  //int16_t
#define BME280_DIG_H3_REG = 0xE3;  //uint8_t
#define BME280_DIG_H4_REG = 0xE4;  //int16_t   E4[11:4]/E5[3:0]
#define BME280_DIG_H5_REG = 0xE5;  //int16_t   Ee[3:0]/E5[11:4]
#define BME280_DIG_H6_REG = 0xE7;  //int8_t   


#define BME280_I2C_ADDRESS ((uint8_t)0b1110110)
#define BME280_I2C_ADDRESS_RW ((uint8_t)0b11101101)

int32_t BME280_write_reg(uint8_t reg, uint8_t *data, uint8_t len);
int32_t BME280_read_reg(uint8_t reg, uint8_t *data, uint8_t len);

void BME280_full_read(uint8_t *data);
int32_t BME280_read_compensation_values(uint8_t *data);
int32_t BME280_set_compensation_values(uint8_t *data);

void BME280_Init();

int32_t BME280_compensate_T_int32(int32_t adc_T, uint16_t dig_T1, int16_t dig_T2, int16_t dig_T3);
uint32_t BME280_compensate_P_int32(int32_t adc_P, uint16_t dig_P1, int16_t dig_P2, int16_t dig_P3,
 int16_t dig_P4, int16_t dig_P5, int16_t dig_P6, int16_t dig_P7, int16_t dig_P8, int16_t dig_P9);
uint32_t BME280_compensate_H_int32(int32_t adc_H, uint8_t dig_H1, int16_t dig_H2, uint8_t dig_H3,
 int16_t dig_H4, int16_t dig_H5, int8_t dig_H6);



#endif /* __MAIN_H */