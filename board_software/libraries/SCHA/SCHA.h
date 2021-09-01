//Copyright (c) 2021, Murata Electronics Oy.
//All rights reserved.

/******************************************************************************
SCHA.h
SCHA63* Arduino Library Header File

******************************************************************************/

#ifndef __SCHA_h
#define __SCHA_h

#include <stdint.h>
#include "Arduino.h"
#include <SPI.h>

#ifndef SCHA6xx_SPI_MODE
#define SCHA6xx_SPI_MODE SPI_MODE0
#endif

#define SCHA6xx_SPI_CLOCK  8000000

// Macros for parsing values from sensor MISO words
#define CONV_INT8_UPPER(a)      ((int8_t)(((a) >> 16) & 0xff))
#define CONV_INT8_LOWER(a)      ((int8_t)(((a) >> 8) & 0xff))
#define CONV_INT16(a)           ((int16_t)(((a) >> 8) & 0xffff))
#define CONV_UINT16(a)          ((uint16_t)(((a) >> 8) & 0xffff))
#define CHECK_RS_ERROR(a)  ((((a) >> 24) & 0x03) != 1 ? true : false)
#define CONV_TEMPERATURE(a)          (25 + ((a) / 30.0))


//Define Operation SPI 32bit Frame
#define WRITE_REG_BANK_0         0xFC000073
#define WRITE_RESET              0xE000017C
#define WRITE_OP_MODE_NORMAL     0xE4000067
#define WRITE_FILTER_13HZ_RATE   0xD8000045 // Set 13 Hz filter
#define WRITE_FILTER_46HZ_RATE   0xD812129E // Set 46 Hz filter
#define WRITE_FILTER_13HZ_ACC    0xE800006D // Set 13 Hz filter
#define WRITE_FILTER_46HZ_ACC    0xE8022248 // Set 46 Hz filter
#define WRITE_EOI_BIT            0xE000025B

#define READ_GYRO_X              0x040000F7
#define READ_GYRO_Y              0x0C0000FB
#define READ_GYRO_Z              0x040000F7
#define READ_ACC_X               0x100000E9
#define READ_ACC_Y               0x140000EF
#define READ_ACC_Z               0x180000E5
#define READ_TEMP                0x1C0000E3

#define READ_SUMMARY_STATUS      0x380000D5
#define READ_TRC_0               0x740000BF
#define READ_TRC_1               0x780000B5
#define READ_TRC_2               0x700000B9

#define READ_MODE                0x640000A7
#define WRITE_MODE_ASM_010       0xE40010AA
#define WRITE_MODE_ASM_001       0xE400088F
#define WRITE_MODE_ASM_100       0xE40020E0


typedef struct _scha_raw_data {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp_due;
    int16_t temp_uno;
} scha_raw_data;

typedef struct _scha_out_data {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp_due;
    float temp_uno;
} scha_out_data;

// Cross axis compensation values
typedef struct _scha_cacv {
    float cxx;
    float cxy;
    float cxz;
    float cyx;
    float cyy;
    float cyz;
    float czx;
    float czy;
    float czz;
    float bxx;
    float bxy;
    float bxz;
    float byx;
    float byy;
    float byz;
    float bzx;
    float bzy;
    float bzz;
} scha_cacv;

union FourByte {
    float         flt;
    unsigned long bit32;
    unsigned int bit16[2];
    unsigned char bit8[4];
};

class SCHA6xx {

public:
  // Constructor
  SCHA6xx(uint8_t CS1, uint8_t CS2, uint8_t EXTRESN1, uint8_t EXTRESN2);
  // Destructor
  ~SCHA6xx();
  
  SPISettings spiSettings{SCHA6xx_SPI_CLOCK, MSBFIRST, SCHA6xx_SPI_MODE};
  scha_out_data out_data;

  boolean begin(void);
  boolean available(void);
  void SCHA_SERIAL_READ();


private:
  uint8_t SCHA6xx_cs1Pin;  // Default SPI chip select pin( UNO - CSB2 )
  uint8_t SCHA6xx_cs2Pin;  // Default SPI chip select pin( DUE - CSB1 )
  uint8_t SCHA6xx_extresn;  // Default EXTRESN pin
  uint8_t SCHA6xx_extresn2; // Default EXTRESN2 pin
  uint8_t SCHA6xx_mode;     // Default inclinometer mode
  bool crcerr, statuserr;

  void beginTransmission();
  void endTransmission();
  void initSPI();
  unsigned long transfer(unsigned long value, uint8_t cs_pin);
  uint32_t SPI_SEND_DUE(uint32_t value);
  uint32_t SPI_SEND_UNO(uint32_t value);
  void SCHA_convert_data(scha_raw_data *data_in, scha_out_data *data_out);
  void SCHA_read_data(scha_raw_data *data);

};
#endif
