//Copyright (c) 2021, Murata Electronics Oy.
//All rights reserved.

/******************************************************************************
SCHA.cpp
SCHA63* Arduino Driver

******************************************************************************/

#include "SCHA.h"

//#define SCHA6xx_x01
//#define SCHA6xx_x02
#define SCHA6xx_x03

#define SENSITIVITY_ACC 4905
#define SI 9.80665

#ifdef SCHA6xx_x01
#define SENSITIVITY_GYRO_X 160
#define SENSITIVITY_GYRO_Y 160
#define SENSITIVITY_GYRO_Z 160
#elif defined SCHA6xx_x02
#define SENSITIVITY_GYRO_X 160
#define SENSITIVITY_GYRO_Y 160
#define SENSITIVITY_GYRO_Z 80
#elif defined SCHA6xx_x03
#define SENSITIVITY_GYRO_X 80
#define SENSITIVITY_GYRO_Y 80
#define SENSITIVITY_GYRO_Z 80
#endif

// FILTER
#define ACC_FILTER  WRITE_FILTER_46HZ_ACC
#define RATE_FILTER WRITE_FILTER_46HZ_RATE

#define MAX_INITIAL 2

static scha_cacv scha_cac_values; // Cross-axis compensation values


static uint8_t CRC8(uint8_t BitValue, uint8_t CRC) {

  uint8_t Temp;
  Temp = (uint8_t)(CRC & 0x80);
  if (BitValue == 0x01) {
    Temp ^= 0x80;
  }
  CRC <<= 1;
  if (Temp > 0) {
    CRC ^= 0x1D;
  }
  return CRC;
}

static uint8_t CalculateCRC(uint32_t Data) {

  uint8_t BitIndex;
  uint8_t BitValue;
  uint8_t CRC;

  CRC = 0xFF;
  for (BitIndex = 31; BitIndex > 7; BitIndex--) {
    BitValue = (uint8_t)((Data >> BitIndex) & 0x01);
    CRC = CRC8(BitValue, CRC);
  }
  CRC = (uint8_t)~CRC;
  return CRC;
}


// Public Methods //////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////
SCHA6xx::SCHA6xx(uint8_t CS1, uint8_t CS2, uint8_t EXTRESN1, uint8_t EXTRESN2) {
  SCHA6xx_cs1Pin = CS1;
  SCHA6xx_cs2Pin = CS2;
  SCHA6xx_extresn = EXTRESN1;
  SCHA6xx_extresn2 = EXTRESN2;
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
SCHA6xx::~SCHA6xx() {
  SPI.end();
}

void SCHA6xx::beginTransmission() {

  SPI.setBitOrder( MSBFIRST ); // MSBFIRST
  SPI.setClockDivider( SPI_CLOCK_DIV2 );// clock 16/2=8MHz , scha=clock range( 0.1-10 MHz)
  SPI.setDataMode( SPI_MODE0 );
}


void SCHA6xx::endTransmission() {

  digitalWrite(SCHA6xx_cs1Pin, HIGH);
  digitalWrite(SCHA6xx_cs2Pin, HIGH);
  SPI.endTransaction();
  unsigned long startmillis = millis();
  while (millis() - startmillis < 1);
}

void SCHA6xx::initSPI() {

  pinMode(SCHA6xx_extresn, OUTPUT);
  pinMode(SCHA6xx_extresn2, OUTPUT);
  pinMode(SCHA6xx_cs1Pin, OUTPUT);
  pinMode(SCHA6xx_cs2Pin, OUTPUT);

  digitalWrite(SCHA6xx_extresn, HIGH);
  digitalWrite(SCHA6xx_extresn2, HIGH);
  digitalWrite(SCHA6xx_cs1Pin, HIGH);
  digitalWrite(SCHA6xx_cs2Pin, HIGH);

  beginTransmission();

  SPI.begin();
}


boolean SCHA6xx::begin(void) {

  int initial_cnt;
  uint32_t resp_due;
  uint32_t resp_uno;
  bool due_ok = false;
  bool uno_ok = false;

  initSPI();

  SPI_SEND_DUE(WRITE_RESET);
  SPI_SEND_DUE(WRITE_REG_BANK_0);
  SPI_SEND_UNO(WRITE_RESET);

  delay(25);

  SPI_SEND_DUE(WRITE_MODE_ASM_010);
  SPI_SEND_DUE(READ_MODE);
  SPI_SEND_DUE(WRITE_MODE_ASM_001);
  SPI_SEND_DUE(READ_MODE);
  SPI_SEND_DUE(WRITE_MODE_ASM_100);
  SPI_SEND_DUE(READ_MODE);
  uint32_t resp = SPI_SEND_DUE(READ_MODE);

  if ((CONV_UINT16(resp) & 0x7) == 7) {

    SPI_SEND_DUE(0xFC00051A);
    SPI_SEND_DUE(0x2C0000CB);

    uint32_t cxx_cxy = SPI_SEND_DUE(0x4C00009B);
    uint32_t cxz_cyx = SPI_SEND_DUE(0x50000089);
    uint32_t cyy_cyz = SPI_SEND_DUE(0x5400008F);
    uint32_t czx_czy = SPI_SEND_DUE(0x58000085);
    uint32_t czz_bxx = SPI_SEND_DUE(0x5C000083);
    uint32_t bxy_bxz = SPI_SEND_DUE(0x600000A1);
    uint32_t byx_byy = SPI_SEND_DUE(0x6C0000AB);
    uint32_t byz_bzx = SPI_SEND_DUE(0x700000B9);
    uint32_t bzy_bzz = SPI_SEND_DUE(0x700000B9);

    scha_cac_values.cxx = CONV_INT8_LOWER(cxx_cxy) / 4096.0 + 1;
    scha_cac_values.cxy = CONV_INT8_UPPER(cxx_cxy) / 4096.0;
    scha_cac_values.cxz = CONV_INT8_LOWER(cxz_cyx) / 4096.0;
    scha_cac_values.cyx = CONV_INT8_UPPER(cxz_cyx) / 4096.0;
    scha_cac_values.cyy = CONV_INT8_LOWER(cyy_cyz) / 4096.0 + 1;
    scha_cac_values.cyz = CONV_INT8_UPPER(cyy_cyz) / 4096.0;
    scha_cac_values.czx = CONV_INT8_LOWER(czx_czy) / 4096.0;
    scha_cac_values.czy = CONV_INT8_UPPER(czx_czy) / 4096.0;
    scha_cac_values.czz = CONV_INT8_LOWER(czz_bxx) / 4096.0 + 1;
    scha_cac_values.bxx = CONV_INT8_UPPER(czz_bxx) / 4096.0 + 1;
    scha_cac_values.bxy = CONV_INT8_LOWER(bxy_bxz) / 4096.0;
    scha_cac_values.bxz = CONV_INT8_UPPER(bxy_bxz) / 4096.0;
    scha_cac_values.byx = CONV_INT8_LOWER(byx_byy) / 4096.0;
    scha_cac_values.byy = CONV_INT8_UPPER(byx_byy) / 4096.0 + 1;
    scha_cac_values.byz = CONV_INT8_LOWER(byz_bzx) / 4096.0;
    scha_cac_values.bzx = CONV_INT8_UPPER(byz_bzx) / 4096.0;
    scha_cac_values.bzy = CONV_INT8_LOWER(bzy_bzz) / 4096.0;
    scha_cac_values.bzz = CONV_INT8_UPPER(bzy_bzz) / 4096.0 + 1;
  } else {
    return false;
  }

  SPI_SEND_DUE(WRITE_REG_BANK_0);
  SPI_SEND_DUE(WRITE_RESET);

  delay(25);

  SPI_SEND_UNO(WRITE_OP_MODE_NORMAL);
  SPI_SEND_DUE(WRITE_OP_MODE_NORMAL);
  SPI_SEND_DUE(WRITE_OP_MODE_NORMAL);
  delay(70);
  
  SPI_SEND_UNO(RATE_FILTER);
  SPI_SEND_UNO(ACC_FILTER);
  
  SPI_SEND_DUE(WRITE_RESET);
  delay(25);

  SPI_SEND_DUE(WRITE_OP_MODE_NORMAL);
  SPI_SEND_DUE(WRITE_OP_MODE_NORMAL);
  
  delay(1);
  SPI_SEND_DUE(RATE_FILTER);
  
  for(initial_cnt = 0; initial_cnt < MAX_INITIAL ; initial_cnt++){
    
    delay(405);
    
    SPI_SEND_DUE(WRITE_EOI_BIT);
    SPI_SEND_UNO(WRITE_EOI_BIT);
    
    /* UNO */
    SPI_SEND_UNO(READ_SUMMARY_STATUS);
    SPI_SEND_UNO(READ_SUMMARY_STATUS);
    delay(3);
    resp_uno = SPI_SEND_UNO(READ_SUMMARY_STATUS);
    uno_ok = CHECK_RS_ERROR(resp_uno) == true ? false: true;
    
    /* DUE */
    SPI_SEND_DUE(READ_SUMMARY_STATUS);
    SPI_SEND_DUE(READ_SUMMARY_STATUS);
    delay(3);
    resp_due = SPI_SEND_DUE(READ_SUMMARY_STATUS);
    due_ok = CHECK_RS_ERROR(resp_due) == true ? false: true;
    if ((due_ok == false || uno_ok == false) && ( initial_cnt < MAX_INITIAL - 1 )) {
      
      SPI_SEND_UNO(WRITE_RESET);
      SPI_SEND_DUE(WRITE_RESET);
      
      delay(25);
      SPI_SEND_UNO(WRITE_OP_MODE_NORMAL);
      SPI_SEND_DUE(WRITE_OP_MODE_NORMAL);
      SPI_SEND_DUE(WRITE_OP_MODE_NORMAL);
      delay(50);
      SPI_SEND_UNO(RATE_FILTER);
      SPI_SEND_UNO(ACC_FILTER);
      SPI_SEND_DUE(RATE_FILTER);
      delay(45);
    } else {
      break;
    }
  }

  if (due_ok == false || uno_ok == false) {
    return false;
  }

  return true;
}


boolean SCHA6xx::available(void) {

  scha_raw_data raw_data;
  scha_out_data scha_data;

  SCHA_read_data(&raw_data);
  SCHA_convert_data(&raw_data, &scha_data);

  out_data = scha_data;

  return true;
}


unsigned long SCHA6xx::transfer(unsigned long value, uint8_t cs_pin) {

  FourByte dataorig;
  FourByte senddata;
  uint8_t CMD, CRC;
  senddata.bit32 = value;

  digitalWrite(cs_pin, LOW);
  delayMicroseconds(10);

  for (int i = 3; i >= 0; i--) {
    dataorig.bit8[i] = SPI.transfer(senddata.bit8[i]);
  }

  CRC = dataorig.bit8[0];
  CMD = dataorig.bit8[3];

  delayMicroseconds(10);
  digitalWrite(cs_pin, HIGH);

  if (CRC == CalculateCRC(dataorig.bit32)) {
    crcerr = false;
  } else {
    crcerr = true;
  }

  if ((CMD && 0x03) == 0x01) {
    statuserr = false;
  } else {
    statuserr = true;
  }

  return dataorig.bit32;
}


/*----------------------------------------------------------------------------
  SPI communication to ASIC1 (DUE) using CS2 ( CS2=Circuit Diagram Define )
 *----------------------------------------------------------------------------*/
uint32_t SCHA6xx::SPI_SEND_DUE(uint32_t value) {
  return transfer(value, SCHA6xx_cs1Pin);
}


/*----------------------------------------------------------------------------
  SPI communication to ASIC2 (UNO) using CS1 ( CS1=Circuit Diagram Define )
 *----------------------------------------------------------------------------*/
uint32_t SCHA6xx::SPI_SEND_UNO(uint32_t value) {
  return transfer(value, SCHA6xx_cs2Pin);
}


void SCHA6xx::SCHA_convert_data(scha_raw_data *data_in, scha_out_data *data_out) {

  data_out->acc_x = data_in->acc_x;
  data_out->acc_y = data_in->acc_y;
  data_out->acc_z = data_in->acc_z;
  data_out->gyro_x = data_in->gyro_x;
  data_out->gyro_y = data_in->gyro_y;
  data_out->gyro_z = data_in->gyro_z;

  data_out->acc_x = (data_out->acc_x / SENSITIVITY_ACC);
  data_out->acc_y = (data_out->acc_y / SENSITIVITY_ACC);
  data_out->acc_z = (data_out->acc_z / SENSITIVITY_ACC);
  data_out->gyro_x = (data_out->gyro_x / SENSITIVITY_GYRO_X);
  data_out->gyro_y = (data_out->gyro_y / SENSITIVITY_GYRO_Y);
  data_out->gyro_z = (data_out->gyro_z / SENSITIVITY_GYRO_Z);

  data_out->temp_due = CONV_TEMPERATURE(data_in->temp_due);
  data_out->temp_uno = CONV_TEMPERATURE(data_in->temp_uno);

  data_out->acc_x = (scha_cac_values.bxx * data_out->acc_x) + (scha_cac_values.bxy * data_out->acc_y) + (scha_cac_values.bxz * data_out->acc_z);
  data_out->acc_y = (scha_cac_values.byx * data_out->acc_x) + (scha_cac_values.byy * data_out->acc_y) + (scha_cac_values.byz * data_out->acc_z);
  data_out->acc_z = (scha_cac_values.bzx * data_out->acc_x) + (scha_cac_values.bzy * data_out->acc_y) + (scha_cac_values.bzz * data_out->acc_z);
  data_out->gyro_x = (scha_cac_values.cxx * data_out->gyro_x) + (scha_cac_values.cxy * data_out->gyro_y) + (scha_cac_values.cxz * data_out->gyro_z);
  data_out->gyro_y = (scha_cac_values.cyx * data_out->gyro_x) + (scha_cac_values.cyy * data_out->gyro_y) + (scha_cac_values.cyz * data_out->gyro_z);
  data_out->gyro_z = (scha_cac_values.czx * data_out->gyro_x) + (scha_cac_values.czy * data_out->gyro_y) + (scha_cac_values.czz * data_out->gyro_z);
}


void SCHA6xx::SCHA_read_data(scha_raw_data *data) {

  SPI_SEND_DUE(READ_GYRO_Y);
  uint32_t gyro_y   = SPI_SEND_DUE(READ_GYRO_Z);
  uint32_t gyro_z   = SPI_SEND_DUE(READ_TEMP);
  uint32_t temp_due = SPI_SEND_DUE(READ_TEMP);

  SPI_SEND_UNO(READ_GYRO_X);
  uint32_t gyro_x   = SPI_SEND_UNO(READ_ACC_X);
  uint32_t acc_x    = SPI_SEND_UNO(READ_ACC_Y);
  uint32_t acc_y    = SPI_SEND_UNO(READ_ACC_Z);
  uint32_t acc_z    = SPI_SEND_UNO(READ_TEMP);
  uint32_t temp_uno = SPI_SEND_UNO(READ_TEMP);

  data->acc_x = CONV_INT16(acc_x);
  data->acc_y = CONV_INT16(acc_y);
  data->acc_z = CONV_INT16(acc_z);
  data->gyro_x = CONV_INT16(gyro_x);
  data->gyro_y = CONV_INT16(gyro_y);
  data->gyro_z = CONV_INT16(gyro_z);
  data->temp_due = CONV_INT16(temp_due);
  data->temp_uno = CONV_INT16(temp_uno);
}


void SCHA6xx::SCHA_SERIAL_READ() {
#if 1
  // Read UNO asic serial number
  SPI_SEND_UNO(READ_TRC_2);
  uint16_t trc_2 = CONV_UINT16(SPI_SEND_UNO(READ_TRC_0));
  uint16_t trc_0 = CONV_UINT16(SPI_SEND_UNO(READ_TRC_1));
  uint16_t trc_1 = CONV_UINT16(SPI_SEND_UNO(READ_TRC_1));
#else
  // Read DUE asic serial number
  SPI_SEND_DUE(READ_TRC_2);
  uint16_t trc_2 = CONV_UINT16(SPI_SEND_DUE(READ_TRC_0));
  uint16_t trc_0 = CONV_UINT16(SPI_SEND_DUE(READ_TRC_1));
  uint16_t trc_1 = CONV_UINT16(SPI_SEND_DUE(READ_TRC_1));
#endif

  uint16_t id_1 = (trc_2 >> 8) & 0x0f;
  uint16_t id_0 = trc_0 & 0xffff;
  uint16_t id_2 = trc_1 & 0xffff;

  char serial_num[14];
  snprintf(serial_num, 14, "%05d%01x%04x", id_2, id_1, id_0);
  Serial.print(serial_num);

  Serial.printf("\n");

}

