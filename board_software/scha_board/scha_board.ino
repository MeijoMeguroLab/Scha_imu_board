// Copyright (c) 2021, MeijoMeguroLab
// All rights reserved.
//
// This software is based in part on the source code available for download from
// https://www.murata.com/ja-jp/products/sensor/gyro/overview/lineup/scha63t/form
// The copyright of the quoted material is held by Murata Electronics Oy.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the MeijoMeguroLab nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include <SCHA.h>
#include <SPI.h>
#include "USBHost_t36.h"
#include <math.h>

#define USBBAUD       230400
#define IMU_BUFF_SIZE 33

#define interval_value   20 /* IMU polling Interval value (ms) */

#define DEBUG_FLAG 0

// ---- IMU ----
SCHA6xx IMU(9, 10, 8, 7);

unsigned long prev;
int isReceived = 0;
int16_t imu_data[14];

// ---- General ----
unsigned char imu_buff[ IMU_BUFF_SIZE ];
uint8_t sendbuff[ 256 ];

// ---- ---- ----


void sensorRead() {

  FourByte data_tmp;
  data_tmp.bit16[0]=0;data_tmp.bit16[1]=0;
  IMU.available();

  data_tmp.flt = IMU.out_data.gyro_x;
  imu_data[0] = (data_tmp.bit32>>16)&0xFFFF;
  imu_data[1] = (data_tmp.bit32)&0xFFFF;
  data_tmp.flt = IMU.out_data.gyro_y;
  imu_data[2] = (data_tmp.bit32>>16)&0xFFFF;
  imu_data[3] = (data_tmp.bit32)&0xFFFF;
  data_tmp.flt = IMU.out_data.gyro_z;
  imu_data[4] = (data_tmp.bit32>>16)&0xFFFF;
  imu_data[5] = (data_tmp.bit32)&0xFFFF;

  data_tmp.flt = IMU.out_data.acc_x;
  imu_data[6] = (data_tmp.bit32>>16)&0xFFFF;
  imu_data[7] = (data_tmp.bit32)&0xFFFF;
  data_tmp.flt = IMU.out_data.acc_y;
  imu_data[8] = (data_tmp.bit32>>16)&0xFFFF;
  imu_data[9] = (data_tmp.bit32)&0xFFFF;
  data_tmp.flt = IMU.out_data.acc_z;
  imu_data[10] = (data_tmp.bit32>>16)&0xFFFF;
  imu_data[11] = (data_tmp.bit32)&0xFFFF;

  data_tmp.flt = IMU.out_data.temp_due;
  imu_data[12] = (data_tmp.bit32>>16)&0xFFFF;
  imu_data[13] = (data_tmp.bit32)&0xFFFF;
}


void setup() {

  while (!Serial) ;
  Serial.begin(USBBAUD);
  
  delay(2000);
  if(IMU.begin() == false) {
    Serial.println("Murata SCHA6xx not connected.");
    while(1); //Freeze
  }
  delay(10);

  memset(sendbuff, 0x00, 256);
  memset(imu_data, 0x00, 14);
}

void loop() {

  int send_length;
  unsigned long curr = millis();
  if ((curr - prev) >= interval_value) {
    sensorRead();
    isReceived = 1;
    prev = curr;

    imu_buff[0] = 0x49; // 'I' hex = 0x49
    imu_buff[1] = (curr >> 24)&0xFF; // Timestap [1]-[4]
    imu_buff[2] = (curr >> 16)&0xFF;
    imu_buff[3] = (curr >> 8)&0xFF;
    imu_buff[4] = (curr)&0xFF;
    // Gyro
    imu_buff[5] = (imu_data[0]>>8)&0xFF;
    imu_buff[6] = (imu_data[0])&0xFF;
    imu_buff[7] = (imu_data[1]>>8)&0xFF;
    imu_buff[8] = (imu_data[1])&0xFF;
    imu_buff[9] = (imu_data[2]>>8)&0xFF;
    imu_buff[10] = (imu_data[2])&0xFF;
    imu_buff[11] = (imu_data[3]>>8)&0xFF;
    imu_buff[12] = (imu_data[3])&0xFF;
    imu_buff[13] = (imu_data[4]>>8)&0xFF;
    imu_buff[14] = (imu_data[4])&0xFF;
    imu_buff[15] = (imu_data[5]>>8)&0xFF;
    imu_buff[16] = (imu_data[5])&0xFF;
    // Accl
    imu_buff[17] = (imu_data[6]>>8)&0xFF;
    imu_buff[18] = (imu_data[6])&0xFF;
    imu_buff[19] = (imu_data[7]>>8)&0xFF;
    imu_buff[20] = (imu_data[7])&0xFF;
    imu_buff[21] = (imu_data[8]>>8)&0xFF;
    imu_buff[22] = (imu_data[8])&0xFF;
    imu_buff[23] = (imu_data[9]>>8)&0xFF;
    imu_buff[24] = (imu_data[9])&0xFF;
    imu_buff[25] = (imu_data[10]>>8)&0xFF;
    imu_buff[26] = (imu_data[10])&0xFF;
    imu_buff[27] = (imu_data[11]>>8)&0xFF;
    imu_buff[28] = (imu_data[11])&0xFF;
    //TEMP
    imu_buff[29] = (imu_data[12]>>8)&0xFF;
    imu_buff[30] = (imu_data[12])&0xFF;
    imu_buff[31] = (imu_data[13]>>8)&0xFF;
    imu_buff[32] = (imu_data[13])&0xFF;
  }

  if(isReceived == 1) {
    /* Set Header Data */
    sendbuff[0] = 0xF7;
    sendbuff[1] = 0xE0;
    send_length = 2;
    
    // Set IMU Data
    memcpy(&sendbuff[send_length], imu_buff, IMU_BUFF_SIZE);
    send_length += IMU_BUFF_SIZE;
    isReceived = 0;

#if DEBUG_FLAG
    Serial.printf("ACC(X,Y,Z):%+.3f,%+.3f,%+.3f[g] GYRO(X,Y,Z):%+.3f,%+.3f,%+.3f[dps]\n", 
        IMU.out_data.acc_x, IMU.out_data.acc_y,IMU.out_data.acc_z, 
        IMU.out_data.gyro_x,IMU.out_data.gyro_y, IMU.out_data.gyro_z);

#else
    Serial.write((byte*)sendbuff, send_length);
    memset(sendbuff, 0x00, send_length);
#endif

  }

}
