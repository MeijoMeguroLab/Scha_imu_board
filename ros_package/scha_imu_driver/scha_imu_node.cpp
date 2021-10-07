// Copyright (c) 2021, MeijoMeguroLab
// All rights reserved.
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

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include <termios.h>
#include <sys/socket.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define GNSS_PACKECT_LEN 2
#define CMD_NUM_MAX (79)
#define BUFF_SIZE 1024

#ifndef SI
#define SI 9.80665
#endif

union FourByte {
    float         flt;
    unsigned long bit32;
    unsigned int bit16[2];
    unsigned char bit8[4];
};

uint32_t imu_time;
// Gyro sensor(x, y, z)
double imu_gyro[3];
// Acceleration sensor(x, y, z)
double imu_accl[3];
// Temperature sensor
double imu_temp;

unsigned char recv_buff[BUFF_SIZE] = {0};
unsigned char buff[BUFF_SIZE] = {0};

void parseIMUdata( unsigned char* imu_data ){

  FourByte data_tmp;
  float local_imu_gyro[3], local_imu_accl[3], local_temp;

  // gyro_x
  data_tmp.bit32 = (*(imu_data+5) << 24) | (*(imu_data+6) << 16 ) | (*(imu_data+7) << 8) | (*(imu_data+8));
  local_imu_gyro[0] = data_tmp.flt;

  // gyro_y
  data_tmp.bit32 = (*(imu_data+9) << 24) | (*(imu_data+10) << 16 ) | (*(imu_data+11) << 8) | (*(imu_data+12));
  local_imu_gyro[1] = data_tmp.flt;

  // gyro_z
  data_tmp.bit32 = (*(imu_data+13) << 24) | (*(imu_data+14) << 16 ) | (*(imu_data+15) << 8) | (*(imu_data+16));
  local_imu_gyro[2] = data_tmp.flt;

  // accl_x
  data_tmp.bit32 = (*(imu_data+17) << 24) | (*(imu_data+18) << 16 ) | (*(imu_data+19) << 8) | (*(imu_data+20));
  local_imu_accl[0] = data_tmp.flt;

  // accl_y
  data_tmp.bit32 = (*(imu_data+21) << 24) | (*(imu_data+22) << 16 ) | (*(imu_data+23) << 8) | (*(imu_data+24));
  local_imu_accl[1] = data_tmp.flt;

  // accl_z
  data_tmp.bit32 = (*(imu_data+25) << 24) | (*(imu_data+26) << 16 ) | (*(imu_data+27) << 8) | (*(imu_data+28));
  local_imu_accl[2] = data_tmp.flt;

  // temperature
  data_tmp.bit32 = (*(imu_data+29) << 24) | (*(imu_data+30) << 16 ) | (*(imu_data+31) << 8) | (*(imu_data+32));
  local_temp = data_tmp.flt;

  // 32bit convert
  for (int i=0; i < 3; i++)
  {
    /* scha6xx */
    imu_gyro[i] = (double)(local_imu_gyro[i] * (M_PI/180));
    imu_accl[i] = (double)(local_imu_accl[i] * SI);
  }

  imu_temp = (double)local_temp;
}

class logger
{
public:

  //! File descripter for USB-ISS
  int fd_;
  //! Saved terminal config
  struct termios defaults_;

  logger()
  {
    imu_gyro[0] = 0.0;
    imu_gyro[1] = 0.0;
    imu_gyro[2] = 0.0;

    imu_accl[0] = 0.0;
    imu_accl[1] = 0.0;
    imu_accl[2] = 0.0;
  }

  ~logger()
  {

  }

  /**
   * @brief Open device
   * @param device Device file name (/dev/ttyACM*)
   * @retval 0 Success
   * @retval -1 Failure
   */
  int open_serial(const char *device_name){
    fd_=open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if( -1 == fd_ ){
      perror("open");
      return fd_;
    }

    fcntl(fd_, F_SETFL,0);
    //load configuration
    struct termios conf_tio;
    tcgetattr(fd_,&conf_tio);
    //set baudrate
    speed_t BAUDRATE = B230400;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    //non canonical, non echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    //non blocking
    conf_tio.c_cc[VMIN]=0;
    conf_tio.c_cc[VTIME]=0;
    //store configuration

    cfmakeraw(&conf_tio);
    tcsetattr(fd_,TCSANOW,&conf_tio);

    struct serial_struct serial_setting;
    ioctl(fd_, TIOCGSERIAL, &serial_setting);
    serial_setting.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCGSERIAL, &serial_setting);

    return fd_;
  }

  /**
   * @brief Close device
   */
  void closePort()
  {
    if (tcsetattr(fd_, TCSANOW, &defaults_) < 0)
    {
      perror("closePort");
    }
    close(fd_);
  }
};
/* ------------------------------------------------------- */

class LoggerNode
{
public:
  logger target;
  ros::NodeHandle node_handle_;
  ros::Publisher sensor_data_pub_;
  ros::Publisher imu_temp_publisher_;

  std::string device_;
  std::string topic_name_imu_;
  std::string topic_name_imu_temp;

  explicit LoggerNode(ros::NodeHandle nh)
    : node_handle_(nh)
  {
    // Read parameters
    const std::string device_key = ros::this_node::getName() + std::string("/device_");
    const std::string topic_key = ros::this_node::getName() + std::string("/topic_name_imu_");
    const std::string topic_key_temp = ros::this_node::getName() + std::string("/topic_name_imu_temp");

    node_handle_.param(device_key, device_, std::string("/dev/ttyACM0"));
    node_handle_.param(topic_key, topic_name_imu_, std::string("/imu/data_raw"));
    node_handle_.param(topic_key_temp, topic_name_imu_temp, std::string("/imu/temp"));

    ROS_INFO("device: %s", device_.c_str());
    ROS_INFO("topic: %s", topic_name_imu_.c_str());
    ROS_INFO("topic: %s", topic_name_imu_temp.c_str());

    // Data publisher
    sensor_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>(topic_name_imu_, 100);
    imu_temp_publisher_ = node_handle_.advertise<std_msgs::Float64>(topic_name_imu_temp, 100);
  }

  ~LoggerNode()
  {
    target.closePort();
  }

  /**
   * @brief Open IMU device file
   */
  bool open(void)
  {
    int fd1 = 0;
    // Open device file
    fd1 = target.open_serial(device_.c_str());
    if (fd1 < 0)
    {
      ROS_ERROR("Failed to open device %s", device_.c_str());
      return fd1;
    }

    return fd1;
  }
  void close()
  {
    target.closePort();
  }

  void publish_imu_data()
  {
    sensor_msgs::Imu data;
    std_msgs::Float64 ImuTempMsg;
    data.header.frame_id = "imu";
    data.header.stamp = ros::Time::now();

    data.linear_acceleration.x = imu_accl[0];
    data.linear_acceleration.y = imu_accl[1];
    data.linear_acceleration.z = imu_accl[2];

    data.angular_velocity.x = imu_gyro[0];
    data.angular_velocity.y = imu_gyro[1];
    data.angular_velocity.z = imu_gyro[2];

    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 1;

    ImuTempMsg.data = imu_temp;

    sensor_data_pub_.publish(data);
    imu_temp_publisher_.publish(ImuTempMsg);
  }


  bool spin()
  {
    uint32_t i,size = 0;

    while (ros::ok())
    {
      ros::spinOnce();

      size = read(target.fd_, buff, sizeof(buff));
      if(size > 0){

        bool isProtcol = false;
        for(int t = 0;t < size; t++){

          if( false == isProtcol ){
            if(((buff[t]&0xFF)==0xF7)&&((buff[t+1]&0xFF)==0xE0)){
              isProtcol = true;
            }
          }else{
            if(buff[t]==0x49){
              parseIMUdata(&buff[t]);
              publish_imu_data();
              t += 32;
            }
          }
        }
      }
    }
  }

};

/* ------------------ */
/* main               */
/* ------------------ */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "scha_imu_node");
  ros::NodeHandle nh("~");

  LoggerNode node(nh);
  if(node.open() < 0){
    ros::shutdown();
    return 0;
  }

  node.spin();

  return 0;
}
