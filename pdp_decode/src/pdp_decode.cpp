/*
 * Copyright (c) 2020, Mastafa Awal
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* This is a ROS package that decodes CAN packets sent by the Power Distribution
* Panel by CTRE. Reverse engineered from the FRC library
https://github.com/wpilibsuite/allwpilib/blob/master/hal/src/main/native/athena/PDP.cpp
* and related files, and a logic analyzer.
* Decoder code provided by the above FRC library
*/

//TO DO: Refactor code to make it more clean into seperate files

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "pdp_frames.hpp"
#include <pdp_msgs/pdp_data.h>

#define PDP_DECODE_LOOP_RATE_HZ (10)

const std::string pdp_can_id = "0x804";

//Enum for API Name
enum {
    Stat1 = 1, Stat2 = 2, Stat3 = 4, StatEnergy = 8, Control = 9
};

//API ID mappings
const std::map<uint32_t, int> api_id_map = 
{
    {0x50, Stat1}, 
    {0x51, Stat2}, 
    {0x52, Stat3},
    {0x5D, StatEnergy}
};

//Struct that holds the flags.
struct flags {
    bool decoding = false;
    //pdp_frame flag. After each status frame is read, it's bit is set to 1.
    //So, if the value is 0100 (4), that means only status 3 is available,
    //if value is 1111 (15), that means all the frames area available, and so on

    //Note: all we need is 4 bits. But using 16 so that it plays nicely with cout
    uint16_t avail_frame = 0;
};
//Array to hold the raw CAN message frames
//Although all we need is 4, having a size of 9 allows us to use the API ID mappings directly with the flags and arrays,
//making it more efficient

can_msgs::Frame pdp_frames[9]; 

struct flags status; 

//Run loop at 10Hz
ros::Rate loop_rate(PDP_DECODE_LOOP_RATE_HZ);

void msg_cb(const can_msgs::Frame& f)
{
  if (status.decoding) {
      return;
  }
  
  //if not currently decoding, then process the frame
  //std::cout << "FRAME ID: " << f.id << "\n";
  //Extract the API ID of the frame
  uint32_t api_id = (f.id >> 6) & 1023;
  //std::cout << "APIID:" << api_id << "\n";
  auto row = api_id_map.find(api_id);
  if (!(row == api_id_map.end())) {
    //If it exists within the map, then add it to the array, and set the corresponding flag
        pdp_frames[row->second] = f;
        status.avail_frame |= row->second;
        // std::cout << "CURR FRAME DECODED:" << row -> second << "\n";
        // std::cout << "CURR FRAMES TO BE DECODED: " << status.avail_frame << "\n";
    } 
}
void decode(pdp_msgs::pdp_data *msg) {
  PdpStatus1 pdpStatus;
  PdpStatus2 pdpStatus2;
  PdpStatus3 pdpStatus3;
  PdpStatusEnergy pdpStatusEnergy;

  //Copy over the data bits to pdpStatus data bits

  for (int i =0; i < 8; i++) {
       pdpStatus.data[i] = pdp_frames[Stat1].data[i];
       pdpStatus2.data[i] = pdp_frames[Stat2].data[i];
       pdpStatus3.data[i] = pdp_frames[Stat3].data[i];
       pdpStatusEnergy.data[i] = pdp_frames[StatEnergy].data[i];
  }

  //Decode Bus voltage
  msg->bus_voltage = pdpStatus3.bits.busVoltage * 0.05 + 4.0;
  //Decode temperature
  msg->temperature = pdpStatus3.bits.temp * 1.03250836957542 - 67.8564500484966;
  //Decode battery internal resistance
  msg->batt_int_res = pdpStatus3.bits.internalResBattery_mOhms;
  //Decode currents
  msg->currents[0] = ((static_cast<uint32_t>(pdpStatus.bits.chan1_h8) << 2) |
                 pdpStatus.bits.chan1_l2) *
                0.125;
  msg->currents[1] = ((static_cast<uint32_t>(pdpStatus.bits.chan2_h6) << 4) |
                 pdpStatus.bits.chan2_l4) *
                0.125;
  msg->currents[2] = ((static_cast<uint32_t>(pdpStatus.bits.chan3_h4) << 6) |
                 pdpStatus.bits.chan3_l6) *
                0.125;
  msg->currents[3] = ((static_cast<uint32_t>(pdpStatus.bits.chan4_h2) << 8) |
                 pdpStatus.bits.chan4_l8) *
                0.125;
  msg->currents[4] = ((static_cast<uint32_t>(pdpStatus.bits.chan5_h8) << 2) |
                 pdpStatus.bits.chan5_l2) *
                0.125;
  msg->currents[5] = ((static_cast<uint32_t>(pdpStatus.bits.chan6_h6) << 4) |
                 pdpStatus.bits.chan6_l4) *
                0.125;

  msg->currents[6] = ((static_cast<uint32_t>(pdpStatus2.bits.chan7_h8) << 2) |
                 pdpStatus2.bits.chan7_l2) *
                0.125;
  msg->currents[7] = ((static_cast<uint32_t>(pdpStatus2.bits.chan8_h6) << 4) |
                 pdpStatus2.bits.chan8_l4) *
                0.125;
  msg->currents[8] = ((static_cast<uint32_t>(pdpStatus2.bits.chan9_h4) << 6) |
                 pdpStatus2.bits.chan9_l6) *
                0.125;
  msg->currents[9] = ((static_cast<uint32_t>(pdpStatus2.bits.chan10_h2) << 8) |
                 pdpStatus2.bits.chan10_l8) *
                0.125;
  msg->currents[10] = ((static_cast<uint32_t>(pdpStatus2.bits.chan11_h8) << 2) |
                  pdpStatus2.bits.chan11_l2) *
                 0.125;
  msg->currents[11] = ((static_cast<uint32_t>(pdpStatus2.bits.chan12_h6) << 4) |
                  pdpStatus2.bits.chan12_l4) *
                 0.125;

  msg->currents[12] = ((static_cast<uint32_t>(pdpStatus3.bits.chan13_h8) << 2) |
                  pdpStatus3.bits.chan13_l2) *
                 0.125;
  msg->currents[13] = ((static_cast<uint32_t>(pdpStatus3.bits.chan14_h6) << 4) |
                  pdpStatus3.bits.chan14_l4) *
                 0.125;
  msg->currents[14] = ((static_cast<uint32_t>(pdpStatus3.bits.chan15_h4) << 6) |
                  pdpStatus3.bits.chan15_l6) *
                 0.125;
  msg->currents[15] = ((static_cast<uint32_t>(pdpStatus3.bits.chan16_h2) << 8) |
                  pdpStatus3.bits.chan16_l8) *
                 0.125;
  //Decode total current
  uint32_t raw;
  raw = pdpStatusEnergy.bits.TotalCurrent_125mAperunit_h8;
  raw <<= 4;
  raw |= pdpStatusEnergy.bits.TotalCurrent_125mAperunit_l4;
  msg->total_current = 0.125 * raw; 
  
  //Decode total power
  raw = 0;
  raw = pdpStatusEnergy.bits.Power_125mWperunit_h4;
  raw <<= 8;
  raw |= pdpStatusEnergy.bits.Power_125mWperunit_m8;
  raw <<= 4;
  raw |= pdpStatusEnergy.bits.Power_125mWperunit_l4;
  msg->power_usage =  0.125 * raw; 
}
   


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pdp_decode");

    ros::NodeHandle nh;

    ros::Rate loop_rate(PDP_DECODE_LOOP_RATE_HZ);

    //Find the publisher by using CAN device parameters.
   
    std::map<std::string,std::string> can_dev_types;
    std::string pdp_topic = "";
    if (ros::param::get("/can_device_types", can_dev_types)) {
        for (auto id_top_pair: can_dev_types) {
            if (pdp_can_id.compare(id_top_pair.first) == 0) {
                pdp_topic = id_top_pair.second;
            }
        }
    } else {
        ROS_WARN("CAN Device type data not found.");
        return 1;
    }

  if (pdp_topic.empty()) {
      ROS_WARN("Unable to find PDP raw CAN frames topic.");;
      return 1;
  }
  std::string can_rx_topic;
  std::string pdp_rx_topic;
  if (ros::param::get("/can_topic_rx", can_rx_topic)) {
       pdp_rx_topic= can_rx_topic + "/" + pdp_topic;
      //std::cout << pdp_rx_topic << "\n";
      
  } else {
      ROS_WARN("Unable to find raw CAN Receive Topic during PDP decode");
      return 1;
  }
  ros::Subscriber sub = nh.subscribe(pdp_rx_topic, 5, msg_cb);

  //TODO: replace with value from parameter server

  ros::Publisher pdp_pub = nh.advertise<pdp_msgs::pdp_data>("/rover/power_data", 10);
  //Second pub for temp?
  while (ros::ok) {
    ros::spinOnce();
    if (status.avail_frame == 0xF) {
        status.decoding = true;
        pdp_msgs::pdp_data msg;
        decode(&msg);
        pdp_pub.publish(msg);

        // std::cout << "STATUS 1:" << pdp_frames[Stat1].id << "\n";
        // std::cout << "STATUS 2:" << pdp_frames[Stat2].id << "\n";
        // std::cout << "STATUS 3:" << pdp_frames[Stat3].id << "\n";
        // std::cout << "STATUS ENERGY:" << pdp_frames[StatEnergy].id << "\n";
       
        //Clear the flag
        //std::cout << "CLEARING FLAGS\n";
        status.avail_frame = 0;
        status.decoding = false;
        loop_rate.sleep();
    }
  }

  //ros::waitForShutdown();
  return 0;
}

