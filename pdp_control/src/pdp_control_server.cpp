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


#include "ros/ros.h"
#include <pdp_control/PdpControl.h>
#include <can_msgs/Frame.h>
#include "can_api/CANAPI.hpp"

uint8_t api_id = 0x70;
ros::Publisher pdp_pub; 

void clear_pdp_faults() {
  can_msgs::Frame msg;
  // std::cout << "BEFORE ID" << msg.id << "\n";
  // std::cout << "DATA SIZE " << unsigned(msg.dlc) << "\n";
  // std::cout << "DATA[0] " << unsigned(msg.data[0]) << "\n";
  std::vector<uint8_t> data = {0x80};
  CANAPI::data_to_can(&msg, api_id, data);
  // std::cout << "AFTER ID" << msg.id << "\n";
  // std::cout << "DATA SIZE " << unsigned(msg.dlc) << "\n";
  // std::cout << "DATA[0]: " << unsigned(msg.data[0]) << "\n";
  pdp_pub.publish(msg);
  //Do stuff
}

void reset_pdp_energy() {
  //Do stuff
    can_msgs::Frame msg;
  // std::cout << "BEFORE ID" << msg.id << "\n";
  // std::cout << "DATA SIZE " << unsigned(msg.dlc) << "\n";
  // std::cout << "DATA[0] " << unsigned(msg.data[0]) << "\n";
  std::vector<uint8_t> data = {0x40};
  CANAPI::data_to_can(&msg, api_id, data);
  // std::cout << "AFTER ID" << msg.id << "\n";
  // std::cout << "DATA SIZE " << unsigned(msg.dlc) << "\n";
  // std::cout << "DATA[0]: " << unsigned(msg.data[0]) << "\n";
  pdp_pub.publish(msg);
  //Do stuff
}
bool send_msg(pdp_control::PdpControl::Request  &req,
         pdp_control::PdpControl::Request &res)
{
  if (req.clear_sticky_faults) {
    ROS_INFO("Clearing PDP sticky faults");
    clear_pdp_faults();
  }
  
  if (req.reset_total_energy) {
    ROS_INFO("Resetting PDP total energy");
    reset_pdp_energy();
  }
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "PDPControlServer");
  ros::NodeHandle nh;
   //Change for modularity
  ros::ServiceServer service = nh.advertiseService("/rover/pdp_control", send_msg);
  std::string topic_tx; 
  if (!ros::param::get("/can_topic_tx", topic_tx)) {
   	ROS_WARN("CAN Topic Transmit name not found, raw CAN data will be sent from topic: sent_messages");
    topic_tx = "sent_messages";
  }

  pdp_pub = nh.advertise<can_msgs::Frame>(topic_tx, 10);
  ROS_INFO("Ready to send PDP Control data");
  ros::spin();

  return 0;
}