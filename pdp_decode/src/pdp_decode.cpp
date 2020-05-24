/*
 * Copyright (c) 2020, Mastafa Awal
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


#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "pdp_decode.hpp"
#include <pdp_msgs/pdp_data.h>
void msg_cb(const can_msgs::Frame& f)
{
  std::cout << "FRAME ID: " << f.id << "\n";
  //Extract the API ID of the frame
  uint32_t api_id = (f.id >> 6) & 1023;
  std::cout << "APIID:" << api_id << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pdp_decode");

    ros::NodeHandle nh;

    //Find the publisher by using CAN device parameters.
    //the PDP CAN ID is 0x804.
    std::map<std::string,std::string> can_dev_types;
    std::string pdp_topic = "";
    if (ros::param::get("/can_device_types", can_dev_types)) {
        for (auto id_top_pair: can_dev_types) {
            if (pdp_id.compare(id_top_pair.first) == 0) {
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
  ros::Subscriber sub = nh.subscribe(pdp_rx_topic, 4, msg_cb);
  while (ros::ok) {
    ros::spinOnce();
  }
 
  //ros::waitForShutdown();
  return 0;
}

