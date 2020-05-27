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


void clear_pdp_faults() {
  //Do stuff
}

void reset_pdp_energy() {
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

  ros::ServiceServer service = nh.advertiseService("pdp_control", send_msg);
  ROS_INFO("Ready to send PDP Control data");
  ros::spin();

  return 0;
}