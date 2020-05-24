
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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRI/src/pdp_decode.cpp.o' failedBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//Enum to hold the API IDs

const std::string pdp_id = "0x804";

//API ID to name mappings
const std::map<int, std::string> api_id_name = 
{{0x50, "stat1"}, {0x51, "stat2"}, {0x52, "stat3"}, {0x5D, "statEnergy"}};


//Struct that holds the flags.
//Processing is done when all 4 CAN frames are received
struct flags {
    bool in_proc = false;
    //pdp_frame flag. After each status frame is read, it's bit is set to 1.
    //So, if the value is 0100 (4), that means only status 3 is available,
    //if value is 1111 (15), that means all the frames area available, and so on
    unsigned char pdp_frame = 0;
};
//Struct to hold the raw CAN message frames
struct pdp_frame {
    can_msgs::Frame stat1;
    can_msgs::Frame stat2;
    can_msgs::Frame stat3;
    can_msgs::Frame statEnergy;
};

void msg_cb(const can_msgs::Frame& f);
