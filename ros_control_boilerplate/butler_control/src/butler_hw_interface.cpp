/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Butler
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <butler_control/butler_hw_interface.h>
#include <boost/unordered_set.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/make_shared.hpp>
#include <socketcan_interface/bcm.h>
#include <socketcan_interface/string.h>
#include <socketcan_interface/socketcan.h>

using namespace can;

namespace butler_control
{

union can_data{
  uint8_t val[8];
  unsigned char bytes_unsigned[8];
};

BCMsocket bcm;
Header header;
boost::shared_ptr<DriverInterface> g_driver;

boost::chrono::duration<double> can_period(5000000);

ButlerHWInterface::ButlerHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("butler_hw_interface", "ButlerHWInterface Ready.");
}

Header toheader(const std::string& s) {
  if (s.empty() || s.size() > 4)
          return MsgHeader(0xfff); // invalid

  std::stringstream stream;
  stream << std::hex << s;
  unsigned int h = 0;
  stream >> h;
  return Header(h & Header::ID_MASK, h & Header::EXTENDED_MASK,
                  h & Header::RTR_MASK, h & Header::ERROR_MASK);
}

//Frame Listener
void frame_reader(const Frame &f)
{

  std::cout << "frame_reader" << std::endl;
  //Get values
  uint8_t output[8];
  for(int i=0; i < f.dlc; ++i){
    output[i] = f.data[i];
    std::cout << std::hex << " " << (int) f.data[i];
  }
  std::cout << std::dec << std::endl;
  std::cout << output[0] << std::endl; 
  std::cout << output[1] << std::endl; 

  //Set values
  g_driver->shutdown();

}



void ButlerHWInterface::init()
{
  GenericHWInterface::init();

  std::string canport = "vcan0";

  // Init if not inited already 
  bcm.init(canport);
  
  // Set the CAN header
  std::string head_str = "5#";
  header = toheader(head_str);

  //double intime = atof("5000000");

  //can_period(intime);

  std::cout << "init done" << std::endl;
  //Socketcan listener
  g_driver = boost::make_shared<SocketCANInterface>();
  CommInterface::FrameListener::Ptr frame_printer = g_driver->createMsgListener(frame_reader);

  if(!g_driver->init(canport, false)){
  }

  //g_driver->run();
  
  //g_driver->shutdown();
  //g_driver.reset();

  // Dunno if this is needed
  // Resize vectors
  //joint_position_prev_.resize(num_joints_, 0.0);
}

void ButlerHWInterface::read(ros::Duration &elapsed_time)
{
  // Error reporting 

  // if (error) CRASH!

  std::cout << "read" << std::endl;

  //g_driver->run();

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}



Frame toframe2(Header header, const uint8_t *in_data, size_t in_size) {
  Frame frame(header);
  //Copies can_data bytes to frame.data
  if (header.isValid())
  {
      if (in_size > 8) 
          return MsgHeader(0xfff);    
      for (size_t i = 0; i < in_size; i++)
      {
          frame.data[i] = in_data[i];
      }
      frame.dlc = in_size;
  }
  return frame; 
}

// -100 - 100
// range is 200
// 0 = -180 degrees
// 200 = 180 degrees
// avrunda neråt

void ButlerHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // Process data
  // Convert to whole numbers
  double degrees;
  uint8_t ticks[4];
  uint8_t micro_ticks[4];
  int rounded;
  double rest_degrees;
  uint8_t out_ticks[8];
  for (size_t joint_id = 0; joint_id < 4; ++joint_id)
  {
    std::cout << "joint:" << joint_id << std::endl;
    // Motor positions have a range between -pi/2 and pi/2
    // 180 degrees rotation limit
    // 0.01745329252 radians per degree
    degrees = rest_degrees = joint_position_command_[joint_id] / 0.01745329252;
    std::cout << "degrees " << degrees << std::endl;
    
    // 1,8 degrees per tick
    rounded = floor(degrees / 1.8);
    std::cout << "rounded: " << rounded << std::endl;
    rest_degrees = degrees - (double)(rounded * 1,8);
    // Make it positive
    rounded += 100;
    ticks[joint_id] = (uint8_t)rounded;

    // create micro ticks
    // 32 micros ticks on a real tick
    // 1,8/32 = 0.05625
    micro_ticks[joint_id] = floor(rest_degrees / 0.05625);
    std::cout << "rest degrees: " << rest_degrees << std::endl;

    //Print
    std::cout << (int)ticks[joint_id] << std::endl;
    std::cout << (int)micro_ticks[joint_id] << std::endl;

    //Move to same array
    out_ticks[joint_id*2] = ticks[joint_id];
    out_ticks[joint_id*2+1] = micro_ticks[joint_id];
  }

  // Convert to frame
  Frame *frame = new Frame;
  Header header = *frame = toframe2(header, out_ticks, 8);

  // Send data.
  bcm.startTX(boost::chrono::duration<double>(5000000), header, 1, frame);


  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    joint_position_[joint_id] = joint_position_command_[joint_id];
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void ButlerHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
