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


using namespace can;

namespace butler_control
{

  BCMsocket bcm;
  Header header;
  boost::shared_ptr<can::ThreadedSocketCANInterface> driver = boost::make_shared<can::ThreadedSocketCANInterface> ();
  can::CommInterface::FrameListener::Ptr frame_listener;

  ButlerHWInterface::ButlerHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)

  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
  {
    ROS_INFO_NAMED("butler_hw_interface", "ButlerHWInterface Ready.");
  }

//Frame Listener
void ButlerHWInterface::frame_reader(const Frame &f)
{
  //Uncomment this code if reading data and using it to update the motor positions

  // Get values
  // Same data structure as write to CAN bus
  //uint8_t output[8];
  //for(int i=0; i < f.dlc; ++i){
  //  output[i] = (uint8_t)f.data[i];
  //}

  // Update position values with return values from the CAN bus
  //for (int joint_id = 0; joint_id < 4; ++joint_id)
  //{
    //joint_position_[joint_id] = ((((int)output[joint_id*2] - 100) * 1.8) + (((int)output[joint_id*2+1]) * 0.05625)) * 0.01745329252;

    //std::cout << "All in one: " << joint_position_[joint_id] << std::endl;
  //}
  std::cout << "Recieved data" << std::endl;
}

void ButlerHWInterface::init()
{
  GenericHWInterface::init();

  // vcan0 is a virtual CAN
  // change to a real port if not a virtual one
  std::string canport = "vcan0";

  // Init bcm for sending data
  bcm.init(canport);

  // Set the CAN header
  std::string head_str = "5#";
  header = toheader(head_str);

  //Socketcan listener
  if(!driver->init(canport, false)){
    std::cout << "ButlerHWInterface init ERROR" << std::endl;
  } 
  // Set method to handle the returned CAN messages
  frame_listener = driver->createMsgListener(CommInterface::FrameDelegate(this, &ButlerHWInterface::frame_reader));

  std::cout << "ButlerHWInterface init done" << std::endl;
}

void ButlerHWInterface::read(ros::Duration &elapsed_time)
{

}

Frame toframe2(Header header, const uint8_t *in_data, size_t in_size) {
  Frame frame(header);
  // Copies data bytes to frame.data
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

// Convert from radians to ticks and microticks
uint8_t * toticks(double radians)
{ 
  uint8_t * ticks = new uint8_t [2];
  // Motor positions have a range between -pi/2 and pi/2
  // 180 degrees rotation limit
  // 0.01745329252 radians per degree
  int degrees = radians / 0.01745329252;

  // 1,8 degrees per tick
  // round down for calculationg microticks
  int rounded = floor(degrees / 1.8);
  int rest_degrees = degrees - (rounded * 1.8);

  // create micro ticks 32 micros ticks on a real tick 1.8/32 = 0.05625
  if (degrees >= 0) // Positive degrees
  {
    ticks[1] = floor(rest_degrees / 0.05625);
  }
  else // Negative degrees
  {
    ticks[1] = ceil(rest_degrees / 0.05625);
  }

  // Make between 0 - 200 instead of -100 - 100
  rounded += 100;
  ticks[0] = (uint8_t)rounded;

  return ticks;
}

// CAN message structure
//  Byte  0   1   2   3   4   5   6   7
//  Motor 1   1   2   2   3   3   4   4
//        T   mT  T   mT  T   mT  T   mT
// T = Ticks
// mT = microTicks

// Sends the joint_position_commands to the arm over CAN
void ButlerHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  // Reset variables
  moved=false;
  for (int i = 0; i<4; i++){
    ticks[i]=100;
    msg_ticks[i]=100;
    micro_ticks[i]=0;
    msg_micro_ticks[i]=0;
    out_ticks[i*2]=100;
  }
  out_ticks[1]=0;
  out_ticks[3]=0;
  out_ticks[5]=0;
  out_ticks[7]=0;

  // Process data
  // Convert to whole numbers
  uint8_t * temp_ticks;
  for (size_t joint_id = 0; joint_id < 4; ++joint_id)
  {
    // Convert radians to ticks and micro ticks
    temp_ticks = toticks(joint_position_command_[joint_id]);
    ticks[joint_id] = temp_ticks[0];
    micro_ticks[joint_id] = temp_ticks[1];

    //std::cout << "joint: " << joint_id << std::endl;
    //std::cout << "total degrees: " << ((ticks[joint_id] * 1.8) + (micro_ticks[joint_id] * 0.05625) - 180) << std::endl << std::endl;

    // Compare and see if we have atleast one whole tick we want to send
    tick_diff=0;
    tick_diff= abs(ticks[joint_id]-current_ticks[joint_id]);

    if (tick_diff>0){
      moved=true;
      if (ticks[joint_id]>current_ticks[joint_id]){          
        std::cout<<"Moving joint " << joint_id << ". Ticks: " << (int)tick_diff<<"."<< std::endl;
        current_ticks[joint_id]=current_ticks[joint_id]+tick_diff;
        msg_ticks[joint_id]=100+tick_diff;
      }else if (ticks[joint_id]<current_ticks[joint_id])
      {
        std::cout<<"Moving joint " << joint_id << ". Ticks: -" << (int)tick_diff<<"."<< std::endl;
        current_ticks[joint_id]=current_ticks[joint_id]-tick_diff;
        msg_ticks[joint_id]=100-tick_diff;
      }
    }

    // Compare and see if we have atleast one whole micro_tick we want to send
    tick_diff=0;
    tick_diff= abs(micro_ticks[joint_id]-current_micro_ticks[joint_id]);
    if (tick_diff>0){
      moved=true;
      if (micro_ticks[joint_id]>current_micro_ticks[joint_id]){
        std::cout<<"Moving joint " << joint_id << ". Micro-Ticks: " << (int)tick_diff<<"."<< std::endl;
        current_micro_ticks[joint_id]=current_micro_ticks[joint_id]+tick_diff;
        msg_micro_ticks[joint_id]=tick_diff;
      }
      else if (micro_ticks[joint_id]<current_micro_ticks[joint_id])
      {
        std::cout<<"Moving joint " << joint_id << ". Micro-Ticks: -" << (int)tick_diff<<"."<< std::endl;
        msg_ticks[joint_id]--;
        msg_micro_ticks[joint_id]=32-tick_diff;
        current_micro_ticks[joint_id]=current_micro_ticks[joint_id]-tick_diff;
      }
    } 
    
    // Move to an array to send to the CAN bus
    // Ordered by motor ex motor 1 ticks and microticks then motor 2 etc.
    out_ticks[joint_id*2] = msg_ticks[joint_id];
    out_ticks[joint_id*2+1] = msg_micro_ticks[joint_id]; 
  }

  if(moved)
  {
    // Convert to frame
    Frame *frame = new Frame;
    std::cout<<"Sending these vals over CAN:         ";
    for (int i=0;i<8;i++)
    {
      std::cout<< (int)out_ticks[i]<<"  ";
    }
    std::cout<<std::endl;
    Header header = *frame = toframe2(header, out_ticks, 8); // Puts the data in out_ticks in a frame

    // Send data
    bcm.startTX(boost::chrono::duration<double>(5000000), header, 1, frame); // Transmits the frame over CAN
    std::cout<<"The expected state of the motors are: ";
    for (int i=0;i<4;i++)
    {
      std::cout<< (int)current_ticks[i]<<"  "<< (int)current_micro_ticks[i]<<"  ";
    }
    std::cout<<std::endl;
    std::cout<<std::endl;
  }
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
