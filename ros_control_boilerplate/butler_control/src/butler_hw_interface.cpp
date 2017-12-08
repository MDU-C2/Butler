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
  //Get values

  //Set values

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
        counter = 0;
        std::cout << "init done" << std::endl;
  //Socketcan listener
/*   g_driver = boost::make_shared<SocketCANInterface>();
  CommInterface::FrameListener::Ptr frame_printer = g_driver->createMsgListener(frame_reader);

  if(!g_driver->init(canport, false)){
  } */

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
// avrunda neråt vid pos
// avrunda uppåt vid neg

void ButlerHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  //reset variables
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

  for (size_t joint_id = 0; joint_id < 4; ++joint_id)
  {
    //std::cout << "joint:" << joint_id << std::endl;
    //std::cout << "radians:" << joint_position_command_[joint_id] << std::endl;
    // Motor positions have a range between -pi/2 and pi/2
    // 180 degrees rotation limit
    // 0.01745329252 radians per degree
    degrees = rest_degrees = joint_position_command_[joint_id] / 0.01745329252;
    //std::cout << "degrees " << degrees << std::endl;
    // 1,8 degrees per tick
    // round down since it is positive
    rounded = floor(degrees / 1.8);
    //std::cout << "rounded ticks: " << rounded << std::endl;
    rest_degrees = degrees - (rounded * 1.8);

    // create micro ticks
    // 32 micros ticks on a real tick
    // 1,8/32 = 0.05625
    if (degrees >= 0) // Positive radians
    {
      //std::cout << "+ angle" << std::endl;
      micro_ticks[joint_id] = floor(rest_degrees / 0.05625);
    }
    else // Negative radians
    {
      //std::cout << "- angle" << std::endl;
      micro_ticks[joint_id] = ceil(rest_degrees / 0.05625);
    }
    
    // Make between 0 - 200 instead of -100 - 100
    rounded += 100;
    ticks[joint_id] = (uint8_t)rounded;

    //std::cout << "rest degrees: " << rest_degrees << std::endl;

    //Print
    //std::cout << (int)ticks[joint_id] << std::endl;
    //std::cout << (int)micro_ticks[joint_id] << std::endl;

//compare and see if we have atleast one whole tick we want to send
    tick_diff=0;
    tick_diff= abs(ticks[joint_id]-current_ticks[joint_id]);

    //std::cout << "total degrees: " << ((ticks[joint_id] * 1.8) + (micro_ticks[joint_id] * 0.05625) - 180) << std::endl << std::endl;

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
//compare and see if we have atleast one whole micro_tick we want to send



    tick_diff=0;
    tick_diff= abs(micro_ticks[joint_id]-current_micro_ticks[joint_id]);
      if (tick_diff>0){
        moved=true;
        /*if (ticks[joint_id]<100){
        	msg_ticks[joint_id]=(32-micro_ticks[joint_id])-(32-current_micro_ticks[joint_id]);
        }
*/

        if (micro_ticks[joint_id]>current_micro_ticks[joint_id]){
        std::cout<<"Moving joint " << joint_id << ". Micro-Ticks: " << (int)tick_diff<<"."<< std::endl;
          current_micro_ticks[joint_id]=current_micro_ticks[joint_id]+tick_diff;
          msg_micro_ticks[joint_id]=tick_diff;
        }else if (micro_ticks[joint_id]<current_micro_ticks[joint_id])
        {
        std::cout<<"Moving joint " << joint_id << ". Micro-Ticks: -" << (int)tick_diff<<"."<< std::endl;
          msg_ticks[joint_id]--;
          msg_micro_ticks[joint_id]=32-tick_diff;
          current_micro_ticks[joint_id]=current_micro_ticks[joint_id]-tick_diff;
        }
        //Move to an array where we gather both ticks and micro ticks
      } 

        //Move to an array where we gather both ticks and micro ticks
        out_ticks[joint_id*2] = msg_ticks[joint_id];
        out_ticks[joint_id*2+1] = msg_micro_ticks[joint_id]; 

  }

if(moved){
  // Convert to frame
  Frame *frame = new Frame;
  std::cout<<"Sending these vals over CAN:         ";
  for (int i=0;i<8;i++)
  {
     std::cout<< (int)out_ticks[i]<<"  ";
  }
  std::cout<<std::endl;
  Header header = *frame = toframe2(header, out_ticks, 8);

  // Send data.
  bcm.startTX(boost::chrono::duration<double>(5000000), header, 1, frame);
  std::cout<<"The expected state of the motors are: ";
  for (int i=0;i<4;i++)
  {
     std::cout<< (int)current_ticks[i]<<"  "<< (int)current_micro_ticks[i]<<"  ";
  }
  std::cout<<std::endl;
  std::cout<<std::endl;
}

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
  //for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  //  joint_position_[joint_id] = joint_position_command_[joint_id];
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
