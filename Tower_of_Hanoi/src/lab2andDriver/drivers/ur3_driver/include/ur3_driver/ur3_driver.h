#ifndef UR3_DRIVER_H
#define UR3_DRIVER_H
#include "ros/ros.h"
#include <stdlib.h>

// defined msg by ur3_driver
#include <ur3_driver/command.h>
#include <ur3_driver/position.h>
#include <ur3_driver/ur_communication.h> 
#include <ur3_driver/ur_realtime_communication.h> 
#include <ur3_driver/robot_state.h>
#include <ur3_driver/robot_state_RT.h> 

using std::cout;
using std::endl;
using std::cin;
using std::string;

//class ur3_comm definition
class ur3_comm{
		/** publishers & subscribers
		*/
		ros::NodeHandle	nh;				// ros nh
		ros::Subscriber	ctrl_sub;		//control coming from the student
		ros::Timer 	ur3_timer;
		
		std::string ip_addr_;

		/** callback functions
		 */
		void ctrl_sub_callback(const ur3_driver::command::ConstPtr& msg);
		std::string host;

	public:
		UrRealtimeCommunication* rt_interface_;
		UrCommunication* sec_interface_;
		std::condition_variable rt_msg_cond;
		std::condition_variable msg_cond;		

		ur3_comm();
		~ur3_comm();
};
#endif
