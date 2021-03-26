//see ur3_driver.h in include folder.
#include "ur3_driver/ur3_driver.h" 

//#include <actionlib/client/simple_client_goal_state.h>
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include <mutex>
#include <condition_variable>

#include <ur3_driver/do_output.h>
#include <vector>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <chrono>

/*	ur3_comm class function: CONSTRUCTOR *************************************
	the constructor initializes publishers, subscribers,
	the joint_names for the trajectories message 
	and the velocity vector for each trajectory points.
******************************************************************************/

ur3_comm::ur3_comm(){
	//initialize all subscribers and publishers
	ctrl_sub=nh.subscribe("ur3/command",100,&ur3_comm::ctrl_sub_callback,this); 

	if (!(ros::param::get("~robot_ip", host))) {
		
		print_fatal("Could not get robot ip. Please supply it by parameter server as robot_ip");
		exit(1);

	}
	
	ROS_INFO("Creating UrRealtimeCommunication");
	
	rt_interface_ = new UrRealtimeCommunication(rt_msg_cond,host,nh,300);
	sleep(1);
	sec_interface_ = new UrCommunication(msg_cond,host,nh);

	sleep(1);


	if (!rt_interface_->start()) {
		ROS_INFO("Error Starting rt interface");
	} else {
		ip_addr_ = rt_interface_->getLocalIp();
		std::cout << "RT Local PC addr " << ip_addr_ << "\n";
	}

	sleep(1);

	if (!sec_interface_->start()) {
		ROS_INFO("Error Starting secondary interface");
	}	
	
}

/*	ur3_comm class function: DESTRUCTOR **************************************
	delete the client, release the memory.

******************************************************************************/
ur3_comm::~ur3_comm(){
	//delete client;
}


/*	ur3_comm class function: ctrl_sub_callback *******************************
	Callback function for the subscriber that subscribe to "ur3/command"
	topic. This function translates the msg sent to "ur3/command" by the 
	student to msg that can be used by the Action client.
******************************************************************************/
void ur3_comm::ctrl_sub_callback(const ur3_driver::command::ConstPtr& msg)
{
	std::string sendCMD;		
	char buf[256];
	float velocity = 0;
	float acceleration = 0;
	// check the student sent a vector of size 6.
	if(msg->destination.size() !=6){
		ROS_INFO("WARNNING: In ctrl_sub_callback- received command size is not 6");
		return;
	}

	if(msg->io_0 == true ) {
		sprintf(buf,"set_digital_out(0,True)\n");
		sendCMD = buf;
	} else {
		sprintf(buf,"set_digital_out(0,False)\n");
		sendCMD = buf;
	}
	std::cout << sendCMD;
	rt_interface_->addCommandToQueue(sendCMD);

	velocity = msg->v;
	acceleration = msg->a;
	if (acceleration < 0.1) {
		ROS_INFO("Acceleration too low setting to 0.1 rad/s^2");
		acceleration = 0.1;
	}
	if (acceleration > 4.0) {
		ROS_INFO("Acceleration too high setting to 4.0 rad/s^2");
		acceleration = 4.0;
	}

	if (velocity < 0.1) {
		ROS_INFO("Velocity too low setting to 0.1 rad/s");
		velocity = 0.1;
	}
	if (velocity > 4.0) {
		ROS_INFO("Velocity too high setting to 4.0 rad/s");
		velocity = 4.0;
	}

	usleep(80000);
	sendCMD = "movej([";
	sprintf(buf,"%.6f,%.6f,%.6f,%.6f,%.6f,%.6f],a=%.6f,v=%.6f)\n",msg->destination[0],
	msg->destination[1],msg->destination[2],msg->destination[3],msg->destination[4],msg->destination[5],
	acceleration,velocity);
	sendCMD += buf;

	std::cout << sendCMD;


	rt_interface_->addCommandToQueue(sendCMD);

	ROS_INFO("Sending Goal..");
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur3_driver");
	ur3_comm ur3_start;


	ros::MultiThreadedSpinner spinner(2);		//Multi-Threaded spinner that uses two threads
	spinner.spin();					// This enables state_sub_callback to publish isRdy variable
	return 0;					// while ctrl_sub_callback is being processed.
}

