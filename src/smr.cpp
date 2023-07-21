#include <ros/ros.h>
#include "rosneuro_smr_processing/Smr.hpp"

int main(int argc, char** argv) {

	
	// ros initialization
	ros::init(argc, argv, "smr");

	rosneuro::processing::Smr smr;
	
	if(smr.configure() == false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ROS_INFO("[INFO] Configuration done");
	
	smr.run();

	ros::shutdown();
	return 0;
}
