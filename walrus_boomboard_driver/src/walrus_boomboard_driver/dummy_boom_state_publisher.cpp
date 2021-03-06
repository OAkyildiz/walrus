#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "walrus_base_hw/realtime_rate.h"

#include <sstream>
#include <vector>

double DEPLOY;
double PAN;
double TILT;
double PI=3.15149;

int main(int argc, char **argv)
{
 

  ros::init(argc, argv, "dummy_boom_state");
  ros::NodeHandle dh;
  
  dh.param("deploy", DEPLOY, 0.0);
  dh.param("pan", PAN, 0.0);
  dh.param("tilt", TILT, 0.0);
  
  ros::Publisher dummy_state_pub = dh.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  walrus_base_hw::RealtimeRate rate(10);
  //create msg
  sensor_msgs::JointState dummy_state;
  //resize fields
  /*
  dummy_state.name.resize(3);
  dummy_state.position.resize(3);
  dummy_state.velocity.resize(3);
  dummy_state.effort.resize(3);
  */
  dummy_state.name.push_back("walrus/boom/deploy_joint");
  dummy_state.name.push_back("walrus/boom/pan_joint");
  dummy_state.name.push_back("walrus/boom/tilt_joint");
  
  for(int i=0; i<3; i++){
    //dummy_state.position.push_back(0.0);
    dummy_state.velocity.push_back(0.0);
    dummy_state.effort.push_back(0.0) ;
  }

  dummy_state.position.push_back(DEPLOY);
  dummy_state.position.push_back(PAN);
  dummy_state.position.push_back(TILT);

  // Startup ROS spinner in background
  ros::AsyncSpinner spinner(1);
  spinner.start();

 

  while (ros::ok())
  {
  	ros::Duration dt;
    ros::Time now;
    rate.beginLoop(&now, &dt);
    /**
     * This is a message object.
     */
   
 
	
    ROS_INFO("Publishing fake boom position");
	
    dummy_state.header.stamp = now;
    dummy_state_pub.publish(dummy_state);


	ros::spinOnce();
    rate.sleep();

  }


  return 0;
}
