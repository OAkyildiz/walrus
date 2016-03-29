#include <ros/ros.h>
#include <ros/spinner.h>

#include "walrus_mainboard_driver/walrus_mainboard_robot.h"

#include "walrus_base_hw/realtime_rate.h"
#include "walrus_base_hw/util.h"

#include <controller_manager/controller_manager.h>

#include <boost/asio.hpp>
#include <rosserial_server/serial_session.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "walrus_main_hw");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  double controller_rate;
  pnh.param<double>("controller_rate", controller_rate, 5);

  std::string port;
  pnh.param<std::string>("port", port, "/dev/walrus_main_board");
  int baud;
  pnh.param("baud", baud, 57600);

  boost::asio::io_service io_service;
  // Destructor is private...
  new rosserial_server::SerialSession(io_service, port, baud);

  boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  walrus_mainboard_driver::MainBoardRobot robot(nh, pnh);

  // Startup ROS spinner in background
  ros::AsyncSpinner spinner(4);
  spinner.start();

controller_manager::ControllerManager cm(&robot, pnh);
  ros::Timer controller_load_timer = walrus_base_hw::createControllerLoadTimer(pnh, &cm);

  walrus_base_hw::RealtimeRate rate(controller_rate);
  while (ros::ok()) {
    ros::Duration dt;
    ros::Time now;
    rate.beginLoop(&now, &dt);

    robot.read(dt);
    cm.update(now, dt);
    robot.write(dt);

    robot.update_diagnostics();

    rate.sleep();
  }
  return 0;
}
