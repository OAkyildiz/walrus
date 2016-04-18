#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <walrus_drive_controller/TankDriveCommandStamped.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <walrus_drive_controller/odometry.h>
#include <walrus_drive_controller/speed_limiter.h>

namespace walrus_drive_controller{

  const double DRIVE_SENSITIVITY=3.0;
  const double M_TO_INCH=39.37;
  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
class WalrusDriveController
  : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
 public:
  WalrusDriveController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);

  private:
    std::string name_;

    /// Odometry related:
  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;
  double command_timeout_;
    bool open_loop_;

  double main_tread_separation_;
  double main_tread_ground_contact_length_;
  double tread_width_;
  double tread_driver_radius_;


    /// Wheel separation and radius calibration multipliers:
    double tread_separation_multiplier_;
    double tread_radius_multiplier_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// Frame to use for odometry and odom tf:
    std::string odom_frame_id_;

    /// Number of wheel joints:
    size_t tread_joints_size_;

  hardware_interface::JointHandle left_tread_joint_;
  hardware_interface::JointHandle right_tread_joint_;
  
      /// 2nd degree Speed limiters for jerk control (Old odom data)):

  Odometry odometry_;
  Odometry odometry_old_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

  realtime_tools::RealtimeBuffer<geometry_msgs::TwistStamped> cmd_vel_buffer_;
  realtime_tools::RealtimeBuffer<walrus_drive_controller::TankDriveCommandStamped> tank_drive_buffer_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber tank_drive_sub_;



  // Speed limiters:
  geometry_msgs::Twist last_cmd_vel_;
  SpeedLimiter limiter_lin_;
  SpeedLimiter limiter_ang_;

 private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
  void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);
    /**
     * \brief tank drive command callback
     * \param command TankDrive command message (twist)
     */
    void tankCallback(const walrus_drive_controller::TankDriveCommand& command);

    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

};

PLUGINLIB_EXPORT_CLASS(walrus_drive_controller::WalrusDriveController, controller_interface::ControllerBase);
} // namespace walrus_drive_controller
