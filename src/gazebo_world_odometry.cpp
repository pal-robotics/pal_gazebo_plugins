#include <assert.h>
#include <pal_gazebo_plugins/gazebo_world_odometry.h>
#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

namespace gazebo {

  void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
    res[0] = atan2( r31, r32 );
    res[1] = asin ( r21 );
    res[2] = atan2( r11, r12 );
  }

  GazeboWorldOdometry::GazeboWorldOdometry()
    : x_offset_(0.0), y_offset_(0.0), yaw_offset_(0.0), gaussian_noise_(0.0), noise_x_(0.0), noise_y_(0.0), noise_yaw_(0.0)
  {}

  // Destructor
  GazeboWorldOdometry::~GazeboWorldOdometry(){
    this->rosNode_->shutdown();
  }

  // Load the controller
  void GazeboWorldOdometry::Load(physics::ModelPtr parent_model, sdf::ElementPtr _sdf){

    ROS_INFO_STREAM("Loading gazebo WORLD ODOMETRY RPY plugin");

    this->robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    this->topic_name_ = "ft_data";
    if (_sdf->GetElement("topicName"))
      this->topic_name_ =
        _sdf->GetElement("topicName")->Get<std::string>();

    if (!_sdf->HasElement("frameName"))
    {
      ROS_INFO("world odometry sensor plugin missing <frameName>, defaults to world");
      this->frame_name_ = "world";
    }
    else
      this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

    if (!_sdf->HasElement("bodyName"))
    {
      ROS_INFO("world odometry sensor plugin missing <bodyName>, defaults to frameName");
      this->body_name_ = this->frame_name_;
    }
    else
      this->body_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

    this->world_ = parent_model->GetWorld();
    std::string link_name_ = body_name_;
    // assert that the body by link_name_ exists
    this->link = parent_model->GetLink(link_name_);
    if (!this->link)
    {
      ROS_FATAL("gazebo_ros_world_ogometry plugin error: bodyName: %s does not exist\n",
        link_name_.c_str());
    }

    this->update_rate_ = 1000.0;
    if (!_sdf->HasElement("updateRate"))
    {
      ROS_INFO("world odometry plugin missing <updateRate>, defaults to %f", this->update_rate_);
    }
    else
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(ros::NodeHandle("gazebo/" + robot_namespace_ + "world_odometry")));
    ddr_->registerVariable("x_offset", &x_offset_, "", -100., 100.);
    ddr_->registerVariable("y_offset", &y_offset_, "", -100., 100.);
    ddr_->registerVariable("yaw_offset", &yaw_offset_, "", -M_PI, M_PI);
    ddr_->registerVariable("gaussian_noise_offset", &gaussian_noise_, "", 0., 5.);
    ddr_->publishServicesTopics();

    // ros callback queue for processing subscription
    this->deferredLoadThread = boost::thread(
      boost::bind(&GazeboWorldOdometry::DeferredLoad, this));
  }

  ////////////////////////////////////////////////////////////////////////////////
  void GazeboWorldOdometry::DeferredLoad(){

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->rosNode_ = new ros::NodeHandle(this->robot_namespace_);
    floatingBasePub_ = this->rosNode_->advertise<nav_msgs::Odometry>(topic_name_, 100);

    // ros callback queue for processing subscription
    this->callbackQueeuThread = boost::thread(
      boost::bind(&GazeboWorldOdometry::RosQueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
     this->update_connection_ =
     event::Events::ConnectWorldUpdateBegin(
     boost::bind(&GazeboWorldOdometry::UpdateChild, this));
  }

  // Update the controller
  void GazeboWorldOdometry::UpdateChild(){
    if (this->floatingBasePub_.getNumSubscribers() <= 0)
      return;


    common::Time cur_time = this->world_->SimTime();

    if (cur_time < last_time_)
    {
        ROS_WARN_NAMED("p3d", "Negative update time difference detected.");
        last_time_ = cur_time;
    }

    // rate control
    if (this->update_rate_ > 0 &&
        (cur_time-last_time_).Double() < (1.0/this->update_rate_))
      return;


    boost::mutex::scoped_lock sclock(this->mutex_);

    ignition::math::Pose3d pose;
    ignition::math::Quaterniond orientation;
    ignition::math::Vector3d position;

    pose = this->link->WorldPose();
    if (gaussian_noise_ > 0.0)
    {

      std::normal_distribution<double> distribution(0.0, gaussian_noise_);
      noise_x_ += distribution(generator_);
      noise_y_ += distribution(generator_);
      noise_yaw_ += distribution(generator_);
    }
    ignition::math::Vector3d position_offset(noise_x_ + x_offset_, noise_y_ + y_offset_, 0.0);
    ignition::math::Quaterniond orientation_offset(0.0, 0.0, yaw_offset_ + noise_yaw_);
    position = position_offset + pose.Pos();
    orientation = ignition::math::Quaterniond(pose.Rot().Roll(), pose.Rot().Pitch(), yaw_offset_ + pose.Rot().Yaw());


//    ignition::math::Vector3d linearVel = this->link->WorldLinearVel();
//    ignition::math::Vector3d angularVel = this->link->WorldAngularVel();

    ignition::math::Vector3d linearVel = this->link->WorldLinearVel();
    ignition::math::Vector3d angularVel = this->link->RelativeAngularVel();

    nav_msgs::Odometry odomMsg;

    odomMsg.pose.pose.position.x = position.X();
    odomMsg.pose.pose.position.y = position.Y();
    odomMsg.pose.pose.position.z = position.Z();

    odomMsg.pose.pose.orientation.x = orientation.X();
    odomMsg.pose.pose.orientation.y = orientation.Y();
    odomMsg.pose.pose.orientation.z = orientation.Z();
    odomMsg.pose.pose.orientation.w = orientation.W();

    odomMsg.twist.twist.linear.x = linearVel.X();
    odomMsg.twist.twist.linear.y = linearVel.Y();
    odomMsg.twist.twist.linear.z = linearVel.Z();

    odomMsg.twist.twist.angular.x = angularVel.X();
    odomMsg.twist.twist.angular.y = angularVel.Y();
    odomMsg.twist.twist.angular.z = angularVel.Z();

    odomMsg.header.stamp.sec = cur_time.sec;
    odomMsg.header.stamp.nsec = cur_time.nsec;
    odomMsg.header.frame_id = frame_name_;
    odomMsg.child_frame_id = body_name_;

    floatingBasePub_.publish(odomMsg);
    // save last time stamp
    last_time_ = cur_time;
  }


  void GazeboWorldOdometry::RosQueueThread()
  {
  //  static const double timeout = 0.01;
    ros::Rate rate(this->update_rate_);

    while (this->rosNode_->ok())
    {
      this->rosQueue.callAvailable(/*ros::WallDuration(timeout)*/);
      rate.sleep();
    }
  }


  GZ_REGISTER_MODEL_PLUGIN(GazeboWorldOdometry)
}



