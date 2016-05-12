#include <assert.h>
#include <pal_gazebo_plugins/gazebo_world_odometry.h>
#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/JointState.h>

namespace gazebo {

  void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
    res[0] = atan2( r31, r32 );
    res[1] = asin ( r21 );
    res[2] = atan2( r11, r12 );
  }

  GazeboWorldOdometry::GazeboWorldOdometry(){}

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
      ROS_INFO("ft sensor plugin missing <frameName>, defaults to world");
      this->frame_name_ = "world";
    }
    else
      this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

    this->world_ = parent_model->GetWorld();
    std::string link_name_ = "base_link";
    // assert that the body by link_name_ exists
    this->link = boost::dynamic_pointer_cast<gazebo::physics::Link>(
      this->world_->GetEntity(link_name_));
    if (!this->link)
    {
      ROS_FATAL("gazebo_ros_imu plugin error: bodyName: %s does not exist\n",
        link_name_.c_str());
    }

    this->update_rate_ = 1000.0;
    if (!_sdf->HasElement("updateRate"))
    {
      ROS_INFO("world odometry plugin missing <updateRate>, defaults to %f", this->update_rate_);
    }
    else
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

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
    floatingBasePub_ = this->rosNode_->advertise<sensor_msgs::JointState>(topic_name_, 100);

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

    boost::mutex::scoped_lock sclock(this->mutex_);

    gazebo::math::Pose pose;
    gazebo::math::Quaternion q;
    gazebo::math::Vector3 pos;

    pose = this->link->GetWorldPose();
    pos = pose.pos;
    q = pose.rot;

    gazebo::math::Vector3 gazebo_rpy =  pose.rot.GetAsEuler();

    gazebo::math::Vector3 vpos = this->link->GetWorldLinearVel();
    gazebo::math::Vector3 veul = this->link->GetWorldAngularVel();
    double vrpy[3];
    vrpy[0] = veul[2];
    vrpy[1] = veul[1];
    vrpy[2] = veul[0];

    double res[3];
    threeaxisrot(2.*(q.x*q.y + q.w*q.z),
                 q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,
                 -2*(q.x*q.z - q.w*q.y),
                 2*(q.y*q.z + q.w*q.x),
                 q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z,
                 res);

    double rpy[3];
    rpy[0] = res[2];
    rpy[1] = res[1];
    rpy[2] = res[0];

    sensor_msgs::JointState floatingJoinstaStates;
    floatingJoinstaStates.position.resize(6);
    floatingJoinstaStates.velocity.resize(6);
    for(size_t i=0; i<3; ++i){
      floatingJoinstaStates.position[i] = pos[i];
      floatingJoinstaStates.velocity[i] = vpos[i];
    }
    for(size_t i=0; i<3; ++i){
//      floatingJoinstaStates.position[i + 3] = rpy[i];
      floatingJoinstaStates.position[i + 3] = gazebo_rpy[i];
//      floatingJoinstaStates.velocity[i + 3] = vrpy[i];
      floatingJoinstaStates.velocity[i + 3] = veul[i];

    }
    floatingBasePub_.publish(floatingJoinstaStates);
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



