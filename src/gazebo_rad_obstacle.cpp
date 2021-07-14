/* includes //{ */

#include <sdf/sdf.hh>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_rad_msgs/Termination.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//}

namespace gazebo
{

/* class Obstacle //{ */

class GAZEBO_VISIBLE Obstacle : public ModelPlugin {
public:
  Obstacle();
  virtual ~Obstacle();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  Eigen::Vector3d    position;
  Eigen::Vector3d    size;
  Eigen::Quaterniond orientation;

  bool          terminated;
  boost::thread publisher_thread;
  void          PublisherLoop();

  std::string                   material;
  double                        publish_rate;
  std::chrono::duration<double> sleep_seconds;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;
  transport::PublisherPtr termination_publisher_;
  event::ConnectionPtr    updateConnection_;

  ros::NodeHandle ros_nh_;
  ros::Publisher  ros_publisher;
};

//}

// | -------------------- Plugin interface -------------------- |

/* Load() //{ */

void Obstacle::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  // init local variables
  model_ = _model;

  position    = Eigen::Vector3d(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());
  orientation = Eigen::Quaterniond(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z());

  /* parse sdf params //{ */
  if (_sdf->HasElement("material")) {
    material = _sdf->Get<std::string>("material");
  } else {
    ROS_WARN("[RadiationObstacle%u]: parameter 'material' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("publish_rate")) {
    publish_rate  = _sdf->Get<double>("publish_rate");
    sleep_seconds = std::chrono::duration<double>(1 / publish_rate);
  } else {
    ROS_WARN("[RadiationObstacle%u]: parameter 'publish_rate' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("size_x")) {
    size[0] = _sdf->Get<double>("size_x");
  } else {
    ROS_WARN("[RadiationObstacle%u]: parameter 'size_x' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("size_y")) {
    size[1] = _sdf->Get<double>("size_y");
  } else {
    ROS_WARN("[RadiationObstacle%u]: parameter 'size_y' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("size_z")) {
    size[2] = _sdf->Get<double>("size_z");
  } else {
    ROS_WARN("[RadiationObstacle%u]: parameter 'size_z' was not specified", model_->GetId());
  }
  //}

  // init gazebo node
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_rad_obstacle", ros::init_options::NoSigintHandler);
  ros_nh_ = ros::NodeHandle("~");

  // gazebo communication
  this->gazebo_publisher_      = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::RadiationObstacle>("/radiation/obstacles", 1);
  this->termination_publisher_ = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::Termination>("/radiation/termination", 1);

  // ros communication
  ros_publisher = ros_nh_.advertise<gazebo_rad_msgs::RadiationObstacle>("/radiation/obstacles", 1);

  terminated       = false;
  publisher_thread = boost::thread(boost::bind(&Obstacle::PublisherLoop, this));
  ROS_INFO("[RadiationObstacle%u]: Plugin initialized", model_->GetId());
}

//}

/* Obstacle() //{ */

Obstacle::Obstacle() {
}

//}

/* ~Obstacle() //{ */

Obstacle::~Obstacle() {

  // inform other gazebo nodes
  gazebo_rad_msgs::msgs::Termination msg;
  msg.set_id(model_->GetId());
  termination_publisher_->Publish(msg);

  // terminate
  terminated = true;
  publisher_thread.join();
  ROS_INFO("[RadiationObstacle%u]: Plugin terminated", model_->GetId());
}

//}

// | --------------------- Custom routines -------------------- |

/* PublisherLoop() //{ */

void Obstacle::PublisherLoop() {

  while (!terminated) {

    /* Gazebo message //{ */

    gazebo_rad_msgs::msgs::RadiationObstacle msg;
    msg.set_pos_x(model_->WorldPose().Pos().X());
    msg.set_pos_y(model_->WorldPose().Pos().Y());
    msg.set_pos_z(model_->WorldPose().Pos().Z() + (size[2] / 2.0));

    msg.set_ori_x(model_->WorldPose().Rot().X());
    msg.set_ori_y(model_->WorldPose().Rot().Y());
    msg.set_ori_z(model_->WorldPose().Rot().Z());
    msg.set_ori_w(model_->WorldPose().Rot().W());

    msg.set_size_x(size[0]);
    msg.set_size_y(size[1]);
    msg.set_size_z(size[2]);
    msg.set_collision_type(2);
    msg.set_id(model_->GetId());
    msg.set_material(material);
    gazebo_publisher_->Publish(msg);

    //}

    /* ROS message (debug) //{ */

    gazebo_rad_msgs::RadiationObstacle debug_msg;
    debug_msg.material           = material;
    debug_msg.id                 = model_->GetId();
    debug_msg.pose.position.x    = model_->WorldPose().Pos().X();
    debug_msg.pose.position.y    = model_->WorldPose().Pos().Y();
    debug_msg.pose.position.z    = model_->WorldPose().Pos().Z() + (size[2] / 2.0);
    debug_msg.pose.orientation.x = model_->WorldPose().Rot().X();
    debug_msg.pose.orientation.y = model_->WorldPose().Rot().Y();
    debug_msg.pose.orientation.z = model_->WorldPose().Rot().Z();
    debug_msg.pose.orientation.w = model_->WorldPose().Rot().W();
    debug_msg.size.x             = size[0];
    debug_msg.size.y             = size[1];
    debug_msg.size.z             = size[2];
    debug_msg.stamp              = ros::Time::now();
    ros_publisher.publish(debug_msg);

    //}

    std::this_thread::sleep_for(sleep_seconds);
  }
}

//}

}  // namespace gazebo

GZ_REGISTER_MODEL_PLUGIN(gazebo::Obstacle)
