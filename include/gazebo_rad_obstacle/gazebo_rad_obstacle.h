#ifndef GAZEBO_RAD_OBSTACLE_H
#define GAZEBO_RAD_OBSTACLE_H

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

namespace gazebo
{
class GAZEBO_VISIBLE Obstacle : public ModelPlugin {
public:
  Obstacle();
  virtual ~Obstacle();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  Eigen::Vector3d position;
  Eigen::Vector3d size;
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

  std::unique_ptr<ros::NodeHandle> ros_node;
  ros::Publisher                   ros_publisher;
};

GZ_REGISTER_MODEL_PLUGIN(Obstacle)
Obstacle::Obstacle() : ModelPlugin() {
}
}  // namespace gazebo

#endif
