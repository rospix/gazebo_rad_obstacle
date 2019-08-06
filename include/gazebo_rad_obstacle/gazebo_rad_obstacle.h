#ifndef GAZEBO_RAD_OBSTACLE_H
#define GAZEBO_RAD_OBSTACLE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_rad_msgs/RadiationObstacle.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.h>

namespace gazebo
{
class GAZEBO_VISIBLE Obstacle : public ModelPlugin {
public:
  Obstacle();
  virtual ~Obstacle();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void EarlyUpdate(const common::UpdateInfo &upd);

private:
  bool          terminated;
  boost::thread publisher_thread;
  void          PublisherLoop();

  std::string                   material;
  double                        publish_rate;
  double                        scale_x, scale_y, scale_z;
  std::chrono::duration<double> sleep_seconds;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;
  event::ConnectionPtr    updateConnection_;

  std::unique_ptr<ros::NodeHandle> ros_node;
  ros::Publisher                   ros_publisher;
  ros::Subscriber                  change_activity_sub;
  ros::Subscriber                  change_material_sub;
};

GZ_REGISTER_MODEL_PLUGIN(Obstacle)
Obstacle::Obstacle() : ModelPlugin() {
}
}  // namespace gazebo

#endif
