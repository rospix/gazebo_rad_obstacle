#include <gazebo_rad_obstacle/gazebo_rad_obstacle.h>

using namespace gazebo;

/* Destructor //{ */
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

/* Load //{ */
void Obstacle::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  // init local variables
  model_       = _model;
  param_change = true;

  position     = Eigen::Vector3d(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());
  orientation  = Eigen::Quaterniond(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z());

  // parse sdf params
  if (_sdf->HasElement("material")) {
    material = _sdf->Get<std::string>("material");
  } else {
    ROS_WARN("[RadiationObstacle%u]: parameter 'material' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("publish_rate")) {
    publish_rate  = _sdf->Get<double>("publish_rate");
    sleep_seconds = std::chrono::duration<double>(1 / publish_rate);
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'publish_rate' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("publish_rate")) {
    publish_rate = _sdf->Get<double>("publish_rate");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'publish_rate' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("scale_x")) {
    scale_x = _sdf->Get<double>("scale_x");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'scale_x' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("scale_y")) {
    scale_y = _sdf->Get<double>("scale_y");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'scale_y' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("scale_z")) {
    scale_z = _sdf->Get<double>("scale_z");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'scale_z' was not specified", model_->GetId());
  }

  // init gazebo node
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_rad_source", ros::init_options::NoSigintHandler);
  ros_node.reset(new ros::NodeHandle("~"));

  // gazebo communication
  this->gazebo_publisher_      = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::RadiationObstacle>("~/radiation/obstacles", 1);
  this->termination_publisher_ = gazebo_node_->Advertise<gazebo_rad_msgs::msgs::Termination>("~/radiation/termination", 1);

  // ros communication
  ros_publisher = ros_node->advertise<gazebo_rad_msgs::RadiationObstacle>("/radiation/obstacles", 1);

  terminated       = false;
  publisher_thread = boost::thread(boost::bind(&Obstacle::PublisherLoop, this));
  ROS_INFO("[RadiationObstacle%u]: Plugin initialized", model_->GetId());
}
//}

/* PublisherLoop //{ */
void Obstacle::PublisherLoop() {
  while (!terminated) {

    /* Eigen::Vector3d    new_position(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z()); */
    /* Eigen::Quaterniond new_orientation(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), */
    /*                                    model_->WorldPose().Rot().Z()); */

    /* if (new_position == position && new_orientation.coeffs() == orientation.coeffs() && !param_change) { */
    /*   std::this_thread::sleep_for(sleep_seconds); */
    /*   continue; */
    /* } */

    /* Gazebo message //{ */
    gazebo_rad_msgs::msgs::RadiationObstacle msg;
    msg.set_pos_x(model_->WorldPose().Pos().X());
    msg.set_pos_y(model_->WorldPose().Pos().Y());
    msg.set_pos_z(model_->WorldPose().Pos().Z());
    msg.set_ori_x(model_->WorldPose().Rot().X());
    msg.set_ori_y(model_->WorldPose().Rot().Y());
    msg.set_ori_z(model_->WorldPose().Rot().Z());
    msg.set_ori_w(model_->WorldPose().Rot().W());
    msg.set_scale_x(scale_x);
    msg.set_scale_y(scale_y);
    msg.set_scale_z(scale_z);
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
    debug_msg.pose.position.z    = model_->WorldPose().Pos().Z();
    debug_msg.pose.orientation.x = model_->WorldPose().Rot().X();
    debug_msg.pose.orientation.y = model_->WorldPose().Rot().Y();
    debug_msg.pose.orientation.z = model_->WorldPose().Rot().Z();
    debug_msg.pose.orientation.w = model_->WorldPose().Rot().W();
    debug_msg.size.x             = scale_x;
    debug_msg.size.y             = scale_y;
    debug_msg.size.z             = scale_z;
    debug_msg.stamp              = ros::Time::now();
    ros_publisher.publish(debug_msg);
    //}

    std::this_thread::sleep_for(sleep_seconds);
    /* position     = new_position; */
    /* orientation  = new_orientation; */
    /* param_change = false; */
  }
}
//}
