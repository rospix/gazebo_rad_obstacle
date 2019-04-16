#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <sdf/sdf.hh>
#include <common.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Link.hh>
#include <ignition/math.hh>

#include <geometry_visual_utils/visual_utils.h>

#include <mutex>

#include <gazebo_rad_msgs/RadiationObstacle.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.h>

namespace gazebo
{

  /* class RadiationObstacle //{ */

  class GAZEBO_VISIBLE RadiationObstacle : public ModelPlugin {
  public:
    RadiationObstacle();
    virtual ~RadiationObstacle();

    void QueueThread() {
      static const double timeout = 0.01;
      while (this->rosNode->ok()) {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo&);

  private:
    std::string namespace_;

    physics::EntityPtr modelEntity;

    physics::ModelPtr    model_;
    physics::WorldPtr    world_;
    event::ConnectionPtr updateConnection_;

    transport::NodePtr      node_handle_;
    transport::PublisherPtr obstacle_pub;

    boost::thread callback_queue_thread_;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::CallbackQueue               rosQueue;
    std::thread                      rosQueueThread;

    gazebo_rad_msgs::msgs::RadiationObstacle obstacle_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;

    std::string material;
    double      width, height, depth;
    double      density;
    double      mac60kev, mac600kev;

    Cuboid cuboid;
  };

  GZ_REGISTER_MODEL_PLUGIN(RadiationObstacle)
  RadiationObstacle::RadiationObstacle() : ModelPlugin() {
  }

  RadiationObstacle::~RadiationObstacle() {
    // end all ROS related stuff
    this->rosNode->shutdown();
    // wait for threads to finish
    this->rosQueueThread.join();
    this->callback_queue_thread_.join();
    // end connection to gazebo
    updateConnection_->~Connection();
  }

  //}

  /* Load() //{ */

  void RadiationObstacle::Load(physics::ModelPtr _model, [[maybe_unused]] sdf::ElementPtr _sdf) {

    int    argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_ros_radiation", ros::init_options::NoSigintHandler);

    this->rosNode.reset(new ros::NodeHandle("~"));

    // Store the pointer to the model.
    model_ = _model;

    world_ = model_->GetWorld();

    last_time_     = world_->SimTime();
    last_gps_time_ = world_->SimTime();

    /* parse SDF //{ */
    if (_sdf->HasElement("width")) {
      getSdfParam<double>(_sdf, "width", width, width);
    } else {
      ROS_INFO("[RadiationObstacle]: parameter 'width' was not specified");
    }
    if (_sdf->HasElement("height")) {
      getSdfParam<double>(_sdf, "height", height, height);
    } else {
      ROS_INFO("[RadiationObstacle]: parameter 'height' was not specified");
    }
    if (_sdf->HasElement("depth")) {
      getSdfParam<double>(_sdf, "depth", depth, depth);
    } else {
      ROS_INFO("[RadiationObstacle]: parameter 'depth' was not specified");
    }
    if (_sdf->HasElement("material")) {
      getSdfParam<std::string>(_sdf, "material", material, material);
    } else {
      ROS_INFO("[RadiationObstacle]: parameter 'material' was not specified");
    }
    if (_sdf->HasElement("density")) {
      getSdfParam<double>(_sdf, "density", density, density);
    } else {
      ROS_INFO("[RadiationObstacle]: parameter 'density' was not specified");
    }
    if (_sdf->HasElement("mac60kev")) {
      getSdfParam<double>(_sdf, "mac60kev", mac60kev, mac60kev);
    } else {
      ROS_INFO("[RadiationObstacle]: parameter 'mac60kev' was not specified");
    }
    if (_sdf->HasElement("mac600kev")) {
      getSdfParam<double>(_sdf, "mac600kev", mac600kev, mac600kev);
    } else {
      ROS_INFO("[RadiationObstacle]: parameter 'mac600kev' was not specified");
    }
    //}a

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RadiationObstacle::OnUpdate, this, _1));

    this->modelName = model_->GetName();

    this->obstacle_pub = node_handle_->Advertise<gazebo_rad_msgs::msgs::RadiationObstacle>("~/radiation/obstacles", 1);

    this->rosQueueThread = std::thread(std::bind(&RadiationObstacle::QueueThread, this));

    cuboid = Cuboid(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0), 1.0, 1.0, 1.0);

    ROS_INFO("[RadiationObstacle%u]: initialized", this->model_->GetId());
  }

  //}

  /* OnUpdate() //{ */

  void RadiationObstacle::OnUpdate(const common::UpdateInfo&) {

    auto                   pose  = model_->GetLinks()[0]->WorldPose();

      obstacle_msg.set_pos_x(pose.Pos().X());
      obstacle_msg.set_pos_y(pose.Pos().Y());
      obstacle_msg.set_pos_z(pose.Pos().Z());

      obstacle_msg.set_ori_w(pose.Rot().W());
      obstacle_msg.set_ori_x(pose.Rot().X());
      obstacle_msg.set_ori_y(pose.Rot().Y());
      obstacle_msg.set_ori_z(pose.Rot().Z());

      obstacle_msg.set_scale_x(depth);
      obstacle_msg.set_scale_y(width);
      obstacle_msg.set_scale_z(height);

      obstacle_msg.set_material(material);
      obstacle_msg.set_density(density);
      obstacle_msg.set_mac60kev(mac60kev);
      obstacle_msg.set_mac600kev(mac600kev);
      obstacle_msg.set_id(this->node_handle_->GetId());

      obstacle_pub->Publish(obstacle_msg);

      Eigen::Vector3d    center(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
      Eigen::Quaterniond orientation(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());

      cuboid     = Cuboid(center, orientation, depth, width, height);
  }

  //}

}  // namespace gazebo
