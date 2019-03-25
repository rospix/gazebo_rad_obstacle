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
#include <gazebo/physics/BoxShape.hh>
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
    ros::Publisher                   visualizer_pub;

    gazebo_rad_msgs::msgs::RadiationObstacle obstacle_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;

    std::string material_;
    double      width, height, depth;
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

    if (_sdf->HasElement("material")) {
      getSdfParam<std::string>(_sdf, "material", material_, material_);
    } else {
      ROS_INFO("[RadiationObstacle]: parameter 'material' was not specified");
    }

    if (_sdf->HasElement("width")) {
      getSdfParam<double>(_sdf, "width", width, width);
    }
    if (_sdf->HasElement("height")) {
      getSdfParam<double>(_sdf, "height", height, height);
    }
    if (_sdf->HasElement("depth")) {
      getSdfParam<double>(_sdf, "depth", depth, depth);
    }

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&RadiationObstacle::OnUpdate, this, _1));

    this->modelName = model_->GetName();

    this->obstacle_pub = node_handle_->Advertise<gazebo_rad_msgs::msgs::RadiationObstacle>("~/radiation/obstacles", 1);

    this->rosQueueThread = std::thread(std::bind(&RadiationObstacle::QueueThread, this));

    this->visualizer_pub = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/obstacle", 1);

    ROS_INFO("[RadiationObstacle%u]: initialized", this->model_->GetId());
  }

  //}

  /* OnUpdate() //{ */

  void RadiationObstacle::OnUpdate(const common::UpdateInfo&) {

    common::Time current_time = world_->SimTime();

    ignition::math::Pose3d T_W_I = model_->WorldPose();
    auto                   link  = model_->GetLinks()[0];

    obstacle_msg.set_pos_x(link->WorldPose().Pos().X());
    obstacle_msg.set_pos_y(link->WorldPose().Pos().Y());
    obstacle_msg.set_pos_z(link->WorldPose().Pos().Z());

    obstacle_msg.set_ori_w(link->WorldPose().Rot().W());
    obstacle_msg.set_ori_x(link->WorldPose().Rot().X());
    obstacle_msg.set_ori_y(link->WorldPose().Rot().Y());
    obstacle_msg.set_ori_z(link->WorldPose().Rot().Z());

    obstacle_msg.set_scale_x(depth);
    obstacle_msg.set_scale_y(width);
    obstacle_msg.set_scale_z(height);

    obstacle_msg.set_material(material_);
    obstacle_msg.set_id(this->node_handle_->GetId());

    obstacle_pub->Publish(obstacle_msg);

    Eigen::Vector3d p1, p2, p3, p4;
    Rectangle       rect;
    p1[0] = link->WorldPose().Pos().X() + depth / 2;
    p1[1] = link->WorldPose().Pos().Y() + width / 2;
    p1[2] = link->WorldPose().Pos().Z() + height / 2;

    p2[0] = link->WorldPose().Pos().X() + depth / 2;
    p2[1] = link->WorldPose().Pos().Y() + width / 2;
    p2[2] = link->WorldPose().Pos().Z() - height / 2;

    p3[0] = link->WorldPose().Pos().X() + depth / 2;
    p3[1] = link->WorldPose().Pos().Y() - width / 2;
    p3[2] = link->WorldPose().Pos().Z() - height / 2;

    p4[0] = link->WorldPose().Pos().X() + depth / 2;
    p4[1] = link->WorldPose().Pos().Y() - width / 2;
    p4[2] = link->WorldPose().Pos().Z() + height / 2;

    rect.points.push_back(p1);
    rect.points.push_back(p2);
    rect.points.push_back(p3);
    rect.points.push_back(p4);


    VisualTools::visualizeRect(visualizer_pub, rect, "/base_link", 0.0, 1.0, 0.7);
  }

    //}

}  // namespace gazebo
