#ifndef _GIMBAL_CONTROLLER_PLUGIN_HH_
#define _GIMBAL_CONTROLLER_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <std_msgs/Float64.h>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo
{
  /// \brief A plugin to control a gimbal.
  class GimbalControllerPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: GimbalControllerPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "\nThe gimbal controller plugin is attached to model[" <<
        _model->GetName() << "]\n";

      // Safety check
      if (_model->GetJointCount() == 0)
      {
	      std::cerr << "Invalid jount count - Gimbal Controller Plugin not loaded." << std::endl;
	      return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
//      this->joint = _model->GetJoint("iris_demo::iris_demo::camera_joint");
      this->tilt_joint = _model->GetJoint("iris_demo::iris_demo::gimbal_small_2d::tilt_joint");
      this->base_joint = _model->GetJoint("iris_demo::iris_demo::iris_gimbal_mount");

      // Set up PID controllers.
      this->tilt_pid = common::PID(50, 500, 0);
      this->base_pid = common::PID(50, 500, 0);

      // Apply the PID controllers to the joint.
      this->model->GetJointController()->SetPositionPID(
		this->tilt_joint->GetScopedName(), this->tilt_pid
		);
      this->model->GetJointController()->SetPositionPID(
		this->base_joint->GetScopedName(), this->base_pid
		);

      // Set the joint's initial target position.
      this->model->GetJointController()->SetPositionTarget(
		this->tilt_joint->GetScopedName(), 0.0
		);
      this->model->GetJointController()->SetPositionTarget(
		this->base_joint->GetScopedName(), 0.0
		);

      const std::string x_setpoint_topic_name = "/iris/camera/x/setpoint";
      const std::string y_setpoint_topic_name = "/iris/camera/y/setpoint";
      const std::string xy_setpoint_topic_name = "/iris/camera/xy/setpoint";

if (!ros::isInitialized())
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
}

this->ros_node.reset(new ros::NodeHandle("gazebo_client"));

ros::SubscribeOptions so_x = ros::SubscribeOptions::create<std_msgs::Float64>(
	x_setpoint_topic_name,
	1,
	boost::bind(&GimbalControllerPlugin::set_base, this, _1),
	ros::VoidPtr(),
	&this->ros_queue);

this->ros_subscriber_x = this->ros_node->subscribe(so_x);

ros::SubscribeOptions so_y = ros::SubscribeOptions::create<std_msgs::Float64>(
	y_setpoint_topic_name,
	1,
	boost::bind(&GimbalControllerPlugin::set_tilt, this, _1),
	ros::VoidPtr(),
	&this->ros_queue);
this->ros_subscriber_y = this->ros_node->subscribe(so_y);

this->ros_queue_thread = std::thread(std::bind(&GimbalControllerPlugin::queue_thread, this));

    }

    private: std::unique_ptr<ros::NodeHandle> ros_node;
    private: ros::Subscriber ros_subscriber_x;
    private: ros::Subscriber ros_subscriber_y;
    private: ros::CallbackQueue ros_queue;
    private: std::thread ros_queue_thread;

private: void OnRosMsg(const std_msgs::Float64ConstPtr & _msg)
{
	std::cerr << "received: " << _msg->data << std::endl;
}

private: void queue_thread()
{
	static const double timeout = 0.01;
	while(this->ros_node->ok())
	{
		this->ros_queue.callAvailable(ros::WallDuration(timeout));
	}
}

    private: physics::ModelPtr model;
    private: physics::JointPtr tilt_joint; // camera pitch control
    private: physics::JointPtr base_joint; // camera yaw control
    private: common::PID tilt_pid;
    private: common::PID base_pid;
    private: transport::NodePtr node_handle;
    private: transport::SubscriberPtr setpoint_subscriber_x;
    private: transport::SubscriberPtr setpoint_subscriber_y;
    private: transport::SubscriberPtr setpoint_subscriber_xy;

	     //		   const msgs::ConstFloat64Ptr

    private: void set_tilt(const std_msgs::Float64ConstPtr & _msg)
    {
	    this->model->GetJointController()->SetPositionTarget(
		this->tilt_joint->GetScopedName(), _msg->data
	    );
    }

    private: void set_base(const std_msgs::Float64ConstPtr & _msg)
    {
	    this->model->GetJointController()->SetPositionTarget(
		this->base_joint->GetScopedName(), _msg->data
	    );
    }

    private: void OnMsg(ConstVector3dPtr &_msg)
    {
	    this->model->GetJointController()->SetPositionTarget(
		this->tilt_joint->GetScopedName(), _msg->x()
	    );
	    this->model->GetJointController()->SetPositionTarget(
		this->tilt_joint->GetScopedName(), _msg->y()
	    );
	    std::cerr << "set point <" << _msg->x() << ", " << _msg->y() << ">" << std::endl;
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)
}
#endif
