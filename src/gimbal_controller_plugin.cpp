#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo/rendering/rendering.hh>

#include <dynamic_reconfigure/server.h>

using namespace gazebo;

// a plugin to control a virtual gimbal from within gazebo itself
class GimbalControllerPlugin : public ModelPlugin
{
	// empty constructor
	public: GimbalControllerPlugin() {}
	
	// loading the model and installing the plugin
	public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{
		// status message
		std::cerr << "\nThe gimbal controller plugin is attached to model[" << _model->GetName() << "]\n";

		// sanity check for correct model and joints
		if ( _model->GetJointCount() == 0 )
		{
		      std::cerr << "Invalid jount count - Gimbal Controller Plugin not loaded." << std::endl;
		      return;
		}

		// store the model pointer for convenience
		this->model = _model;

		// get pointers to the relevant joints
		this->tilt_joint = _model->GetJoint("iris_demo::iris_demo::gimbal_small_2d::tilt_joint");
		this->base_joint = _model->GetJoint("iris_demo::iris_demo::iris_gimbal_mount");

		//      this->tilt_joint->SetPosition(0, 0, true);
		//      this->base_joint->SetPosition(1, -1.5708, true);

		// set up PID controllers
		// parameters in order: p, i, d, imax, imin, cmdMax, cmdMin
		// tilt k_d started at 0.3
		this->tilt_pid = common::PID(3, 0, 0.9, 2, 0, 1.5708, -1.5708);
		this->base_pid = common::PID(0.8, 0, 0, 0.3, 0, 1.5708, -1.5708);

		// apply the PID controllers to the joint
		this->model->GetJointController()->SetPositionPID( this->tilt_joint->GetScopedName(), this->tilt_pid );
		this->model->GetJointController()->SetPositionPID( this->base_joint->GetScopedName(), this->base_pid );

		// set the joint's initial target position
		this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), 0.0 );
		this->model->GetJointController()->SetPositionTarget( this->base_joint->GetScopedName(), 0.0 );

		// subscribed topic names
		const std::string x_setpoint_topic_name = "/iris/camera/x/setpoint";
		const std::string y_setpoint_topic_name = "/iris/camera/y/setpoint";
		const std::string idle_state_topic_name = "/iris/camera/idle_state";
		const std::string x_k_p_topic_name      = "/gimbal_controller/x/k_p";
		const std::string x_k_i_topic_name      = "/gimbal_controller/x/k_i";
		const std::string x_k_d_topic_name      = "/gimbal_controller/x/k_d";
		const std::string y_k_p_topic_name      = "/gimbal_controller/y/k_p";
		const std::string y_k_i_topic_name      = "/gimbal_controller/y/k_i";
		const std::string y_k_d_topic_name      = "/gimbal_controller/y/k_d";	
		const std::string landing_pad_relative_position_topic_name	= "/landing_pad/relative_position";

		// sanity check for ROS
		if ( ! ros::isInitialized() )
		{
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client",
			ros::init_options::NoSigintHandler);
		}

		// initialize ROS node
		this->ros_node.reset(new ros::NodeHandle("gazebo_client"));
		
		// initialize x setpoint subscriber
		ros::SubscribeOptions so_x = ros::SubscribeOptions::create<std_msgs::Float64>(
			x_setpoint_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_base, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->ros_subscriber_x = this->ros_node->subscribe(so_x);

		// initialize y setpoint subscriber
		ros::SubscribeOptions so_y = ros::SubscribeOptions::create<std_msgs::Float64>(
			y_setpoint_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_tilt, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->ros_subscriber_y = this->ros_node->subscribe(so_y);

		// initialize idle state status subscriber
		ros::SubscribeOptions so_idle = ros::SubscribeOptions::create<std_msgs::Bool>(
			idle_state_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_idle, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->idle_state_subscriber = this->ros_node->subscribe(so_idle);

		// initialize ROS message queue
		this->ros_queue_thread = std::thread(std::bind(&GimbalControllerPlugin::queue_thread, this));

		// initialize x_k_p subscriber
		ros::SubscribeOptions so_x_k_p = ros::SubscribeOptions::create<std_msgs::Float64>(
			x_k_p_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_x_k_p, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->x_k_p_subscriber = this->ros_node->subscribe(so_x_k_p);
		// initialize x_k_i subscriber
		ros::SubscribeOptions so_x_k_i = ros::SubscribeOptions::create<std_msgs::Float64>(
			x_k_i_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_x_k_i, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->x_k_i_subscriber = this->ros_node->subscribe(so_x_k_i);
		// initialize x_k_d subscriber
		ros::SubscribeOptions so_x_k_d = ros::SubscribeOptions::create<std_msgs::Float64>(
			x_k_d_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_x_k_d, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->x_k_d_subscriber = this->ros_node->subscribe(so_x_k_d);

		// initialize y_k_p subscriber
		ros::SubscribeOptions so_y_k_p = ros::SubscribeOptions::create<std_msgs::Float64>(
			y_k_p_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_y_k_p, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->y_k_p_subscriber = this->ros_node->subscribe(so_y_k_p);
		// initialize y_k_i subscriber
		ros::SubscribeOptions so_y_k_i = ros::SubscribeOptions::create<std_msgs::Float64>(
			y_k_i_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_y_k_i, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->y_k_i_subscriber = this->ros_node->subscribe(so_y_k_i);
		// initialize y_k_d subscriber
		ros::SubscribeOptions so_y_k_d = ros::SubscribeOptions::create<std_msgs::Float64>(
			y_k_d_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_y_k_d, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->y_k_d_subscriber = this->ros_node->subscribe(so_y_k_d);
/*		
		// initial attempts
		ignition::math::Vector3d pose;
		pose = this->base_joint->RelativePose();
		std::cerr << *pose << std::endl;
*/
		std::cerr << this->base_joint->GetChild()->RelativePose() << std::endl;
		std::cerr << this->tilt_joint->GetChild()->WorldPose() << std::endl;

	}

	private: std::unique_ptr<ros::NodeHandle> ros_node;
	private: ros::Subscriber ros_subscriber_x;
	private: ros::Subscriber ros_subscriber_y;
	private: ros::Subscriber x_k_p_subscriber;
	private: ros::Subscriber x_k_i_subscriber;
	private: ros::Subscriber x_k_d_subscriber;
	private: ros::Subscriber y_k_p_subscriber;
	private: ros::Subscriber y_k_i_subscriber;
	private: ros::Subscriber y_k_d_subscriber;
	private: ros::Subscriber idle_state_subscriber;
	private: ros::Subscriber landing_pad_relative_position_subscriber;
	private: ros::CallbackQueue ros_queue;
	private: std::thread ros_queue_thread;
	private: ros::Publisher pose_publisher;
	private: physics::ModelPtr model;
	private: physics::JointPtr tilt_joint; // camera pitch control
	private: physics::JointPtr base_joint; // camera yaw control
	private: common::PID tilt_pid;
	private: common::PID base_pid;
	private: transport::NodePtr node_handle;

	private: void camera_relative_pose()
	{
		;
	}

	private: double x_idle_setpoint;
	private: double y_idle_setpoint;
	private: double p_x;
	private: double i_x;
	private: double d_x;
	private: double p_y;
	private: double i_y;
	private: double d_y; 
	
	private: void OnRosMsg(const std_msgs::Float64ConstPtr & _msg)
	{
		std::cerr << "received: " << _msg->data << std::endl;
	}

	private: void queue_thread()
	{
		static const double timeout = 0.05;
		while(this->ros_node->ok())
		{
			this->ros_queue.callAvailable(ros::WallDuration(timeout));
		}
	}

	private: void set_tilt(const std_msgs::Float64ConstPtr & _msg)
	{
	    this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), this->tilt_joint->Position(0) + _msg->data );
	}

	private: void set_base(const std_msgs::Float64ConstPtr & _msg)
	{
		this->model->GetJointController()->SetPositionTarget( this->base_joint->GetScopedName(), this->base_joint->Position(0) + _msg->data );
	}

	private: void set_idle(const std_msgs::BoolConstPtr & _msg)
	{
		if (_msg->data)
		{
		    this->model->GetJointController()->SetPositionTarget( this->base_joint->GetScopedName(), 0 );
		    this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), 0 );
		}
	}

	private: void set_x_k_p(const std_msgs::Float64ConstPtr & _msg)
	{
		this->base_pid.SetPGain( _msg->data );
		for(int i = 0; i < 100; i ++)
		{
			ROS_INFO("received %f", _msg->data);
			std::cerr << _msg->data << std::endl;
		}
	}
	private: void set_x_k_i(const std_msgs::Float64ConstPtr & _msg)
	{
		this->base_pid.SetIGain( _msg->data );
	}
	private: void set_x_k_d(const std_msgs::Float64ConstPtr & _msg)
	{
		this->base_pid.SetDGain( _msg->data );
	}
	private: void set_y_k_p(const std_msgs::Float64ConstPtr & _msg)
	{
		this->tilt_pid.SetPGain( _msg->data );
	}
	private: void set_y_k_i(const std_msgs::Float64ConstPtr & _msg)
	{
		this->tilt_pid.SetIGain( _msg->data );
	}
	private: void set_y_k_d(const std_msgs::Float64ConstPtr & _msg)
	{
		this->tilt_pid.SetDGain( _msg->data );
	}

	private: void OnMsg(ConstVector3dPtr &_msg)
	{
		this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), _msg->x() );
		this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), _msg->y() );
		std::cerr << "set point <" << _msg->x() << ", " << _msg->y() << ">" << std::endl;
	}
};

// register the plugin with gazebo so that it can be called
GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)
