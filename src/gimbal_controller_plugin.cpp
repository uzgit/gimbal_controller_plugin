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

#include <gazebo_plugins/PubQueue.h>

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

		// set up PID controllers
		// parameters in order:        p,   i,   d,imax,imin, cmdMax,  cmdMin
		//this->tilt_pid = common::PID( 0.2, 0.1, 0.005, 15.0, 0.0, 1.5708, -1.5708);
		//this->base_pid = common::PID( 0.2, 0.1, 0.005, 15.0, 0.0, 1.5708, -1.5708);
		this->tilt_pid = common::PID( 0.5, 0.1, 0.05, 15.0, 0.0, 1.5708, -1.5708);
		this->base_pid = common::PID( 0.5, 0.1, 0.05, 15.0, 0.0, 1.5708, -1.5708);

		// apply the PID controllers to the joint
		this->model->GetJointController()->SetPositionPID( this->tilt_joint->GetScopedName(), this->tilt_pid );
		this->model->GetJointController()->SetPositionPID( this->base_joint->GetScopedName(), this->base_pid );

		// set the joint's initial target position
		this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), 0.0 );
		this->model->GetJointController()->SetPositionTarget( this->base_joint->GetScopedName(), 0.0 );

		// subscribed topic names
		const std::string x_setpoint_topic_name = "/gimbal/x/setpoint";
		const std::string y_setpoint_topic_name = "/gimbal/y/setpoint";
		const std::string x_kp_topic_name = "/gimbal/x/kp";
		const std::string x_ki_topic_name = "/gimbal/x/ki";
		const std::string x_kd_topic_name = "/gimbal/x/kd";
		const std::string y_kp_topic_name = "/gimbal/y/kp";
		const std::string y_ki_topic_name = "/gimbal/y/ki";
		const std::string y_kd_topic_name = "/gimbal/y/kd";
		const std::string idle_state_topic_name = "/gimbal/idle_state";

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
//		this->idle_state_subscriber = this->ros_node->subscribe(so_idle);

		// initialize kp subscriber for x dimension
		ros::SubscribeOptions so_x_kp = ros::SubscribeOptions::create<std_msgs::Float64>(
			x_kp_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_x_kp, this, _1),
			ros::VoidPtr(),
			& this->ros_queue
			);
		this->ros_subscriber_x_kp = this->ros_node->subscribe(so_x_kp);

		// initialize ki subscriber for x dimension
		ros::SubscribeOptions so_x_ki = ros::SubscribeOptions::create<std_msgs::Float64>(
			x_ki_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_x_ki, this, _1),
			ros::VoidPtr(),
			& this->ros_queue
			);
		this->ros_subscriber_x_ki = this->ros_node->subscribe(so_x_ki);

		// initialize kd subscriber for x dimension
		ros::SubscribeOptions so_x_kd = ros::SubscribeOptions::create<std_msgs::Float64>(
			x_kd_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_x_kd, this, _1),
			ros::VoidPtr(),
			& this->ros_queue
			);
		this->ros_subscriber_x_kd = this->ros_node->subscribe(so_x_kd);
		
		// initialize kp subscriber for y dimension
		ros::SubscribeOptions so_y_kp = ros::SubscribeOptions::create<std_msgs::Float64>(
			y_kp_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_y_kp, this, _1),
			ros::VoidPtr(),
			& this->ros_queue
			);
		this->ros_subscriber_y_kp = this->ros_node->subscribe(so_y_kp);

		// initialize ki subscriber for y dimension
		ros::SubscribeOptions so_y_ki = ros::SubscribeOptions::create<std_msgs::Float64>(
			y_ki_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_y_ki, this, _1),
			ros::VoidPtr(),
			& this->ros_queue
			);
		this->ros_subscriber_y_ki = this->ros_node->subscribe(so_y_ki);

		// initialize kd subscriber for y dimension
		ros::SubscribeOptions so_y_kd = ros::SubscribeOptions::create<std_msgs::Float64>(
			y_kd_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_y_kd, this, _1),
			ros::VoidPtr(),
			& this->ros_queue
			);
		this->ros_subscriber_y_kd = this->ros_node->subscribe(so_y_kd);

		// initialize ROS message queue
		this->ros_queue_thread = std::thread(std::bind(&GimbalControllerPlugin::queue_thread, this));

		this->publisher_multi_queue.startServiceThread();
		this->publisher_queue = this->publisher_multi_queue.addPub<std_msgs::Float64>();
		this->base_joint_position_publisher = this->ros_node->advertise<std_msgs::Float64>("/gimbal/x/position", 100);
		this->tilt_joint_position_publisher = this->ros_node->advertise<std_msgs::Float64>("/gimbal/y/position", 100);
	}

	private: std::unique_ptr<ros::NodeHandle> ros_node;
	private: ros::Subscriber ros_subscriber_x;
	private: ros::Subscriber ros_subscriber_y;
	private: ros::Subscriber ros_subscriber_x_kp;
	private: ros::Subscriber ros_subscriber_x_ki;
	private: ros::Subscriber ros_subscriber_x_kd;
	private: ros::Subscriber ros_subscriber_y_kp;
	private: ros::Subscriber ros_subscriber_y_ki;
	private: ros::Subscriber ros_subscriber_y_kd;
	private: ros::Subscriber idle_state_subscriber;
	private: ros::Subscriber landing_pad_relative_position_subscriber;
	private: ros::CallbackQueue ros_queue;
	private: std::thread ros_queue_thread;
	private: physics::ModelPtr model;
	private: physics::JointPtr tilt_joint; // camera pitch control
	private: physics::JointPtr base_joint; // camera yaw control
	private: common::PID tilt_pid;
	private: common::PID base_pid;
	private: transport::NodePtr node_handle;
	private: transport::PublisherPtr orientation_publisher;

	private: double x_idle_setpoint;
	private: double y_idle_setpoint;

	private: ros::Publisher base_joint_position_publisher;
	private: ros::Publisher tilt_joint_position_publisher;
	private: PubQueue<std_msgs::Float64>::Ptr publisher_queue;
	private: PubMultiQueue publisher_multi_queue;

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
		this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), _msg->data );

		std_msgs::Float64 msg;
		msg.data = this->tilt_joint->Position(0);
		this->publisher_queue->push(msg, this->tilt_joint_position_publisher);
	}

	private: void set_base(const std_msgs::Float64ConstPtr & _msg)
	{
		this->model->GetJointController()->SetPositionTarget( this->base_joint->GetScopedName(), _msg->data );

		std_msgs::Float64 msg;
		msg.data = this->base_joint->Position(0);
		this->publisher_queue->push(msg, this->base_joint_position_publisher);
	}
	
	private: void set_x_kp(const std_msgs::Float64ConstPtr & _msg)
	{
		std::cerr << "setting x_kp to " << _msg->data << std::endl;
		this->base_pid.SetPGain( _msg->data );
	}
	private: void set_x_ki(const std_msgs::Float64ConstPtr & _msg)
	{
		this->base_pid.SetIGain( _msg->data );
	}
	private: void set_x_kd(const std_msgs::Float64ConstPtr & _msg)
	{
		this->base_pid.SetDGain( _msg->data );
	}
	
	private: void set_y_kp(const std_msgs::Float64ConstPtr & _msg)
	{
		this->tilt_pid.SetPGain( _msg->data );
	}
	private: void set_y_ki(const std_msgs::Float64ConstPtr & _msg)
	{
		this->tilt_pid.SetIGain( _msg->data );
	}
	private: void set_y_kd(const std_msgs::Float64ConstPtr & _msg)
	{
		this->tilt_pid.SetDGain( _msg->data );
	}
	
	private: void set_idle(const std_msgs::BoolConstPtr & _msg)
	{
		if (_msg->data)
		{
		    this->model->GetJointController()->SetPositionTarget( this->base_joint->GetScopedName(), 0 );
		    this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), 0 );
		}
	}
};

// register the plugin with gazebo so that it can be called
GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)
