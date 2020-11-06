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

#include <math.h>

#define DEBUG 0

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
 
		try
		{
			// get pointers to the relevant joints
			this->tilt_joint = _model->GetJoint("iris_demo::gimbal_small_2d::tilt_joint");
			this->pan_joint  = _model->GetJoint("iris_demo::iris_gimbal_mount");
		
			std::cerr << "Gimbal Controller Plugin is operating on the following joints:" << std::endl;
			std::cerr << "\tTilt joint: " << this->tilt_joint->GetScopedName() << std::endl;
			std::cerr << "\t Pan joint: " << this->pan_joint->GetScopedName() << std::endl;
		}
		catch( std::exception& e )
		{
			std::cerr << e.what() << std::endl;

			// exit if we can't get the relevant joints
			exit(-1);
		}

		// set up PID controllers
		// parameters in order:        p,   i,   d,     imax,imin,   cmdMax,  cmdMin
//		this->tilt_pid = common::PID( 0.5, 0.1, 0.01,  15.0, 0.0, 0.174533, -M_PI_2);
//		this->pan_pid = common::PID( 0.5, 0.1, 0.05,   15.0, 0.0, 0.174533, -M_PI_2);
		this->tilt_pid = common::PID( 0.5, 0.1, 0.05, 0, 0.0, M_PI, -M_PI);
		this->pan_pid  = common::PID( 0.5, 0.1, 0.05, 0, 0.0, M_PI, -M_PI);

		// apply the PID controllers to the joint
		this->model->GetJointController()->SetPositionPID( this->tilt_joint->GetScopedName(), this->tilt_pid );
		this->model->GetJointController()->SetPositionPID(  this->pan_joint->GetScopedName(), this->pan_pid  );

		// set initial gimbal positions		
#if DEBUG
		// notify
		std::cerr << "Setting initial gimbal positions:" << std::endl << "\tTilt: " << this->initial_tilt << std::endl << "\tPan:  " << this->initial_pan << std::endl;
#endif
		set_pan( 0 );
		set_tilt(0);

		// subscribed topic names
		const std::string x_control_effort_topic_name = "/pid/camera/control_effort/x";
		const std::string y_control_effort_topic_name = "/pid/camera/control_effort/y";

		// sanity check if ROS is running
		if ( ! ros::isInitialized() )
		{
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client",
			ros::init_options::NoSigintHandler);
		}

		// initialize ROS node
		this->ros_node.reset(new ros::NodeHandle("gazebo_client"));
		
		// initialize x pixel position subscriber
		ros::SubscribeOptions so_x_pixel_position = ros::SubscribeOptions::create<std_msgs::Float64>(
			x_control_effort_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_pan_callback, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->ros_subscriber_x_pixel_position = this->ros_node->subscribe(so_x_pixel_position);
		
		// initialize y pixel position subscriber
		ros::SubscribeOptions so_y_pixel_position = ros::SubscribeOptions::create<std_msgs::Float64>(
			y_control_effort_topic_name,
			1,
			boost::bind(&GimbalControllerPlugin::set_tilt_callback, this, _1),
			ros::VoidPtr(),
			& this->ros_queue);
		this->ros_subscriber_y_pixel_position = this->ros_node->subscribe(so_y_pixel_position);
	
		// initialize ROS message queue
		this->ros_queue_thread = std::thread(std::bind(&GimbalControllerPlugin::queue_thread, this));
	}

	private: std::unique_ptr<ros::NodeHandle> ros_node;
	private: ros::Subscriber ros_subscriber_x_pixel_position;
	private: ros::Subscriber ros_subscriber_y_pixel_position;
	private: ros::CallbackQueue ros_queue;
	private: std::thread ros_queue_thread;
	private: physics::ModelPtr model;
	private: physics::JointPtr tilt_joint; // camera pitch control
	private: physics::JointPtr pan_joint; // camera yaw control
	private: common::PID tilt_pid;
	private: common::PID pan_pid;
	private: transport::NodePtr node_handle;

	private: double initial_tilt = 0.2;
	private: double initial_pan  = 0.0;

	private: void queue_thread()
	{
		static const double timeout = 0.05;
		while(this->ros_node->ok())
		{
			this->ros_queue.callAvailable(ros::WallDuration(timeout));
		}
	}

	// uses negative orientation because the gimbal is mounted inverted in the model
	private: void set_tilt(double target)
	{
		target -= initial_tilt;

		this->model->GetJointController()->SetPositionTarget( this->tilt_joint->GetScopedName(), target );
#if DEBUG
		std::cerr << "Tilt target: " << target << ", " << "tilt position: " << this->tilt_joint->Position(0) << std::endl;
#endif
	}
	 
	private: void set_pan(double target)
	{
		target += initial_pan;

		this->model->GetJointController()->SetPositionTarget( this->pan_joint->GetScopedName(), target );
#if DEBUG		
		std::cerr << "Pan target: " << target << ", " << "pan position: " << this->pan_joint->Position(0) << std::endl;
#endif
	}

	private: void set_tilt_callback(const std_msgs::Float64ConstPtr & _msg)
	{
		set_tilt(M_PI * _msg->data);
	}

	private: void set_pan_callback(const std_msgs::Float64ConstPtr & _msg)
	{
		set_pan(M_PI * _msg->data);
	}
};

// register the plugin with gazebo so that it can be called
GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)
