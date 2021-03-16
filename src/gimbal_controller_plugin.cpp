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
#include <mavros_msgs/OverrideRCIn.h>

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

		// get joint names
		std::cerr << "Attempting to get tilt_joint..." << std::endl;
		this->tilt_joint_name = _sdf->Get<std::string>("tilt_joint");
		std::cerr << "\tname: " << this->tilt_joint_name << std::endl;
		std::cerr << "Attempting to get pan_joint..." << std::endl;
		this->pan_joint_name  = _sdf->Get<std::string>("pan_joint");
		std::cerr << "\tname: " << this->pan_joint_name << std::endl;

		std::cerr << "Getting tilt pid parameters..." << std::endl;
		auto tilt_pid_parameters = _sdf->GetElement("tilt_pid");
		this->tilt_pid_p = tilt_pid_parameters->Get<double>("p");
		this->tilt_pid_i = tilt_pid_parameters->Get<double>("i");
		this->tilt_pid_d = tilt_pid_parameters->Get<double>("d");
		
		std::cerr << "Getting pan pid parameters..." << std::endl;
		auto  pan_pid_parameters = _sdf->GetElement("pan_pid");
		this->pan_pid_p = pan_pid_parameters->Get<double>("p");
		this->pan_pid_i = pan_pid_parameters->Get<double>("i");
		this->pan_pid_d = pan_pid_parameters->Get<double>("d");

		// store the model pointer for convenience
		this->model = _model;

	        std::cerr << "Attempting to find relevant joints..." << std::endl;	
		try
		{
			// get pointers to the relevant joints
//			this->tilt_joint = _model->GetJoint("iris_demo::gimbal_small_2d::tilt_joint");
//			this->pan_joint  = _model->GetJoint("iris_demo::iris_gimbal_mount");
			this->tilt_joint = _model->GetJoint(this->tilt_joint_name);
			this->pan_joint  = _model->GetJoint(this->pan_joint_name);
		
			std::cerr << "Gimbal Controller Plugin is operating on the following joints:" << std::endl;
			std::cerr << "\tTilt joint: " << this->tilt_joint << " ---> " << this->tilt_joint->GetScopedName() << std::endl;
			std::cerr << "\t Pan joint: " << this->pan_joint->GetScopedName() << std::endl;
		}
		catch( std::exception& e )
		{
			std::cerr << e.what() << std::endl;

			// exit if we can't get the relevant joints
			exit(-1);
		}

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

		// initialize ROS message queue
		this->ros_queue_thread = std::thread(std::bind(&GimbalControllerPlugin::queue_thread, this));

		// set up PID controllers
		// parameters in order:        p,   i,   d,     imax,imin,   cmdMax,  cmdMin
		this->tilt_pid = common::PID( this->tilt_pid_p, this->tilt_pid_i, this->tilt_pid_d, 0, 0.0, M_PI, -M_PI);
		this->pan_pid  = common::PID(  this->pan_pid_p, this->pan_pid_i,  this->pan_pid_d, 0, 0.0, M_PI, -M_PI);

		std::cerr << "Using the following PID parameters:" << std::endl;
		std::cerr << "\tTilt PID parameters (p, i, d) = (" << this->tilt_pid_p << ", " << this->tilt_pid_i << ", " << this->tilt_pid_d << ")" << std::endl;
		std::cerr << "\t Pan PID parameters (p, i, d) = (" << this->pan_pid_p << ", " << this->pan_pid_i << ", " << this->pan_pid_d << ")" << std::endl;

		// apply the PID controllers to the joint
		this->model->GetJointController()->SetPositionPID( this->tilt_joint->GetScopedName(), this->tilt_pid );
		this->model->GetJointController()->SetPositionPID(  this->pan_joint->GetScopedName(), this->pan_pid  );

		// set initial gimbal positions		
#if DEBUG
		// notify
		std::cerr << "Setting initial gimbal positions:" << std::endl << "\tTilt: " << this->initial_tilt << std::endl << "\tPan:  " << this->initial_pan << std::endl;
#endif

		// subscribed topic names
		try
		{
			this->rc_override_topic = _sdf->Get<std::string>("rc_override_topic");
			this->tilt_channel = _sdf->Get<int>("tilt_channel");
			this->pan_channel = _sdf->Get<int>("pan_channel");

			std::cerr << "Got rc_override_topic '" << this->rc_override_topic << "' with tilt_channel=" << this->tilt_channel << " and pan_channel=" << this->pan_channel << std::endl;

			// initialize x pixel position subscriber
			ros::SubscribeOptions so_mavros_rc_override = ros::SubscribeOptions::create<mavros_msgs::OverrideRCIn>(
				rc_override_topic,
				1,
				boost::bind(&GimbalControllerPlugin::set_control_callback, this, _1),
				ros::VoidPtr(),
				& this->ros_queue);
			this->ros_subscriber_x_pixel_position = this->ros_node->subscribe(so_mavros_rc_override);
		}
		catch( ... )
		{
			std::cerr << "Did not find rc_override_topic!" << std::endl;
			this->target_pan_topic = _sdf->Get<std::string>("target_pan_topic");
			this->target_tilt_topic = _sdf->Get<std::string>("target_tilt_topic");

			std::cerr << "Subscribing to topics:" << std::endl;
			std::cerr << "\tTarget tilt: " << this->target_tilt_topic << std::endl;

			// initialize y pixel position subscriber
			ros::SubscribeOptions so_y_pixel_position = ros::SubscribeOptions::create<std_msgs::Float64>(
				target_tilt_topic,
				1,
				boost::bind(&GimbalControllerPlugin::set_tilt_callback, this, _1),
				ros::VoidPtr(),
				& this->ros_queue);
			this->ros_subscriber_y_pixel_position = this->ros_node->subscribe(so_y_pixel_position);
		}

		set_pan( 0 );
		set_tilt( -1 );
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

	private: std::string tilt_joint_name;
	private: std::string pan_joint_name;
	private: std::string rc_override_topic;
	private: int tilt_channel;
	private: int pan_channel;
	private: std::string target_pan_topic;
	private: std::string target_tilt_topic;

	private: double tilt_pid_p = 0;
	private: double tilt_pid_i = 0;
	private: double tilt_pid_d = 0;
	private: double  pan_pid_p = 0;
	private: double  pan_pid_i = 0;
	private: double  pan_pid_d = 0;

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

	private: double pwm_signal_to_control_effort( int pwm )
	{
		double result = (pwm - (double)1500) / (double)500;
		return result;
	}

	private: void set_control_callback(const mavros_msgs::OverrideRCIn::ConstPtr & _msg)
	{
		int tilt_pwm = _msg->channels[this->tilt_channel - 1];

		double tilt_effort = pwm_signal_to_control_effort(tilt_pwm);

		set_tilt( M_PI * tilt_effort );
		set_pan(0);
	}
};

// register the plugin with gazebo so that it can be called
GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)
