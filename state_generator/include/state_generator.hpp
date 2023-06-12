#ifndef __STATE_GENERATOR_HPP__
#define __STATE_GENERATOR_HPP__
#pragma once

// STD header
#include <mutex>
#include <vector>
#include <utility>
#include <memory>

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

// Utility header
#include <util/atom_task.hpp>
#include <util/ini_parser.hpp>


#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

#include <autoku_msgs/PlatformState.h>
#include <autoku_msgs/VehicleState.h>
#include <autoku_msgs/WGS84.h>
#include <autoku_msgs/Gnss.h>

#include <sim_msgs/CarlaEgoVehicleStatus.h>
#include <sim_msgs/CarlaEgoVehicleControl.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

#include <novatel_oem7_msgs/INSPVA.h>
#include <novatel_oem7_msgs/CORRIMU.h>

#include <lanelet2_projection/UTM.h>


// Namespace
using namespace ros; 
using namespace tf;
using namespace std;

lanelet::projection::UtmProjector projection(lanelet::Origin({37.5421000, 127.0770500}));

class StateGenerator : public AtomTask{
                // Constant variables
        const double STEERING_RATIO = 15.4;
        const double RAD2DEG = 57.2958;
        const double DEG2RAD = 0.0175;
        const double KPH2MPS = 0.2778;
        const double MPS2KPH = 3.6;
    public:
        // Constructor
       	explicit StateGenerator(int id, std::string task_node, double period);
        // Operator
        // Destructor
       	virtual ~StateGenerator();

       	void Init();
      	void Run();
        void GetParameter();
      	void Publish();
		void Terminate();
		// void ProcessINI();

        // - - - - - - - - - - - - - - Functions - - - - - - - - - - - - - - //
    private:    
        // Callback functions         
        void CallbackPlatformState(const autoku_msgs::PlatformState::ConstPtr& msg);
        void CallbackSimState(const sim_msgs::CarlaEgoVehicleStatus::ConstPtr& msg);


// freshman project ///////////////////////////////
        void CallbackRCcarState(const std_msgs::Int8::ConstPtr& msg);
        void CallbackNovatel(const novatel_oem7_msgs::INSPVA::ConstPtr& msg);
        void CallbackNovatelAccel(const novatel_oem7_msgs::CORRIMU::ConstPtr& msg);
///////////////////////////////////////////////
        void CallbackKalmanPose(const autoku_msgs::Gnss::ConstPtr& msg);
        void CallbackGnss(const autoku_msgs::Gnss::ConstPtr& msg);
        void CallbackReference(const autoku_msgs::WGS84::ConstPtr& msg);

        void GetVehicleStateRviz();
	

        // - - - - - - - - - - - - - - Variables - - - - - - - - - - - - - - //    
    private:    
    	// Subscriber
        ros::Subscriber sub_novatel_;
        ros::Subscriber sub_novatel_accel;
        ros::Subscriber sub_sim_state_;
        ros::Subscriber sub_vehicle_state_;
        ros::Subscriber sub_reference_;
        ros::Subscriber sub_kalman_pose_;

        // Publisher
        ros::Publisher pub_vehicle_state_;
        ros::Publisher pub_system_time_;



        // Mutex
        mutex mutex_vehicle_state_;

        // Outputs
        autoku_msgs::VehicleState vehicle_state_;
        std_msgs::Float64 o_system_time_;

        std::string mode_;
        double ref_lat_;
        double ref_lon_;
        double ref_height_;
        
        // Environments
        // IniParser v_ini_parser_;
        TransformListener tf_listener_;

        lanelet::GPSPoint current_pos;

};

#endif // __STATE_GENERATOR_HPP__