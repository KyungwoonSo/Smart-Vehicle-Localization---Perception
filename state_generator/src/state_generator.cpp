#include "state_generator.hpp"

StateGenerator::StateGenerator(int id, std::string task_node, double period)
    :AtomTask(id, task_node, period) {}

StateGenerator::~StateGenerator(){}

void StateGenerator::Init(){
    // Node initialization
    NodeHandle nh;
    GetParameter();
    
    // Subscriber init
    if (mode_ == "sim"){
        sub_sim_state_ = nh.subscribe("/sim/vehicle_state", 10, &StateGenerator::CallbackSimState, this);
        // ROS_WARN("MODE : SIM!");
    }
    else{
        // original
        // sub_vehicle_state_ = nh.subscribe("/communication/vehicle_state", 10, &StateGenerator::CallbackPlatformState, this);
        // sub_reference_ = nh.subscribe("/reference",10, &StateGenerator::CallbackReference,this);
        // sub_gnss_ = nh.subscribe("/gnss", 10, &StateGenerator::CallbackGnss, this);

        // freshman project
        ROS_WARN("MODE : REAL!");
        // sub_vehicle_state_ = nh.subscribe("/arduino_operation_mode", 10, &StateGenerator::CallbackRCcarState, this);
        // sub_reference_ = nh.subscribe("/reference",10, &StateGenerator::CallbackReference,this);
        // sub_novatel_ = nh.subscribe("/novatel/oem7/inspva", 10, &StateGenerator::CallbackNovatel, this);
        // sub_novatel_accel = nh.subscribe("/novatel/oem7/corrimu", 10, &StateGenerator::CallbackNovatelAccel, this);
        sub_kalman_pose_ = nh.subscribe("/kalman_pose", 10, &StateGenerator::CallbackKalmanPose, this); 
    }

    // Publisher init
    pub_vehicle_state_ = nh.advertise<autoku_msgs::VehicleState>("/vehicle_state", 10); 
    pub_system_time_ = nh.advertise<std_msgs::Float64>("sys/state_generator", 1);
    

	// ProcessINI();
    ROS_INFO("INIT Complete");                
}
void StateGenerator::GetParameter(){
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    nhPrivate.getParam("mode",mode_);
    
    // if (!nh.getParam("mode", mode_)) mode_ = "";
    ROS_INFO("mode: %s", mode_.c_str());

    // Parse parameters
    // nhPrivate.getParam("p_strInputMotion",p_strInputMotion);


    // ROS_INFO("p_ConfigPoseEstimationEkfFile : %s", p_ConfigPoseEstimationEkfFile.c_str());
}

void StateGenerator::Run(){
    o_system_time_.data = ros::Time::now().toSec();
    ROS_WARN("State Generator. mode: %s", mode_.c_str());
}

void StateGenerator::Publish(){
    pub_vehicle_state_.publish(vehicle_state_);
    pub_system_time_.publish(o_system_time_);
}

void StateGenerator::Terminate(){}

// Callback functions


void StateGenerator::CallbackKalmanPose(const autoku_msgs::Gnss::ConstPtr& msg) {

    //주의 사항 state generator.hpp파일에 map의 projection point origin 변경할것!!
    mutex_vehicle_state_.lock();
    vehicle_state_.header.frame_id = msg->header.frame_id;
    vehicle_state_.header.stamp = msg->header.stamp;

    current_pos.lat = msg->latitude;
    current_pos.lon = msg->longitude;

    vehicle_state_.x = projection.forward(current_pos).x();
    vehicle_state_.y = projection.forward(current_pos).y();
    
    double azimuth_heading = msg->heading;  
    if (azimuth_heading > M_PI){
        azimuth_heading -= 2.0 * M_PI;
    }
    else if (azimuth_heading < -M_PI){
        azimuth_heading += 2.0 * M_PI;
    }

    vehicle_state_.yaw = azimuth_heading; 
    
    mutex_vehicle_state_.unlock();
}



void StateGenerator::CallbackPlatformState(const autoku_msgs::PlatformState::ConstPtr& msg) {
    mutex_vehicle_state_.lock();

    vehicle_state_.steering_angle = msg->steer;
    vehicle_state_.speed = msg->speed_ms;

    if (msg->gear == 2){
        vehicle_state_.gear = vehicle_state_.DRIVE;
    }
    else if(msg->gear == 1){
        vehicle_state_.gear = vehicle_state_.NEUTRAL;
    }
    else{vehicle_state_.gear = vehicle_state_.REVERSE;}

    if(msg->speed_ms >= 0.0){
        vehicle_state_.direction = vehicle_state_.FORWARD;
    }
    else{vehicle_state_.direction = vehicle_state_.BACKWARD;}

    if(msg->automode == true){
        vehicle_state_.operation_mode = vehicle_state_.AUTONOMOUS;
    }
    else{vehicle_state_.operation_mode = vehicle_state_.MANUAL;}

    mutex_vehicle_state_.unlock();
}

void StateGenerator::CallbackRCcarState(const std_msgs::Int8::ConstPtr& msg) { //fix msg type 
    mutex_vehicle_state_.lock();

    vehicle_state_.gear = vehicle_state_.DRIVE;
    
    if(msg->data == true){
        vehicle_state_.operation_mode = vehicle_state_.AUTONOMOUS;
    }
    else{vehicle_state_.operation_mode = vehicle_state_.MANUAL;}

    mutex_vehicle_state_.unlock();
}

void StateGenerator::CallbackNovatel(const novatel_oem7_msgs::INSPVA::ConstPtr& msg) {
    mutex_vehicle_state_.lock();

    double ref_latitude_rad = ref_lat_ * M_PI / 180.;

    double m1 = 111132.92;
    double m2 = -559.82;
    double m3 = 1.175;
    double m4 = -0.0023;

    double p1 = 111412.84;
    double p2 = -93.5;
    double p3 = 0.118;

    double lenoflat = m1 + (m2 * cos(2 * ref_latitude_rad))
    + (m3 * cos(4 * ref_latitude_rad))
    + (m4 * cos(6 * ref_latitude_rad));
    
    double lenoflon = p1 * cos(ref_latitude_rad)
    + (p2 * cos(3 * ref_latitude_rad))
    + (p3 * cos(5 * ref_latitude_rad));


    vehicle_state_.x = (msg->longitude - ref_lon_)*lenoflon;
    vehicle_state_.y = (msg->latitude  - ref_lat_)*lenoflat;
    vehicle_state_.z = 0.0;//msg->altitude;
   
    
    double azimuth_heading = (-(msg -> azimuth) + 90.0) * M_PI/180.0;
    if (azimuth_heading > M_PI){
        azimuth_heading -= 2.0 * M_PI;
    }
    else if (azimuth_heading < -M_PI){
        azimuth_heading += 2.0 * M_PI;
    }
    vehicle_state_.yaw = azimuth_heading; 

    // vehicle_state_.yaw = msg->azimuth * M_PI/180.0; 

    // vehicle_state_.x_sigma = (msg->longitude_stdev);
    // vehicle_state_.y_sigma = (msg->latitude_stdev);
    // vehicle_state_.z_sigma = (msg->height_stdev);
    // vehicle_state_.yaw_sigma = (msg->azimuth_stdev);

        

    vehicle_state_.speed = sqrt(pow(msg->north_velocity, 2) + pow(msg->east_velocity, 2));

    vehicle_state_.driving_mode = vehicle_state_.AUTONOMOUS;
    vehicle_state_.direction = vehicle_state_.FORWARD;
  
    // auto projector = projection::UtmProjector(Origin({ref_lat_, ref_lon_}));
    // lanelet::BasicPoint3d point;
    // point.x() = point_x;
    // point.y() = point_y;
    // lanelet::GPSPoint gps_point = projector.reverse(point);


    // autoku_msgs::WGS84 latlon_msg;
    // latlon_msg.latitude = gps_point.lat;
    // latlon_msg.longitude = gps_point.lon;
    // latlon_msg.height = 0.0;

    mutex_vehicle_state_.unlock();
}

void StateGenerator::CallbackNovatelAccel(const novatel_oem7_msgs::CORRIMU::ConstPtr& msg) {
    vehicle_state_.acceleration = sqrt(pow(msg->lateral_acc, 2) + pow(msg->longitudinal_acc, 2));
}


void StateGenerator::CallbackSimState(const sim_msgs::CarlaEgoVehicleStatus::ConstPtr& msg) {
    mutex_vehicle_state_.lock();
    
    vehicle_state_.header.frame_id = msg->header.frame_id;
    vehicle_state_.header.stamp = msg->header.stamp;
    vehicle_state_.x = msg->position.x;
    vehicle_state_.y = msg->position.y;
    // m_estimated_vehicle_state.z = msg->position.z;
    vehicle_state_.z = 0.0;
    tf::Quaternion q(msg->orientation.x,
                    msg->orientation.y,
                    msg->orientation.z,
                    msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    vehicle_state_.yaw = yaw;
    vehicle_state_.yaw_rate = msg->yaw_rate;
    vehicle_state_.speed = (msg->velocity)/3.6;
    // m_estimated_vehicle_state.acceleration = msg->acceleration.linear.x;
    vehicle_state_.steering_angle = -msg->control.steer * RAD2DEG;
    vehicle_state_.direction = msg->direction;
    vehicle_state_.operation_mode = static_cast<unsigned char>(msg->operation_mode);

    if (msg->direction == 1){
        vehicle_state_.gear = vehicle_state_.DRIVE;
    }
    else{vehicle_state_.gear = vehicle_state_.REVERSE;}
    
    mutex_vehicle_state_.unlock();
}

void StateGenerator::CallbackGnss(const autoku_msgs::Gnss::ConstPtr& msg) {
    mutex_vehicle_state_.lock();

    double ref_latitude_rad = ref_lat_ * M_PI / 180.;

    double m1 = 111132.92;
    double m2 = -559.82;
    double m3 = 1.175;
    double m4 = -0.0023;

    double p1 = 111412.84;
    double p2 = -93.5;
    double p3 = 0.118;

    double lenoflat = m1 + (m2 * cos(2 * ref_latitude_rad))
    + (m3 * cos(4 * ref_latitude_rad))
    + (m4 * cos(6 * ref_latitude_rad));
    
    double lenoflon = p1 * cos(ref_latitude_rad)
    + (p2 * cos(3 * ref_latitude_rad))
    + (p3 * cos(5 * ref_latitude_rad));


    vehicle_state_.x = (msg->longitude - ref_lon_)*lenoflon;
    vehicle_state_.y = (msg->latitude  - ref_lat_)*lenoflat;
    vehicle_state_.z = 0.0;//msg->altitude;
   
    
    double azimuth_heading = (-(msg -> heading) + 90.0) * M_PI/180.0;
    if (azimuth_heading > M_PI){
        azimuth_heading -= 2.0 * M_PI;
    }
    else if (azimuth_heading < -M_PI){
        azimuth_heading += 2.0 * M_PI;
    }

    vehicle_state_.yaw = azimuth_heading; 

    vehicle_state_.x_sigma = (msg->longitude_sigma);
    vehicle_state_.y_sigma = (msg->latitude_sigma);
    vehicle_state_.z_sigma = (msg->altitude_sigma);
    vehicle_state_.yaw_sigma = (msg->heading_sigma);

    mutex_vehicle_state_.unlock();
}

void StateGenerator::CallbackReference(const autoku_msgs::WGS84::ConstPtr& msg){
    ref_lat_ = msg->latitude;
    ref_lon_ = msg->longitude;
    ROS_WARN("ref_lat : %.5f", ref_lat_);
    ROS_WARN("ref_lon : %.5f", ref_lon_);
    ref_height_ = msg->height;
}


int main(int argc, char **argv){
    std::string node_name = "state_generator";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");
    
    int id;
    if (!nh.getParam("task_id/id_state_generator", id))
        id = 21;

    double period;
    if (!nh.getParam("task_period/period_state_generator", period))
        period = 0.02;    

    ROS_INFO("Complete to get parameters! (ID: %d, Period: %.3f)", id, period);
    
    StateGenerator main_task(id, node_name, period);
    main_task.Exec();

    return 0;
}