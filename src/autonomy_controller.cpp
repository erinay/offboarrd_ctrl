#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>
#include <iostream>

#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>

class AutonomyController: public rclcpp::Node{
    public:
        AutonomyController(): Node("autonomy_node"){
            rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1));
			sensor_qos.best_effort();
			sensor_qos.durability_volatile();


            init_flag=false;
            land_flag=false;
            hover_flag=false;
            arming_flag=false;
            brake_flag=false;
            arm_command=false;

            R_NED2ENU << 0, 1, 0,
                         1, 0, 0,
                         0, 0, -1;

            R_FLU2FRD << 1, 0, 0,
                         0, -1, 0,
                         0, 0, -1;

            lio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry", sensor_qos, std::bind(&AutonomyController::state_callback, this, std::placeholders::_1));
            vehicle_odom_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", sensor_qos,
                std::bind(&AutonomyController::position_listener,this, std::placeholders::_1));
            vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v4", sensor_qos,
                std::bind(&AutonomyController::status_callback, this, std::placeholders::_1));
            lio_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
                "/fmu/in/vehicle_visual_odometry", 1);
            vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
            offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            
            //subscribers and publishers: adding for UDP
            arming_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/Drone1/arming", sensor_qos, std::bind(&AutonomyController::arming_callback, this, std::placeholders::_1));
            hover_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/Drone1/end_hover", sensor_qos, std::bind(&AutonomyController::hover_callback, this, std::placeholders::_1));
        }
        
    private:

    // Publishers & Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odom_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr lio_odometry_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arming_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hover_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // States
    builtin_interfaces::msg::Time t;
    bool init_flag, land_flag, hover_flag, arming_flag, arm_command, brake_flag;
    uint64_t t_curr, state_start_time;
    Eigen::Vector3f r, v, pose_cov, vel_cov, r0, r_odom, r_hold;
    Eigen::Quaternionf q;
    Eigen::Matrix3f R_NED2ENU, R_FLU2FRD, R;
    const float TRANSLATE_TIME=4.0;
    const float BRAKE_TIME=2.0; //braking before landing

    // STATE MACHINE
    enum class MissionState{
        WAIT, 
        TAKEOFF,
        TRANSLATE, 
        BRAKE,
        LAND,
        DONE
    };

    MissionState mission_state_ = MissionState::WAIT;

    //FSM HELPER FUCNTIO
    void transition(MissionState new_state){
        mission_state_=new_state;
        state_start_time=t_curr;
        std::cout<<"STATE TRANSITIONING: "<<(int)(new_state)<<std::endl;
    }

    void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        t = msg->header.stamp;

        Eigen::Vector3f pos_enu{
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        };

        Eigen::Vector3f vel_enu{
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,
        };

       Eigen::Quaternionf q_measured{
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        };

        // Modify to rotate later
        pose_cov[0] = msg->pose.covariance[7];
        pose_cov[1] = msg->pose.covariance[0];
        pose_cov[2] = msg->pose.covariance[14];
        vel_cov[0] = msg->twist.covariance[7];
        vel_cov[1] = msg->twist.covariance[0];
        vel_cov[2] = msg->twist.covariance[14];

        r = R_NED2ENU.transpose() * pos_enu;
        v = R_NED2ENU.transpose() * vel_enu;

        //R_FRD2NED = R_ENU2NED * R_ENULiDAR *RLiDARFRD
        // PX4: Drone body in FRD, Global in NED frame
        // Last rotation rotates aligns measurements to FRD frame, LiDAR in FLU
        // First rotation rotates from ENU to NED global frame
        Eigen::Matrix3f R = R_NED2ENU.transpose() * q_measured.toRotationMatrix() * R_FLU2FRD;
        q = Eigen::Quaternionf(R);

        publish_fmu_odometry();
    }

    void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg){
        // std::cout << "Nav state: " << static_cast<int>(msg->nav_state) << std::endl;
        // std::cout << "arming state: " << static_cast<int>(msg->arming_state) << std::endl;
        if (msg->nav_state == 14 and msg->arming_state==2 and init_flag==false and arm_command==true) {
            init_flag=true;
            std::cout << "Init Flag to True, Takeoff clear" << std::endl;
            transition(MissionState::TAKEOFF);
        }
        else if (msg->nav_state==1 and msg->arming_state==1){
            // Reset with setting to manual mode and disarmed
            init_flag=false;
            hover_flag=false;
            arming_flag=false;
            arm_command=false;
            brake_flag=false;
            transition(MissionState::WAIT);
        }
    }

    void hover_callback(const std_msgs::msg::Bool::SharedPtr msg){
        hover_flag = msg->data;
    }

    void arming_callback(const std_msgs::msg::Bool::SharedPtr msg){
        // std::cout<<(int)msg->data<<std::endl;
        if (msg->data and arming_flag==false){
            std::cout<<"Arm signal received"<<std::endl;
            arming_flag = true;
            publish_arm_command();
            arm_command=true;
        }
    }

    void position_listener(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
        Eigen::Vector3f pos_ned{msg->position[0], msg->position[1], msg->position[2]};
        r_odom = pos_ned;
        t_curr = msg->timestamp;
        if(init_flag==false){
            r0 = r_odom;        }
        if (brake_flag==false){
            r_hold=r_odom;
        }
        publish_trajectory_command();
    }

    void publish_trajectory_command(){
        
        float t = static_cast<float>(t_curr-state_start_time)*1e-6f;

        switch(mission_state_) {
            case MissionState::WAIT: {
                publish_offboard_control_mode(false,true);
                publish_zero_vel();
                break;
            }

            case MissionState::TAKEOFF: {
                publish_offboard_control_mode(true, false);
                publish_takeof_command();
                if(hover_flag==true){
                    transition(MissionState::TRANSLATE);
                }
                break;
            }

            case MissionState::TRANSLATE: {
                publish_offboard_control_mode(false, true);
                publish_translate_command();
                if(t>=TRANSLATE_TIME){
                    transition(MissionState::BRAKE);
                }
                break;
            }
            case MissionState::BRAKE: {
                publish_offboard_control_mode(true, false);
                publish_hold_command();
                if(t>=BRAKE_TIME){
                    transition(MissionState::LAND);
                }
                break;
            }
            case MissionState::LAND: {
                publish_land_command();
                std::cout << "Landing" << std::endl;
                transition(MissionState::DONE);
                break;
            }
            case MissionState::DONE: {
                //DO NOTHING, no commands set
                break;
            }
        }
    }

    void publish_zero_vel(){
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
        msg.position = {NAN, NAN, NAN};
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.acceleration = {NAN,NAN,NAN};
        msg.jerk = {NAN,NAN,NAN};
        msg.yaw = NAN;
        msg.yawspeed = NAN;
        trajectory_setpoint_publisher_->publish(msg);
    }
    void publish_takeof_command(){
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
        msg.position[0] = r0.x();
        msg.position[1] = r0.y();
        msg.position[2] = r0.z()-2.0;
        msg.velocity = {NAN,NAN,NAN};
        msg.acceleration = {NAN,NAN,NAN};
        msg.jerk = {NAN,NAN,NAN};
        msg.yaw = NAN;
        msg.yawspeed = NAN;
        trajectory_setpoint_publisher_->publish(msg);
    }
    void publish_translate_command(){
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
        msg.position = {NAN, NAN, NAN};
        msg.velocity[0] = 1.0;
        msg.velocity[1] = 0.0;
        msg.velocity[2] = 0.0;
        msg.acceleration = {NAN,NAN,NAN};
        msg.jerk = {NAN,NAN,NAN};
        msg.yaw = NAN;
        msg.yawspeed = NAN;
        trajectory_setpoint_publisher_->publish(msg);
    }
    void publish_hold_command(){
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
        msg.position[0] = r_hold.x();
        msg.position[1] = r_hold.y();
        msg.position[2] = r_hold.z();
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.acceleration = {NAN,NAN,NAN};
        msg.jerk = {NAN,NAN,NAN};
        msg.yaw = NAN;
        msg.yawspeed = NAN;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_fmu_odometry(){

        px4_msgs::msg::VehicleOdometry msg{};

        msg.timestamp =  t.sec * 1000000ULL + t.nanosec / 1000ULL;

        // position, velocity, orientation
        msg.position[0] = r.x();
        msg.position[1] = r.y();
        msg.position[2] = r.z();
        msg.velocity[0] = v.x();
        msg.velocity[1] = v.y();
        msg.velocity[2] = v.z();
        msg.q[0] = q.w();
        msg.q[1] = q.x();
        msg.q[2] = q.y();
        msg.q[3] = q.z();

        //covariances
        msg.position_variance[0] = std::clamp(pose_cov[0], 0.01f, 1.0f);
        msg.position_variance[1] = std::clamp(pose_cov[1], 0.01f, 1.0f);
        msg.position_variance[2] = std::clamp(pose_cov[2], 0.01f, 1.0f);
        msg.velocity_variance[0] = std::clamp(vel_cov[0], 0.01f, 1.0f);
        msg.velocity_variance[1] = std::clamp(vel_cov[1], 0.01f, 1.0f);
        msg.velocity_variance[2] = std::clamp(vel_cov[2], 0.01f, 1.0f);
        msg.quality=1;

        msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

        lio_odometry_publisher_->publish(msg);
    }


    void publish_land_command(float param1=0.0f, float param2=0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void publish_arm_command(){
        px4_msgs::msg::VehicleCommand msg{};

        msg.param1 = 1.0;
        msg.param2 = 0.0;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }


    void publish_offboard_control_mode(bool pos_mode, bool vel_mode)
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = pos_mode;
        msg.velocity = vel_mode;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    
    try{
        rclcpp::spin(std::make_shared<AutonomyController>());
    }
    catch(const char* msg){
        rclcpp::shutdown();
        std::cout << msg << std::endl;
    }
}
