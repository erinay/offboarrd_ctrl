#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>

#include "offboard_attitude.h"

#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>

class OffboardAttitude: public rclcpp::Node
{
public:
    OffboardAttitude() : Node("offboard_attitude")
    {
        this->declare_parameter<std::string>("mpc_type", "single");
        mpc_type_ = this->get_parameter("mpc_type").as_string();
        // Positive values rotate counter-clockwise in the ENU world frame.
        this->declare_parameter<double>("yaw_rate_rad_s", 0.15);
        yaw_rate_rad_s_ = static_cast<float>(this->get_parameter("yaw_rate_rad_s").as_double());

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		attitude_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        R_NED2ENU << 0, 1, 0,
                    1, 0, 0,
                    0, 0, -1;

        R_FLU2FRD << 1,  0,  0,
                 0, -1,  0,
                 0,  0, -1;


        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
               .best_effort()
               .durability_volatile();

        vehicle_state_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
            std::bind(&OffboardAttitude::position_listener,this, std::placeholders::_1));
        waypoint_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/waypoint", qos, 
            std::bind(&OffboardAttitude::waypoint_callback, this, std::placeholders::_1));
            
        auto timer_callback = [this]() -> void {  // <-- capture timer_count by reference
            publish_offboard_control_mode();
            publish_attitude_command();
            
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }   
private:

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1=0.0f, float param2=0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void position_listener(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        Eigen::Vector3f pos_ned{msg->position[0], msg->position[1], msg->position[2]};
        Eigen::Vector3f vel_ned{msg->velocity[0], msg->velocity[1], msg->velocity[2]};
        Eigen::Quaternionf q_ned{msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
        r = R_NED2ENU * pos_ned;
        v = R_NED2ENU * vel_ned;
        R = R_NED2ENU * q_ned.toRotationMatrix() * R_FLU2FRD.transpose();
    }

    void waypoint_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg){
        r_d.x() = (float)(msg->pose.position.x);
        r_d.y() = (float)(msg->pose.position.y);
        r_d.z() = flight_alt;
    }

    void velocity_reference()
    {
        // Replace with MPC single integrator reference in the future
        const float kp = 2.0;
        vd << 0.0, 0.0, 0.0;

        uv = kp*(rd-r) + vd;

        const float xy_vel = (uv.x()*uv.x()+uv.y()+uv.y());
        const float z_vel = (uv.z()*uv.z());
        if(xy_vel>(maximum_xy_speed*maximum_xy_speed)){
            const float ratio = maximum_xy_speed/xy_vel;
            uv.x() *= ratio;
            uv.y() *= ratio;
        }
        if(z_vel>(maximum_z_speed*maximum_z_speed)){
            std::clamp(uv.z(), -maximum_z_speed, maximum_z_speed);
        }
    }

    void acceleration_reference()
    {   
        // Replace with MPC double integrator reference in the future
        const float kd = 0.5;

                    // Go through these parameters
        // vd << 0.0, 0.0, 0.0;

        ad = kd*(uv-v);
    }

    void attitude_reference(){
        Eigen::Vector3f grav = {0.0, 0.0, -g};
        fd = mass * (ad-grav);

        // Advance the heading setpoint each control cycle to rotate in place.
        // atan2 keeps the value bounded while preserving the equivalent heading.
        yawd_ = std::atan2(std::sin(yawd_ + yaw_rate_rad_s_ * dt),
                           std::cos(yawd_ + yaw_rate_rad_s_ * dt));
        const float yawd = yawd_;
        Eigen::Vector3f bzd = fd;
        bzd.normalize();
        Eigen::Vector3f byawd{-std::sin(yawd), std::cos(yawd), 0.0};
        Eigen::Vector3f bxd = byawd.cross(bzd);
        bxd.normalize();
        Eigen::Vector3f byd = bzd.cross(bxd); 
        byd.normalize();

        Eigen::Matrix3f Rd;
        Rd.col(0) = bxd;
        Rd.col(1) = byd;
        Rd.col(2) = bzd;
        Eigen::Matrix3f Rd_ned = R_NED2ENU.transpose()*Rd*R_NED2ENU; // body and world frame are both in NED
        qd = Eigen::Quaternionf(Rd_ned);

        float F = fd.dot(R.col(2));
        float F_max = 1.0/t_hover*g*mass;
        thrust =  -F/F_max;
        thrust = std::clamp(thrust, -1.0f, 0.0f);
    }

    void publish_attitude_command()
    {   
        velocity_reference();
        acceleration_reference();
        attitude_reference();
        
        px4_msgs::msg::VehicleAttitudeSetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.q_d[0] = qd.w();
        msg.q_d[1] = qd.x();
        msg.q_d[2] = qd.y();
        msg.q_d[3] = qd.z();
        msg.thrust_body[0] = 0.0; // FRD frame [-1,1]
        msg.thrust_body[1] = 0.0;
        msg.thrust_body[2] = thrust;

        attitude_setpoint_publisher_->publish(msg);

        std::cout << "Current Z: " << r.z() << " meters, Thrust: " << thrust << std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_subscriber_;

    bool armed = false;
    float thrust = 0.0;
    float yawd_ = 0.0f;
    float yaw_rate_rad_s_ = 0.15f;
    float time_traj = 0.0;
    int count = 0;
    Eigen::Vector3f r, v, r_d;
    Eigen::Matrix3f R_NED2ENU, R_FLU2FRD;
    std::string mpc_type_; 
    std::ofstream log_file; 
    
    Eigen::Quaternionf qd;
    Eigen::Matrix3f R;
    Eigen::Vector3f rd, vd, ad, fd, uv, u_mpc;
    Eigen::VectorXd yref, y, yref_e;
    
    const float t_hover = 0.72; // 2kg.
    const float g = 9.81;
    const float mass = 1.0;
    const float dt = 0.1; 
    const float flight_alt = 1.5;
    const float maximum_xy_speed = 0.5f;
    const float maximum_z_speed = 0.4f;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);

    try{
        rclcpp::spin(std::make_shared<OffboardAttitude>());
        throw("Terminated");
    }
    catch(const char* msg){
        rclcpp::shutdown();
        std::cout << msg << std::endl;
    }
}
