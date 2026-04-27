#include <px4_msgs/msg/offboard_control_mode.hpp>
// #include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>


#include <chrono>
#include <iostream>

class OffboardPosition: public rclcpp::Node
{
public:
    OffboardPosition() : Node("offboard_position")
    {
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		position_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleTrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
               .best_effort()
               .transient_local();
        vehicle_state_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos,
            std::bind(&OffboardAttitude::position_listener,this, std::placeholders::_1));
            
        auto timer_callback = [this]() -> void {  // <-- capture timer_count by reference
            publish_offboard_control_mode();
            publish_attitude_command();
            
            if (counter == 0) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
           }
           if (counter <5){
                counter ++;
           }
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }   
private:
    
    void arm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

        RCLCPP_INFO(this->get_logger(), "Arm command send");
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
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

    void position_listener(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        z = msg->z;
        vz = msg->vz;
    }

    void publish_attitude_command()
    {
        px4_msgs::msg::VehicleAttitudeSetpoint msg{};

        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.q_d[0] = 1.0;
        msg.q_d[1] = 0.0;
        msg.q_d[2] = 0.0;
        msg.q_d[3] = 0.0;

        // Compute thrust
        float az = kp*(zd-z)+kd*(-vz);
        float thrust =  t_hover*(az/g-1);
        
        // Clip
        // if(thrust  0.0){
        //     thrust=0.0;
        // }
        // else if (thrust >1.0){
        //     thrust=1.0;
        // }

        msg.thrust_body[0] = 0.0; // FRD frame [-1,1]
        msg.thrust_body[1] = 0.0;
        msg.thrust_body[2] = thrust;

        attitude_setpoint_publisher_->publish(msg);

        std::cout << "Current Z: " << z << " meters, Thrust: " << thrust << std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_state_sub_;

    bool armed = false;
    float thrust = 0.0;
    float z = 0.0;
    float vz = 0.0;
    int counter = 0;
    
    const float t_hover = 0.72; // 2kg.
    const float g = 9.81;
    const float kp = 1.0;
    const float kd = 0.5;
    const double zd = -5.0; // PX4 uses FRD coordinates
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);

    try{
        rclcpp::spin(std::make_shared<OffboardPosition>());
        throw("Terminated");
    }
    catch(const char* msg){
        rclcpp::shutdown();
        std::cout << msg << std::endl;
    }
}