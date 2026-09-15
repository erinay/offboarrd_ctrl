#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>

#include "offboard_attitude.h"

#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

class OffboardAttitude: public rclcpp::Node
{
public:
    OffboardAttitude() : Node("offboard_attitude")
    {
        this->declare_parameter<std::string>("mpc_type", "single");
        mpc_type_ = this->get_parameter("mpc_type").as_string();
        this->declare_parameter<double>("yaw_rate_rad_s", 0.15);
        yaw_rate_rad_s_ = static_cast<float>(this->get_parameter("yaw_rate_rad_s").as_double());
        this->declare_parameter<bool>("hover_test", false);
        hover_test_ = this->get_parameter("hover_test").as_bool();
        this->declare_parameter<double>("hover_altitude", flight_alt);
        hover_altitude_ = static_cast<float>(this->get_parameter("hover_altitude").as_double());
        this->declare_parameter<double>("waypoint_radius", 1.0);
        waypoint_radius_ = static_cast<float>(this->get_parameter("waypoint_radius").as_double());
        this->declare_parameter<bool>("use_lio_visual_odometry", false); // Fast-lio turned off for sim 
        use_lio_visual_odometry_ = this->get_parameter("use_lio_visual_odometry").as_bool();
        R_NED2ENU << 0, 1, 0,
            1, 0, 0,
            0, 0, -1;

        R_FLU2FRD << 1,  0,  0,
                 0, -1,  0,
                 0,  0, -1;
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
               .best_effort()
               .durability_volatile();
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		attitude_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        if (use_lio_visual_odometry_) {
            lio_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
                    "/fmu/in/vehicle_visual_odometry", 1);
            lio_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/odometry", qos, std::bind(&OffboardAttitude::state_callback, this, std::placeholders::_1));
            RCLCPP_WARN(
                this->get_logger(),
                "LIO visual odometry is ENABLED: forwarding /odometry to PX4 EKF");
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "LIO visual odometry is disabled; controller state comes only from PX4 vehicle_odometry");
        }
        vehicle_state_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
            std::bind(&OffboardAttitude::position_listener,this, std::placeholders::_1));
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v4", qos,
            std::bind(&OffboardAttitude::status_callback, this, std::placeholders::_1));
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("/dstar_path", qos,
            std::bind(&OffboardAttitude::path_listener, this, std::placeholders::_1));
            
        auto timer_callback = [this]() -> void {
            publish_offboard_control_mode();
            publish_attitude_command();
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }   
private:
    // True once the vehicle is at or past corner index `c` of a fresh D* path -- either within
    // waypoint_radius_ of it, or its live position already projects beyond it along the incoming
    // segment poses[c-1]->poses[c] (a wide turn can clear the corner without ever entering that
    // radius). Read against the live r, not the path's own poses[0]: D* replans from wherever the
    // robot was *at plan time*, which can already be stale by the time this message is handled.
    //
    // Ported from control_node.h's (poisson_px4) fix for the same corner-stall bug: hard-coding
    // the target to poses[1] forever meant it never advanced as the vehicle closed in, and this
    // node's P-control cascade (velocity_reference) fades to ~0 as uv approaches a stationary
    // target, so the approach stalled at the corner instead of continuing past it. Unlike
    // control_node.h, this only ports the advance logic (root cause), not its further
    // waypoint_lookahead extension -- that extension is only safe there because it's gated by a
    // straight-line CBF chord check, and this node has no CBF/map to check a chord against.
    bool corner_passed(const std::vector<geometry_msgs::msg::PoseStamped>& poses, std::size_t c){
        /**
         * @brief Checks if the vehicle has passed or gotten close enough to a corner waypoint.
         * 
         * Prevents the vehicle from getting stuck trying to hit a tight turn exactly 
         * by checking both a proximity radius and a projection across the segment boundary.
         * 
         * @param poses All waypoints in dstar path
         * @param c Index of upcoming corner waypoint
         * @return true If the robot is within the waypoint radius or has structurally crossed past the corner.
         * @return false If the robot has not yet reached or passed the corner.         
         */

        const Eigen::Vector2f corner((float)poses[c].pose.position.x, (float)poses[c].pose.position.y);
        const Eigen::Vector2f pos(r.x(), r.y());
        if ((pos - corner).norm() < waypoint_radius_) return true;

        const Eigen::Vector2f prev((float)poses[c-1].pose.position.x, (float)poses[c-1].pose.position.y);
        const Eigen::Vector2f seg = corner - prev;
        const float seg_len2 = seg.squaredNorm();
        if (seg_len2 < 1e-6f) return false; // degenerate segment -- nothing to project onto
        const float t = (pos - prev).dot(seg) / seg_len2;
        return t >= 1.0f;
    }

    void path_listener(const nav_msgs::msg::Path::SharedPtr msg){
        if (hover_test_) {
            return;
        }

        if(msg->poses.size()<=1){
            have_waypoint_=false;
            return;
        }

        // D* replans from the current robot pose and republishes the whole path on every message,
        // so progress needs no persistent waypoint index across callbacks -- skip any corner
        // already reached/overshot since D* last planned, so the target keeps advancing instead of
        // sticking to the same (possibly already-passed) corner.
        std::size_t corner = 1;
        while (corner + 1 < msg->poses.size() && corner_passed(msg->poses, corner)){
            ++corner;
        }

        waypoint.x() = (float)(msg->poses[corner].pose.position.x);
        waypoint.y() = (float)(msg->poses[corner].pose.position.y);
        waypoint.z() = flight_alt;

        have_waypoint_=true;
    }

    void state_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        t = msg->header.stamp;

        Eigen::Vector3f pos_enu{
            (float)msg->pose.pose.position.x,
            (float)msg->pose.pose.position.y,
            (float)msg->pose.pose.position.z
        };

        Eigen::Vector3f vel_enu{
            (float)msg->twist.twist.linear.x,
            (float)msg->twist.twist.linear.y,
            (float)msg->twist.twist.linear.z,
        };

       Eigen::Quaternionf q_measured{
            (float)msg->pose.pose.orientation.w,
            (float)msg->pose.pose.orientation.x,
            (float)msg->pose.pose.orientation.y,
            (float)msg->pose.pose.orientation.z
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

    void publish_fmu_odometry(){

        px4_msgs::msg::VehicleOdometry msg{};

        msg.timestamp =  t.sec * 1000000ULL + t.nanosec / 1000ULL;
        msg.timestamp_sample = msg.timestamp;

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

    void position_listener(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        Eigen::Vector3f pos_ned{msg->position[0], msg->position[1], msg->position[2]};
        Eigen::Vector3f vel_ned{msg->velocity[0], msg->velocity[1], msg->velocity[2]};
        Eigen::Quaternionf q_ned{msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
        r = R_NED2ENU * pos_ned;
        v = R_NED2ENU * vel_ned;
        R = R_NED2ENU * q_ned.toRotationMatrix() * R_FLU2FRD;// * R_FLU2FRD.transpose();
        
        position_listened=true;

        // Cache the most recent disarmed state.  The hover target is latched
        // from this value on the armed transition, so takeoff cannot move the
        // XY reference before the test begins.
        if (hover_test_ && !armed) {
            prearm_position_ = r;
            have_prearm_position_ = true;
        }
    }

    void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        const bool now_armed =
            msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

        if (hover_test_ && now_armed && !armed) {
            // Prefer the state received while disarmed.  Falling back to the
            // latest state also handles an arm event that arrives before the
            // first vehicle-odometry sample.
            rd = have_prearm_position_ ? prearm_position_ : r;
            rd.z() = hover_altitude_;

            RCLCPP_INFO(
                this->get_logger(),
                "Hover target latched at arm: %.2f %.2f %.2f",
                rd.x(), rd.y(), rd.z());
        } else if (hover_test_ && !now_armed && armed) {
            // A later arm cycle must use a new pre-arm state rather than a
            // reference from the preceding flight.
            have_prearm_position_ = false;
        }

        armed = now_armed;
    }

    void velocity_reference()
    {
        // Replace with MPC single integrator reference in the future
        const float kp = 1.0;
        vd << 0.0, 0.0, 0.0;

        std::cout<<(flight_alt-0.15f)<<std::endl;
        std::cout<<(r.z()>(flight_alt-0.15f))<<std::endl;

        if (!takeoff_complete_ && r.z() >= 0.35f) {
            takeoff_complete_ = true;
        }

        if (hover_test_ || !have_waypoint_ || !takeoff_complete_) {
            // Before arming, continue publishing benign setpoints so PX4 can
            // enter Offboard.  Once armed, rd is deliberately never modified
            // until disarm resets the latch in status_callback().
            rd = have_prearm_position_ ? prearm_position_ : r;
            rd.z() = hover_altitude_;
    
        } else {
            std::cout<<"followign waypoint"<<std::endl;
            rd.x() = waypoint.x();
            rd.y() = waypoint.y();
            rd.z() = flight_alt;
        }

        uv = kp*(rd-r) + vd;

        const float xy_vel = std::sqrt(uv.x()*uv.x() + uv.y()*uv.y());

        if (xy_vel > maximum_xy_speed) {
            const float ratio = maximum_xy_speed / xy_vel;
            uv.x() *= ratio;
            uv.y() *= ratio;
        }
        uv.z() = std::clamp(uv.z(), -maximum_z_speed, maximum_z_speed);
    }

    void acceleration_reference()
    {   
        // Replace with MPC double integrator reference in the future
        const float kd = 0.6;

                    // Go through these parameters
        // vd << 0.0, 0.0, 0.0;

        ad = kd*(uv-v);
    }

    void attitude_reference(){
        Eigen::Vector3f grav = {0.0, 0.0, -g};
        fd = mass * (ad-grav);

        // Advance the heading setpoint each control cycle to rotate in place.
        // atan2 keeps the value bounded while preserving the equivalent heading.
        yawd_ = std::atan2(R(1,0), R(0,0));
        if (r.z() > 0.4f) {
            const float xy_speed = std::sqrt(uv.x()*uv.x() + uv.y()*uv.y());
            // yawd_ += 10.0f*M_PI/180.0f;
            // yawd_ = std::atan2(std::sin(yawd_), std::cos(yawd_));
            if (xy_speed > 0.05f) {
                yawd_ = std::atan2(uv.y(), uv.x());
            }
        }
        const float yawd = yawd_;
        std::cout<<yawd<<std::endl;
        
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
        Eigen::Matrix3f Rd_ned = R_NED2ENU.transpose()*Rd*R_FLU2FRD.transpose(); // body and world frame are both in NED
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
        // // PX4 consumes attitude setpoints in NED, whose yaw sign is opposite ENU.
        // // This feed-forward command therefore matches the rotating q_d target.
        // msg.yaw_sp_move_rate = -yaw_rate_rad_s_;
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
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr lio_odometry_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;

    bool armed = false;
    bool position_listened = false;
    bool hover_test_ = false;
    bool use_lio_visual_odometry_ = false;
    bool have_prearm_position_ = false;
    bool have_waypoint_ = false;
    bool takeoff_complete_ = false;
    float waypoint_radius_ = 1.0f;
    float thrust = 0.0;
    float yawd_ = 0.0f;
    float yaw_rate_rad_s_ = 0.15f;
    float time_traj = 0.0;
    int count = 0;
    Eigen::Vector3f r = Eigen::Vector3f::Zero();
    Eigen::Vector3f prearm_position_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f waypoint = Eigen::Vector3f::Zero();
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    Eigen::Matrix3f R_NED2ENU, R_FLU2FRD;
    std::string mpc_type_; 
    std::ofstream log_file; 
    
    Eigen::Quaternionf qd,q;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f rd = Eigen::Vector3f::Zero();
    Eigen::Vector3f vd, ad, fd, uv, u_mpc, pose_cov, vel_cov;
    Eigen::VectorXd yref, y, yref_e;
    builtin_interfaces::msg::Time t;
    
    const float t_hover = 0.8; // 2kg.
    const float g = 9.81;
    const float mass = 1.0;
    const float dt = 0.1; 
    const float flight_alt = 0.5;
    float hover_altitude_ = flight_alt;
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
