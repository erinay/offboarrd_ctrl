#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>

#include "offboard_attitude.h"

#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>

extern "C" {
    #include "acados_solver_single_integrator.h"
    #include "acados_solver_double_integrator.h"
}

class OffboardAttitude: public rclcpp::Node
{
public:
    OffboardAttitude() : Node("offboard_attitude")
    {
        this->declare_parameter<std::string>("mpc_type", "single");
        mpc_type_ = this->get_parameter("mpc_type").as_string();

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		attitude_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        R_NED2ENU << 0, 1, 0,
                    1, 0, 0,
                    0, 0, -1;

        load_csv("src/offboard_ctrl/trajectory_data/slow_caltech_togt_traj.csv");
        std::string filename = "src/offboard_ctrl/data/tracking_" + mpc_type_ + "_D.csv";
        log_file.open(filename);
        log_file << "state,t,r_x,r_y,r_z,pd_x,pd_y,pd_z,u_x,u_y,u_z,ud_x,ud_y,ud_z, a_x, a_y, a_z, ad_x, ad_y, ad_z\n";

        if(mpc_type_=="single"){
            std::cout << "Single Integrator MPC running!" << std::endl;
            capsule = single_integrator_acados_create_capsule();
            single_integrator_acados_create(capsule);
            nlp_in = single_integrator_acados_get_nlp_in(capsule);
            nlp_out = single_integrator_acados_get_nlp_out(capsule);
            nlp_solver = single_integrator_acados_get_nlp_solver(capsule);
            yref.resize(6);
            y.resize(3);
            yref_e.resize(3);
        }
        else if(mpc_type_=="double"){
            std::cout << "Double Integrator MPC running!" << std::endl;
            capsule_double = double_integrator_acados_create_capsule();
            double_integrator_acados_create(capsule_double);
            nlp_in = double_integrator_acados_get_nlp_in(capsule_double);
            nlp_out = double_integrator_acados_get_nlp_out(capsule_double);
            nlp_solver = double_integrator_acados_get_nlp_solver(capsule_double);
            yref.resize(9);
            y.resize(6);
            yref_e.resize(6);
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
               .best_effort()
               .transient_local();

        vehicle_state_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
            std::bind(&OffboardAttitude::position_listener,this, std::placeholders::_1));
            
        auto timer_callback = [this]() -> void {  // <-- capture timer_count by reference
            publish_offboard_control_mode();
            publish_attitude_command();
            
        };

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }   
private:
    
    void load_csv(const std::string& filename)
    {
        std::ifstream file(filename); // open file given filename
        std::string line;

        std::getline(file,line); //gets first line (header) and does not nothing with it

        while(std::getline(file,line)){
            std::stringstream ss(line); // cretes "stream" from line, allows us to scan it
            std::string cell;
            std::vector<float> values;

            // split line every time it hits comma. and stores in "cell"
            while(std::getline(ss, cell, ',')){
                values.push_back(std::stof(cell)); // convert string in cell to double value
            }

            TrajectoryPoint point;
            point.t = values[0];
            point.pos = Eigen::Vector3f(values[1], values[2], values[3]);
            point.vel = Eigen::Vector3f(values[8], values[9], values[10]);
            point.acc = Eigen::Vector3f(values[14], values[15], values[16]);

            traj.push_back(point); // place into trajectory vector
        }
    }
    void publish_offboard_control_mode()
    {
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
        R = R_NED2ENU * q_ned.toRotationMatrix() * R_NED2ENU.transpose();
    }

    void velocity_reference()
    {
        // Replace with MPC single integrator reference in the future
        const float kp = 2.0;
        rd << traj[0].pos(0), traj[0].pos(1), traj[0].pos(2);
        vd << 0.0, 0.0, 0.0;

        uv = kp*(rd-r) + vd;
    }

    void mpc_velocity_reference()
    {        
        y << r(0), r(1), r(2);
        int N = single_integrator_acados_get_nlp_dims(capsule)->N;

        // Make sure I.C. starts at current state
        ocp_nlp_constraints_model_set(single_integrator_acados_get_nlp_config(capsule), single_integrator_acados_get_nlp_dims(capsule), nlp_in, nlp_out, 0, "lbx", y.data());
        ocp_nlp_constraints_model_set(single_integrator_acados_get_nlp_config(capsule), single_integrator_acados_get_nlp_dims(capsule), nlp_in, nlp_out, 0, "ubx", y.data());

        // Set reference
        // This loop says that for each stage, the terminal position is where we want to be
        for (int i=0; i<=N; i++){    
            float t_k = time_traj + i*dt;
            int closest_idx = closest_index(t_k);

            // Without interpolation
            yref << traj[closest_idx].pos(0), traj[closest_idx].pos(1), traj[closest_idx].pos(2), traj[closest_idx].vel(0), traj[closest_idx].vel(1), traj[closest_idx].vel(2);

            if (i<N){
                ocp_nlp_cost_model_set(single_integrator_acados_get_nlp_config(capsule), single_integrator_acados_get_nlp_dims(capsule),
                    nlp_in, i, "yref", yref.data());
            }
            else{
                yref_e << traj[closest_idx].pos(0), traj[closest_idx].pos(1), traj[closest_idx].pos(2);
                ocp_nlp_cost_model_set(single_integrator_acados_get_nlp_config(capsule), single_integrator_acados_get_nlp_dims(capsule), nlp_in, N, "yref", yref_e.data());
            }
        }

        single_integrator_acados_solve(capsule);

        Eigen::Vector3d uv_double;

        ocp_nlp_out_get(single_integrator_acados_get_nlp_config(capsule), single_integrator_acados_get_nlp_dims(capsule),
            nlp_out, 0, "u", uv_double.data());
        uv = uv_double.cast<float>();
        std::cout << "MPC Output: " << uv.transpose() << std::endl;
        std::cout << "y: " << y.transpose() << std::endl;
        std::cout << "yref: " << yref.transpose() << std::endl;
        std::cout << "uv: " << uv.transpose() << std::endl;
    }

    void acceleration_reference()
    {   

        // Replace with MPC double integrator reference in the future
        const float kd = 0.5;

        // Go through these parameters
        // vd << 0.0, 0.0, 0.0;

        ad = kd*(uv-v);
    }

    void mpc_acceleration_reference()
    {
        vd << 0.0, 0.0, 0.0;
        ad << 0.0, 0.0, 0.0; // Acceleration / control desired to be 0
        y << r(0), r(1), r(2), v(0), v(1), v(2);

        int N = double_integrator_acados_get_nlp_dims(capsule_double)->N; 

        // Set 'bound' constraint on state
        ocp_nlp_constraints_model_set(double_integrator_acados_get_nlp_config(capsule_double), double_integrator_acados_get_nlp_dims(capsule_double),
            nlp_in, nlp_out,  0, "lbx", (void*) y.data());
        ocp_nlp_constraints_model_set(double_integrator_acados_get_nlp_config(capsule_double), double_integrator_acados_get_nlp_dims(capsule_double),
            nlp_in, nlp_out, 0, "ubx", (void*) y.data());
        
        // Set reference
        // This loop says that for each stage, the terminal position is where we want to be
        if(state==START){
            yref <<  traj[0].pos(0), traj[0].pos(1), traj[0].pos(2),  traj[0].vel(0), traj[0].vel(1), traj[0].vel(2), ad(0), ad(1), ad(2);
            yref_e << traj[0].pos(0), traj[0].pos(1), traj[0].pos(2), vd(0), vd(1), vd(2);
            for (int i=0; i<=N; i++){
                if (i<N){
                    ocp_nlp_cost_model_set(double_integrator_acados_get_nlp_config(capsule_double), double_integrator_acados_get_nlp_dims(capsule_double),
                        nlp_in, i, "yref", yref.data());
                }
                else{
                    yref_e << traj[0].pos(0), traj[0].pos(1), traj[0].pos(2), vd(0), vd(1), vd(2);
                    ocp_nlp_cost_model_set(double_integrator_acados_get_nlp_config(capsule_double), double_integrator_acados_get_nlp_dims(capsule_double), nlp_in, N, "yref", yref_e.data());
                }
            }
        }
        else if (state==TRACK_TRAJ){
            for (int i=0; i<N; i++){
                float t_k = time_traj + i*dt;
                int closest_idx = closest_index(t_k);

                // Without interpolation
                yref << traj[closest_idx].pos(0), traj[closest_idx].pos(1), traj[closest_idx].pos(2), traj[0].vel(0), traj[0].vel(1), traj[0].vel(2),  traj[0].acc(0), traj[0].acc(1), traj[0].acc(2);
                
                if (i<N){
                    ocp_nlp_cost_model_set(double_integrator_acados_get_nlp_config(capsule_double), double_integrator_acados_get_nlp_dims(capsule_double),
                        nlp_in, i, "yref", yref.data());
                }
                else{
                    yref_e << traj[closest_idx].pos(0), traj[closest_idx].pos(1), traj[closest_idx].pos(2), vd(0), vd(1), vd(2);
                    ocp_nlp_cost_model_set(double_integrator_acados_get_nlp_config(capsule_double), double_integrator_acados_get_nlp_dims(capsule_double), nlp_in, N, "yref", yref_e.data());
                }
            }
        }


        double_integrator_acados_solve(capsule_double);

        Eigen::Vector3d ad_double;
        ocp_nlp_out_get(double_integrator_acados_get_nlp_config(capsule_double), double_integrator_acados_get_nlp_dims(capsule_double),
            nlp_out, 0, "u", ad_double.data());

        ad = ad_double.cast<float>();

    }

    void attitude_reference()
    {
        Eigen::Vector3f grav = {0.0, 0.0, -g};
        fd = mass * (ad-grav);

        float yawd = 0.0;
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
        if (state==START){
            velocity_reference();
            acceleration_reference();
        }
        else if (mpc_type_=="single"){
            mpc_velocity_reference();
            acceleration_reference();
        }
        else if (mpc_type_=="double"){
            mpc_acceleration_reference();
        }
        // velocity_reference();
        // acceleration_reference();
        
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
        
        //update state machine if neccessary
        Eigen::Vector3f p_start(traj[0].pos(0), traj[0].pos(1), traj[0].pos(2));
        const float pos_tol = 0.2f;
        const float vel_tol = 0.05f;

        if ((r - p_start).norm() < pos_tol &&
            v.norm() < vel_tol &&
            state == START)
        {
            state = TRACK_TRAJ;
            time_traj = 0.0;
        }
        if (state==TRACK_TRAJ){
            time_traj+=0.1;
        }

        std::cout << "State" << state << std::endl;

        // Logging
        int closest_idx = closest_index(time_traj);
        log_file<< (state == TRACK_TRAJ ? 1 : 0) << "," <<traj[closest_idx].t << "," << r.x() << "," << r.y() << "," << r.z() << "," << traj[closest_idx].pos(0) << "," << traj[closest_idx].pos(1) << "," << traj[closest_idx].pos(2) << 
            "," << uv[0] << "," << uv[1] << "," << uv[2] << "," << traj[closest_idx].vel(0) << "," << traj[closest_idx].vel(1) << "," << traj[closest_idx].vel(2) << 
            "," << ad[0] << "," << ad[1] << "," << ad[2] << "," << traj[closest_idx].acc(0) << "," << traj[closest_idx].acc(1) << "," << traj[closest_idx].acc(2) << "\n";
    }

    int closest_index(float t){
        // Returns first index when traj[i].t >= t
        int it = std::lower_bound(traj.begin(), traj.end(), t, 
            [](const TrajectoryPoint& pt, float val){ return pt.t < val;})-traj.begin();
        if (it == 0) return 0;
        if (it == (int)traj.size()) return traj.size()-1;
        return it;
    }

    enum MPC_state {
        START,
        TRACK_TRAJ
    };

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_state_sub_;

    bool armed = false;
    float thrust = 0.0;
    float time_traj = 0.0;
    int count = 0;
    MPC_state state = START;
    Eigen::Vector3f r, v;
    Eigen::Matrix3f R_NED2ENU;
    std::string mpc_type_; 
    std::vector<TrajectoryPoint> traj;
    std::ofstream log_file; 

    single_integrator_solver_capsule* capsule;
    double_integrator_solver_capsule* capsule_double;
    ocp_nlp_in* nlp_in;
    ocp_nlp_out* nlp_out;
    ocp_nlp_solver* nlp_solver;

    
    Eigen::Quaternionf qd;
    Eigen::Matrix3f R;
    Eigen::Vector3f rd, vd, ad, fd, uv, u_mpc;
    Eigen::VectorXd yref, y, yref_e;
    
    const float t_hover = 0.72; // 2kg.
    const float g = 9.81;
    const float mass = 1.0;
    const float dt = 0.1; 
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