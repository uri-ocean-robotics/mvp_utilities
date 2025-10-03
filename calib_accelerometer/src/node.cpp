#include <memory>
#include <vector>
#include <atomic>
#include <Eigen/Eigen>
#include <fstream>
#include <chrono>
#include <ctime>  

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"   // Simple empty service type
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

class ImuAccCalib : public rclcpp::Node
{
public:
    ImuAccCalib()
    : Node("imu_service_node"), save_(false), calib_(false)
    {
        // Get param
        this->declare_parameter<int>("max_samples", 100);
        if (!this->get_parameter("max_samples", max_samples_)) {
            RCLCPP_ERROR(this->get_logger(), "max_samples: no param available!");
        }
        this->declare_parameter<bool>("log", false);
        if (!this->get_parameter("log", log_)) {
            RCLCPP_ERROR(this->get_logger(), "log: no param available!");
        }        

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscriber to IMU topic
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/imu/data", 1000,
          std::bind(&ImuAccCalib::imu_callback, this, std::placeholders::_1));

        // Three service servers with Empty type
        start_srv_ = this->create_service<std_srvs::srv::Empty>(
          "start_sampling", std::bind(&ImuAccCalib::startCallback, this,
                              std::placeholders::_1, std::placeholders::_2));

        stop_srv_ = this->create_service<std_srvs::srv::Empty>(
          "stop_sampling", std::bind(&ImuAccCalib::stopCallback, this,
                              std::placeholders::_1, std::placeholders::_2));

        calib_srv_ = this->create_service<std_srvs::srv::Empty>(
          "calibrate_accl", std::bind(&ImuAccCalib::calibCallback, this,
                                std::placeholders::_1, std::placeholders::_2));
    }

private:
    // --- Subscriber callback ---
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(),
        //             "IMU orientation: [%.3f, %.3f, %.3f, %.3f]",
        //             msg->orientation.x, msg->orientation.y,
        //             msg->orientation.z, msg->orientation.w);

        // ----------------------------------------------------------------- //
        // start save if needed
        // ----------------------------------------------------------------- //

        if(save_)
        {
            // store the received acceleration
            buffer_accl_.emplace_back(Eigen::Vector3d(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
            ));

            // if buffer eached the max, erase the first one
            if(buffer_accl_.size() > max_samples_)
            {
                buffer_accl_.erase(buffer_accl_.begin(), buffer_accl_.begin() + 100);
                RCLCPP_WARN(this->get_logger(), "Larger then the max %d, drop the oldest 100", max_samples_);
            }

            // report the size of buffer
            if(buffer_accl_.size() % 100 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Got %ld acceleration readings", buffer_accl_.size());
                RCLCPP_INFO(this->get_logger(), "IMU accel example: %f, %f, %f", 
                    msg->linear_acceleration.x,
                    msg->linear_acceleration.y,
                    msg->linear_acceleration.z);
            }
        }
        
        // ----------------------------------------------------------------- //
        // do the calibration
        // ----------------------------------------------------------------- //

        if(calib_)
        {
            calib_ = false;

            calibrateAccl();

            std::vector<Eigen::Vector3d>().swap(buffer_accl_);
        }  
        
        // publish tf
        if(!R_I_G_.isZero() && !ba_avg_.isZero())
        {
            // create the tf
            Eigen::Isometry3d eigen_tf = Eigen::Isometry3d::Identity();
            eigen_tf.linear() = R_I_G_.transpose();
            eigen_tf.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
            
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->get_clock()->now();
            tf_msg.header.frame_id = "world";
            tf_msg.child_frame_id = "imu";
            tf_msg.transform = tf2::eigenToTransform(eigen_tf).transform; 

            // Publish transform
            tf_broadcaster_->sendTransform(tf_msg);
        }

    }

    // --- Service callbacks ---
    void startCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                          std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        RCLCPP_INFO(this->get_logger(), "Start collect acceleration");
        save_ = true;
    }

    void stopCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                          std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        RCLCPP_INFO(this->get_logger(), "Stop collect acceleration");
        save_ = false;
    }

    void calibCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
                            std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
    {
        RCLCPP_INFO(this->get_logger(), "Do Calibration");
        calib_ = true;
    }

    //! \brief skew-symmetric form of a 3-d vector
    //! \param vec 3x1 vector
    //! \return 3x3 matrix
    //!
    Eigen::Matrix3d toSkewSymmetric(const Eigen::Vector3d& vec) {
        Eigen::Matrix3d mat;
        mat <<      0, -vec(2),  vec(1),
                vec(2),       0, -vec(0),
                -vec(1),  vec(0),       0;

        return mat;
    }    

    //! \brief performance the IMU acceleration calibration: 
    //!        1) the orientation between IMU and gravity;
    //!        2) the accel
    //!
    void calibrateAccl()
    {
        // log if needed
        if(log_)
        {
            // prepare the log path
            auto system_time = std::chrono::system_clock::now();
            std::time_t system_time_t = std::chrono::system_clock::to_time_t(system_time);
            auto time_form = std::ctime(&system_time_t);
            log_path_ = "/tmp/calib_accel_" + std::string(time_form) + ".txt";

            // open log path
            file.open(log_path_, std::ios_base::app);//std::ios_base::app
        }

        for(const auto& i: buffer_accl_)
        {
            // Normalize the gravity z-axis that projected into IMU frame:
            //   cosine of inertial frame z-axis (gravity align with z-axis) 
            //   with IMU frame's x-axis,y-axis and x-axis
            Eigen::Vector3d z_I_G = i / i.norm();

            // Normalize the gravity x-axis that projected into IMU frame:
            //    Get x-axis to perpendicular to z-axis
            //    Use [Gram-Schmidt Process](https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process)
            Eigen::Vector3d x_I(1, 0, 0);
            Eigen::Vector3d x_I_G = x_I - z_I_G * z_I_G.transpose() * x_I;
            x_I_G = x_I_G / x_I_G.norm();
            
            // Normalize the gravity y-axis that projected into IMU frame:
            //    Get y from the cross product of these two
            Eigen::Vector3d y_I_G = toSkewSymmetric(z_I_G) * x_I_G;

            // From these axes get rotation
            R_I_G_.block(0, 0, 3, 1) = x_I_G;
            R_I_G_.block(0, 1, 3, 1) = y_I_G;
            R_I_G_.block(0, 2, 3, 1) = z_I_G;

            Eigen::Vector3d ba = i - R_I_G_ * Eigen::Vector3d(0.0,0.0,9.81);
            ba_avg_.x() += ba.x();
            ba_avg_.y() += ba.y();
            ba_avg_.z() += ba.z();

            // log if needed
            if(log_)
            {
                file << ba.transpose()<<std::endl;
            }
        }

        // log if needed
        if(log_)
        {
            file<<"sum bias:\n";
            file<<ba_avg_.transpose()<<std::endl;
            file<<"buffer size:"<<buffer_accl_.size()<<std::endl;
            file.close();             
        }

        ba_avg_  = ba_avg_ / buffer_accl_.size();

        std::cout<<"\n====================\n";
        std::cout<<"Result:\n";
        std::cout<<" R_I_G:\n" << R_I_G_ << std::endl;
        std::cout<<" accl bias:\n" << ba_avg_ << std::endl;
    }

    // Members
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calib_srv_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ROS Parameters
    int max_samples_;
    bool log_;

    // Buffer
    std::vector<Eigen::Vector3d> buffer_accl_;
    std::atomic<bool> save_;
    std::atomic<bool> calib_;

    // Results
    Eigen::Vector3d ba_avg_;
    Eigen::Matrix3d R_I_G_;

    // Log
    std::string log_path_;
    std::ofstream file;    

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuAccCalib>());
    rclcpp::shutdown();
    return 0;
}
