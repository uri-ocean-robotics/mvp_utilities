#include <vector>
#include <atomic>
#include <Eigen/Eigen>
#include <fstream>
#include <chrono>
#include <ctime>  

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

class CalibAccel {
public:

    //! \brief the constructor: ros topic/service setup, param loading
    //! \param nh the node handler
    //! \param pnh the private node handler
    //! 
    CalibAccel(ros::NodeHandle nh, ros::NodeHandle pnh): 
        nh_(nh), pnh_(pnh), save_(false), calib_(false)
    {
        Eigen::Vector3d ba_avg_ = Eigen::Vector3d::Zero();
        Eigen::Matrix3d R_I_G_ = Eigen::Matrix3d::Zero();

        sub_ = nh_.subscribe("/imu_input", 100, &CalibAccel::imuCallback, this);

        start_srv_ = pnh_.advertiseService("start_sampling", &CalibAccel::startCallback, this);

        stop_srv_ = pnh_.advertiseService("stop_sampling", &CalibAccel::stopCallback, this);

        calib_srv_ = pnh_.advertiseService("calibrate_accl", &CalibAccel::calibCallback, this);

        pnh_.param<int>("max_samples", max_samples_, 100);
        pnh_.param<bool>("log", log_, false);

    }

    //! \brief IMU callback: store the IMU acceleration and do calibration if needed
    //! \param msg IMU msg
    //! 
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

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
                ROS_WARN("Larger then the max %d, drop the oldest 100", max_samples_);
            }

            // report the size of buffer
            if(buffer_accl_.size() % 100 == 0)
            {
                ROS_INFO("Got %ld acceleration readings", buffer_accl_.size());
                ROS_INFO("IMU accel example: %f, %f, %f", 
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
            // buffer_accl_.clear();
        }

        // publish tf
        if(!R_I_G_.isZero() && !ba_avg_.isZero())
        {

            static tf::TransformBroadcaster br;
            
            tf::Matrix3x3 mat;
            tf::matrixEigenToTF(R_I_G_.transpose(), mat);
            tf::Transform transform = tf::Transform(mat, tf::Vector3(0.0,0.0,0.0));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu_calib"));
        }
    }

    //! \brief service callback to start saving IMU acceleration into buffer
    //! 
    bool startCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        save_ = true;
        ROS_INFO("Start collect acceleration");
        return true;
    }


    //! \brief service callback to stop saving IMU acceleration into buffer
    //!
    bool stopCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        save_ = false;
        ROS_INFO("Stop collect acceleration");
        return true;
    }

    //! \brief service callback to performance the calibration
    //! 
    bool calibCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        calib_ = true;
        ROS_INFO("Do Calibration");
        return true;
    }

private:
    // ROS related 
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_;
    ros::ServiceServer start_srv_;
    ros::ServiceServer stop_srv_;
    ros::ServiceServer calib_srv_;

    // Parameters
    int max_samples_;
    bool log_;
    std::string log_path_;
    std::ofstream file;

    // Buffer
    std::vector<Eigen::Vector3d> buffer_accl_;
    std::atomic<bool> save_;
    std::atomic<bool> calib_;

    Eigen::Vector3d ba_avg_;
    Eigen::Matrix3d R_I_G_;

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

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "CalibAccel");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    CalibAccel node(nh, pnh);
    ros::spin();
}