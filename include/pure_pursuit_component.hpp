#ifndef PURE_PURSUIT_COMPONENT_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace pure_pursuit
{

template <typename T1, typename T2> double distance(T1 pt1, T2 pt2) {
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) +
              pow(pt1.z - pt2.z, 2));
}

class PurePursuit : public rclcpp::Node
{
private:
    double ld_gain_;
    double ld_;
    double min_ld_;
    double car_wheel_base_;
    double alpha_;
    double car_speed_;

    int controller_freq_;
    int point_idx_;
    int last_p_idx_;
    
    double last_dist_ = std::numeric_limits<double>::infinity();
    
    bool got_path_ = false;
    bool path_done_ = true;
    bool loop_ = false;
    
    std::string map_frame_;
    std::string base_frame_;
    //ros::Time last_msg_time_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    
    geometry_msgs::msg::PoseStamped target_point_;
    ackermann_msgs::msg::AckermannDriveStamped control_msg_;
    geometry_msgs::msg::PointStamped lookahead_p;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr l_point_pub_;
  
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void odom_clk_(const nav_msgs::msg::Odometry &msg)
    {
        RCLCPP_INFO(this->get_logger(), "odom_clk_");

        this->car_speed_ = msg.twist.twist.linear.x;
        this->ld_ = std::max(this->ld_gain_ * this->car_speed_, this->min_ld_);
    }

    void path_clk_(const nav_msgs::msg::Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "path_clk_");

        path_ = msg->poses;
        // path_.push_back(msg->poses[0]);
        got_path_ = true;
        path_done_ = false;
        point_idx_ = 0;
        double start_end_dist =
            distance(path_[0].pose.position, path_.back().pose.position);
        RCLCPP_INFO(this->get_logger(), "Start to End Distance: %f", start_end_dist);
        RCLCPP_INFO(this->get_logger(), "Min lookup distance: %f", min_ld_);
        if (start_end_dist <= min_ld_) {
            loop_ = true;
            RCLCPP_INFO(this->get_logger(), "Is Loop: True");
        }
    }

    void timer_clk_()
    {
        RCLCPP_INFO(this->get_logger(), "timer_clk_");

        double y_t = 0, ld_2 = 0, delta = 0;
        double distance_ = 0;

        if (got_path_) 
        {
            // get the current robot location by tf base_link -> map
            // iterate over the path points
            // if the distance between a point and robot > lookahead break and take
            // this point transform this point to the robot base_link the y component
            // of this point is y_t delta can be computed as atan2(2 * yt * L_, ld_2)
            try 
            {
                /*base_location_ = this->tfBuffer_.lookupTransform(
                    map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));*/
                geometry_msgs::msg::TransformStamped base_location = this->tf_buffer_->lookupTransform(
                    this->map_frame_, this->base_frame_,
                    tf2::TimePointZero);

                for (; point_idx_ < path_.size(); point_idx_++) 
                {
                    distance_ = distance(path_[point_idx_].pose.position,
                                    base_location.transform.translation);
                
                    RCLCPP_INFO(this->get_logger(), "Point ID: %d, Distance %f", point_idx_, distance_);
                
                    if (distance_ >= ld_) 
                    {
                        path_[point_idx_].header.stamp =
                            this->get_clock()->now(); // Set the timestamp to now for the transform
                                            // to work, because it tries to transform the
                                            // point at the time stamp of the input point
                       /* tf_buffer_->transform(path_[point_idx_], target_point_, base_frame_,
                                            
                                            rclcpp::Duration::from_seconds(1.0)); //ros::Duration(0.1));
                        */
                       
                       //this->tf_buffer_->transform(path_[point_idx_], target_point_, 
                       //     base_frame_, tf2::durationFromSec(0.1));
                       //
                       //auto transformStampedENU = tf_buffer_.lookupTransform(AIRSIM_FRAME_ID, vehicle_name, ros::Time(0), ros::Duration(1));
                //tf2::doTransform(lidar_msg, lidar_msg_enu, transformStampedENU);
                       tf2::doTransform(path_[point_idx_], target_point_, base_location);
                       break;
                    }
                }

                // Calculate the steering angle
                ld_2 = ld_ * ld_;
                y_t = target_point_.pose.position.y;
                delta = atan2(2 * car_wheel_base_ * y_t, ld_2);
                control_msg_.drive.steering_angle = delta;
                control_msg_.drive.speed = 2;
                control_msg_.header.stamp = this->get_clock()->now(); //ros::Time::now();
                control_pub_->publish(control_msg_);

                last_p_idx_ = point_idx_;
                last_dist_ = distance_;
                if (point_idx_ == path_.size() && loop_) 
                {
                    point_idx_ = 0;
                } 
                else if (point_idx_ == path_.size()) 
                {
                    RCLCPP_INFO(this->get_logger(),"Reached final point");
                    control_msg_.drive.steering_angle = 0;
                    control_msg_.drive.speed = 0;
                    control_msg_.header.stamp = this->get_clock()->now();
                    control_pub_->publish(control_msg_);
                    got_path_ = false;
                    point_idx_ = 0;
                }
                lookahead_p.point = path_[point_idx_].pose.position;
                lookahead_p.header = path_[point_idx_].header;
                l_point_pub_->publish(lookahead_p); // Publish the lookahead point
            } 
            catch (tf2::TransformException &ex) 
            {
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            }
        }
    }

public:
    explicit PurePursuit(const rclcpp::NodeOptions &options)
         : Node("pure_pursuit", options)
    {
        /*ros::NodeHandle nh_;
        ros::NodeHandle nh_private("~");
        */
       
        // Node paramters
        /*nh_private.param<double>("ld_gain", ld_gain_, 1.0);
        nh_private.param<double>("min_ld", min_ld_, 0.5);
        nh_private.param<double>("car_wheel_base", car_wheel_base_, 0.44);
        nh_private.param<int>("controller_freq", controller_freq_, 10);
        nh_private.param<std::string>("map_frame", map_frame_, "map");
        nh_private.param<std::string>("base_frame", base_frame_, "base_link");
        */
       
        this->ld_gain_ = this->declare_parameter<double>("ld_gain", 1.0);
        this->min_ld_ = this->declare_parameter<double>("min_ld", 0.5);
        this->car_wheel_base_ = this->declare_parameter<double>("car_wheel_base", 0.44);
        this->controller_freq_ = this->declare_parameter<int>("controller_freq", 10);
        this->map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        this->base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

        this->ld_ = this->min_ld_;

        //ld_ = min_ld_;
        // publishers
        this->control_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/pure_pursuit/control", 1);

        this->l_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/pure_pursuit/lookahead_point", 1);

        // subscribers
        this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, 
            std::bind(&PurePursuit::odom_clk_, this, std::placeholders::_1));
        
        this->path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/pure_pursuit/path", 1, 
            std::bind(&PurePursuit::path_clk_, this, std::placeholders::_1));
        
        /*ros_rate_ = new ros::Rate(controller_freq_);
        // Publishers and subscribers
        control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
            "/pure_pursuit/control", 1);
        ros::Subscriber odom_sub_ =
            nh_.subscribe("/odom", 1, &PurePursuit::odom_clk_, this);
        ros::Subscriber path_sub_ =
            nh_.subscribe("/pure_pursuit/path", 1, &PurePursuit::path_clk_, this);
        */
        this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_));

        // main loop
        //control_loop_();
        this->timer_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);  
        this->timer_ = this->create_wall_timer(std::chrono::seconds(1)/this->controller_freq_, 
            std::bind(&PurePursuit::timer_clk_, this),
            this->timer_cb_group_);
    }


};

} // namespace pure_pursuit

#endif // PURE_PURSUIT_COMPONENT_HPP