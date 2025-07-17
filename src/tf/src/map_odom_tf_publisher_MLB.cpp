#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <proj.h>
#include <Eigen/Geometry>

class MapOdomTFPublisherStatic : public rclcpp::Node
{
public:
    MapOdomTFPublisherStatic()
        : Node("map_odom_tf_static_node")
    {
        tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "ublox_gps_node/fix", 10, std::bind(&MapOdomTFPublisherStatic::callback_gps, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/navsat", 10, std::bind(&MapOdomTFPublisherStatic::callback_odom, this, std::placeholders::_1));

        dx_ = 0.0;
        dy_ = 0.0;

        // 기준점 (K-City BS)
        map_lat_ = 37.2388925;
        map_lon_ = 126.77293309999999;

        threshold_ = 0.05;

        // ✅ Molodensky-Badekas 변환 설정 (Proj7 이상)
        const char *helmert_def = 
            "+proj=helmert +x=-115.8 +y=-54.3 +z=-79.6 "
            "+rx=-0.000001004 +ry=-0.000001887 +rz=-0.000005103 "
            "+s=-0.00000001116 "
            "+convention=coordinate_frame";

        P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:2097", nullptr);
        if (P == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create projection.");
            throw std::runtime_error("Failed to create projection.");
        }

        P_helmert = proj_create(PJ_DEFAULT_CTX, helmert_def);
        if (P_helmert == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create Helmert transformation.");
            throw std::runtime_error("Failed to create Helmert transformation.");
        }

        PJ_COORD origin = proj_coord(map_lat_, map_lon_, 0, 0);
        PJ COORD transformed = proj_trans(P_helmert, PJ_FWD, origin);
        PJ_COORD result = proj_trans(P, PJ_FWD, transformed);
        map_x_ = result.xy.x;
        map_y_ = result.xy.y;
    }

    ~MapOdomTFPublisherStatic()origin
    {
        if (P)
        {
            proj_destroy(P);
        }
        if (P_helmert)
        {
            proj_destroy(P_helmert);
        }
    }

    void publish_tf()
    {
        if (!odom_)
            return;

        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.frame_id = "map";
        tf_msg.header.stamp = odom_->header.stamp;
        tf_msg.child_frame_id = "odom";

        double trans_x = dx_;
        double trans_y = dy_;

        tf_msg.transform.translation.x = trans_y;
        tf_msg.transform.translation.y = trans_x;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = 0.;
        tf_msg.transform.rotation.y = 0.;
        tf_msg.transform.rotation.z = 0.;
        tf_msg.transform.rotation.w = 1.;

        tf_publisher_->sendTransform(tf_msg);
    }

private:
    void callback_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (dx_ == 0.0 && dy_ == 0.0)
        {
            if (msg->position_covariance[0] < threshold_ && msg->position_covariance[4] < threshold_)
            {
                PJ_COORD latlon = proj_coord(msg->latitude, msg->longitude, 0, 0);
                
                // ✅ Molodensky-Badekas 변환 적용
                PJ_COORD transformed = proj_trans(P_helmert, PJ_FWD, latlon);
                PJ_COORD xy = proj_trans(P, PJ_FWD, transformed);
                
                dx_ = xy.xy.x - map_x_;
                dy_ = xy.xy.y - map_y_;
                
                RCLCPP_INFO(this->get_logger(), "Map origin: x = %f, y = %f", map_x_, map_y_);
                RCLCPP_INFO(this->get_logger(), "odom origin: x = %f, y = %f", xy.xy.x, xy.xy.y);
                RCLCPP_INFO(this->get_logger(), "Origin calculated: dx = %f, dy = %f", dx_, dy_);
            }
        }
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    PJ *P;
    PJ *P_helmert;  // Molodensky-Badekas 변환용
    double map_lat_, map_lon_;
    double map_x_, map_y_;
    double dx_, dy_;
    double threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOdomTFPublisherStatic>();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->publish_tf();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
