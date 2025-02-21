#ifndef WORLD_TO_BODY_TRANSFORMER_HPP
#define WORLD_TO_BODY_TRANSFORMER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <limits>

class WorldToBodyTransformer {
private:
    struct OriginFrame {
        std::string frame_id;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    std::vector<OriginFrame> origin_frames_;
    Eigen::Vector3d current_origin_position_;
    Eigen::Quaterniond current_origin_orientation_;
    bool origin_detected_;


    ros::NodeHandle nh_;
    Eigen::Matrix3d TWfluWned;

    void loadOriginFrames();

public:
    WorldToBodyTransformer();
    void initialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped transformToBodyFrame(const geometry_msgs::PoseStamped& world_pose);
};

#endif // WORLD_TO_BODY_TRANSFORMER_HPP