#include "world_to_body_transformer.hpp"

void WorldToBodyTransformer::loadOriginFrames() {
    // 初始化12个坐标系数据
    origin_frames_.push_back({
        "1",
        Eigen::Vector3d(-7.797241474690959e-05, -1.0070796533909049e-05, 0.3251775801181793),
        Eigen::Quaterniond(0.9999999999999996, 0.0, 0.0, -2.98023259404094e-08).normalized()
    });

    origin_frames_.push_back({
        "2",
        Eigen::Vector3d(143.5283167693185, 286.6593686388817, -8.07265317440033),
        Eigen::Quaterniond(0.9994145065181568, 0.0, 0.0, -0.03421467756781126).normalized()
    });

    origin_frames_.push_back({
        "3",
        Eigen::Vector3d(547.7333699682529, 518.6191855150987, -30.38767781853676),
        Eigen::Quaterniond(0.9667845877188579, 0.0, 0.0, -0.255592568259091).normalized()
    });

    origin_frames_.push_back({
        "4",
        Eigen::Vector3d(1349.795011657175, -103.3772185898915, -7.438585817813873),
        Eigen::Quaterniond(-0.023956627966215706, 0.0, 0.0, 0.9997129988034007).normalized()
    });

    origin_frames_.push_back({
        "5",
        Eigen::Vector3d(1176.0722936566176, -424.10053753646343, -77.45748656988144),
        Eigen::Quaterniond(0.03092763053436549, 0.0, 0.0, 0.9995216264141211).normalized()
    });

    origin_frames_.push_back({
        "6",
        Eigen::Vector3d(783.4068348079065, -675.3972814817362, -2.921000897884369),
        Eigen::Quaterniond(0.28071304536580544, 0.0, 0.0, 0.9597917410362808).normalized()
    });

    origin_frames_.push_back({
        "7",
        Eigen::Vector3d(2.0145176513944563, -287.5242436395712, -3.6605679392814636),
        Eigen::Quaterniond(0.9883275620915748, 0.0, 0.0, 0.15234378887937736).normalized()
    });

    origin_frames_.push_back({
        "8",
        Eigen::Vector3d(713.3507769458574, -728.138444094561, -14.86404338479042),
        Eigen::Quaterniond(0.7701665379781655, 0.0, 0.0, 0.6378428519460941).normalized()
    });

    origin_frames_.push_back({
        "9",
        Eigen::Vector3d(1032.6203946952767, -483.9575003948862, -57.436096370220184),
        Eigen::Quaterniond(0.6490074332249746, 0.0, 0.0, 0.7607820657841049).normalized()
    });

    origin_frames_.push_back({
        "10",
        Eigen::Vector3d(1327.7289582635153, 153.8401840529607, -35.309341967105865),
        Eigen::Quaterniond(-0.15684482075275696, 0.0, 0.0, 0.9876232592456677).normalized()
    });

    origin_frames_.push_back({
        "11",
        Eigen::Vector3d(676.7573854358187, 539.0798705915453, -47.691846042871475),
        Eigen::Quaterniond(-0.3471278030897661, 0.0, 0.0, 0.9378178332288593).normalized()
    });

    origin_frames_.push_back({
        "12",
        Eigen::Vector3d(264.47128897902326, 397.3294812893412, 0.026209861040115356),
        Eigen::Quaterniond(0.7828287004342174, 0.0, 0.0, -0.6222372744994262).normalized()
    });
}

WorldToBodyTransformer::WorldToBodyTransformer() : nh_(""), origin_detected_(false) {
    loadOriginFrames();
    ROS_WARN_THROTTLE(1, "2222---------------------------------------");
    ros::Subscriber sub = nh_.subscribe("/airsim_node/initial_pose", 1, 
        &WorldToBodyTransformer::initialPoseCallback, this);
    TWfluWned << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
}

void WorldToBodyTransformer::initialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    Eigen::Vector3d current_pos(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
    );

    double min_distance = std::numeric_limits<double>::max();
    OriginFrame nearest_frame;

    for (const auto& frame : origin_frames_) {
        double distance = (current_pos - frame.position).norm();
        if (distance < min_distance) {
            min_distance = distance;
            nearest_frame = frame;
        }
    }

    if (min_distance <= 5.0) {
        current_origin_position_ = nearest_frame.position;
        current_origin_orientation_ = nearest_frame.orientation;
        origin_detected_ = true;
        ROS_INFO_STREAM("Detected takeoff from frame: " << nearest_frame.frame_id);
    } else {
        origin_detected_ = false;
        ROS_WARN("No valid origin frame found within 5m radius");
    }
}

geometry_msgs::PoseStamped WorldToBodyTransformer::transformToBodyFrame(const geometry_msgs::PoseStamped& world_pose) {
    geometry_msgs::PoseStamped body_pose = world_pose;

    if (!origin_detected_) {
        ROS_WARN_THROTTLE(1, "Origin frame not detected yet");
        return body_pose;
    }
    Eigen::Vector3d ned_position = TWfluWned * world_position;
    // 位置变换
    Eigen::Vector3d world_position(
        world_pose.pose.position.x,
        world_pose.pose.position.y,
        world_pose.pose.position.z
    );
    
    Eigen::Vector3d relative_position = world_position - current_origin_position_;
    Eigen::Vector3d body_position = current_origin_orientation_.conjugate() * relative_position;

    // 姿态变换
    Eigen::Quaterniond world_orientation(
        world_pose.pose.orientation.w,
        world_pose.pose.orientation.x,
        world_pose.pose.orientation.y,
        world_pose.pose.orientation.z
    );
    
    Eigen::Quaterniond body_orientation = current_origin_orientation_.conjugate() * world_orientation;

    // 更新输出位姿
    body_pose.pose.position.x = body_position.x();
    body_pose.pose.position.y = body_position.y();
    body_pose.pose.position.z = body_position.z();
    
    body_pose.pose.orientation.x = body_orientation.x();
    body_pose.pose.orientation.y = body_orientation.y();
    body_pose.pose.orientation.z = body_orientation.z();
    body_pose.pose.orientation.w = body_orientation.w();

    return body_pose;
}