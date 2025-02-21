#ifndef _BASIC_DEV_CPP_
#define _BASIC_DEV_CPP_

#include "basic_dev.hpp"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "basic_dev"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄
    BasicDev go(&n);
    return 0;
}

BasicDev::BasicDev(ros::NodeHandle *nh):transformer()
{  
    //创建图像传输控制句柄
    it = std::make_unique<image_transport::ImageTransport>(*nh); 
    front_left_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));
    front_right_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));

    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
    velcmd.twist.linear.x = 0; //x方向线速度(m/s)
    velcmd.twist.linear.y = 0;//y方向线速度(m/s)
    velcmd.twist.linear.z = 0; //z方向线速度(m/s)

    pwm_cmd.rotorPWM0 = 0.1;
    pwm_cmd.rotorPWM1 = 0.1;
    pwm_cmd.rotorPWM2 = 0.1;
    pwm_cmd.rotorPWM3 = 0.1;

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, std::bind(&BasicDev::pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    gps_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, std::bind(&BasicDev::gps_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&BasicDev::imu_cb, this, std::placeholders::_1));//imu数据
    lidar_suber = nh->subscribe<sensor_msgs::PointCloud2>("airsim_node/drone_1/lidar", 1, std::bind(&BasicDev::lidar_cb, this, std::placeholders::_1));//imu数据

    ros::Subscriber initial_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>(
    "/airsim_node/initial_pose", 1, std::bind(&WorldToBodyTransformer::initialPoseCallback, &transformer, std::placeholders::_1));

    front_left_view_suber = it->subscribe("airsim_node/drone_1/front_left/Scene", 1, std::bind(&BasicDev::front_left_view_cb, this,  std::placeholders::_1));
    front_right_view_suber = it->subscribe("airsim_node/drone_1/front_right/Scene", 1, std::bind(&BasicDev::front_right_view_cb, this,  std::placeholders::_1));
    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");
    //通过publisher实现对无人机的控制
    vel_publisher = nh->advertise<airsim_ros::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 100);
    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("airsim_node/drone_1/rotor_pwm_cmd", 1);
    pose_publisher = nh->advertise<geometry_msgs::PoseStamped>("/pose", 1);

    gps_vins_publisher = nh->advertise<geometry_msgs::PoseStamped>("/gps_vins", 1);

    takeoff_client.call(takeoff); //起飞
    // land_client.call(land); //降落
    // reset_client.call(reset); //重置

    ros::Rate loop_rate(50); // 控制循环频率为10 Hz
    while (ros::ok()) {
        BasicDev::Keycontrol();
        loop_rate.sleep();
        ros::spinOnce();
    }

    // ros::spin();
    // --------------------------------------------------


    // std::thread keyControlThread(&BasicDev::keyControlThread, this);

    // Main ROS loop
    // ros::Rate loop_rate(10);  // 控制循环频率为50 Hz
    // while (ros::ok()) {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    // Join the key control thread when exiting
    // keyControlThread.join();
    // --------------------------------------------------
}

BasicDev::~BasicDev()
{
}

char BasicDev::getKey() 
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    ROS_INFO("Get_Key");
    return ch;
}



void BasicDev::keyControlThread()
{
    while (ros::ok()) {
        char key = getKey();
        if (key != '\0') {
            // Reset velocity
            velcmd.twist.angular.z = 0;
            velcmd.twist.linear.x = 0.0;
            velcmd.twist.linear.y = 0.0;
            velcmd.twist.linear.z = 0.0;
            switch (key) {
                case 'w':
                    velcmd.twist.linear.x = 2.0;  // Forward
                    ROS_INFO("Forward");
                    break;
                case 's':
                    velcmd.twist.linear.x = -2.0;  // Backward
                    ROS_INFO("Backward");
                    break;
                case 'a':
                    velcmd.twist.linear.y = -1.0;  // Left
                    ROS_INFO("Left");
                    break;
                case 'd':
                    velcmd.twist.linear.y = 1.0;  // Right
                    ROS_INFO("Right");
                    break;
                case 'k':
                    velcmd.twist.linear.z = 1.0;  // Down
                    ROS_INFO("Down");
                    break;
                case 'j':
                    velcmd.twist.linear.z = -1.0;  // Up
                    ROS_INFO("Up");
                    break;
                case 'q':
                    velcmd.twist.angular.z = -0.2;
                    ROS_INFO("q");
                    break;
                case 'e':
                    velcmd.twist.angular.z = 0.2;
                    ROS_INFO("r");
                    break;
                default:
                    ROS_WARN("Unknown key pressed: %c", key);
                    break;
            }
            vel_publisher.publish(velcmd);
            ROS_INFO("fly-----------------------------------------------------------");

        }
    }
}

void BasicDev::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    // ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //     eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // body_pose.header = msg->header;  // 保持时间戳等头部信息不变
    // body_pose.pose.position.x = msg->pose.position.x - 547.7333699682529;
    // body_pose.pose.position.y = msg->pose.position.y - 518.6191855150987;
    // body_pose.pose.position.z = -(msg->pose.position.z + 30.38767972588539);

    // // 将欧拉角转换回四元数并设置到消息中
    // Eigen::Quaterniond new_q = Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()) *
    //                             Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()) *
    //                             Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX());
    
    // body_pose.pose.orientation.w = new_q.w();
    // body_pose.pose.orientation.x = new_q.x();
    // body_pose.pose.orientation.y = new_q.y();
    // body_pose.pose.orientation.z = new_q.z();

    // 发布消息
    // pose_publisher.publish(body_pose);

    body_pose = transformer.transformToBodyFrame(*msg);
    pose_publisher.publish(body_pose);
}

void BasicDev::gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    ROS_INFO("Get gps data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // body_pose = transformer.transformToBodyFrame(*msg);

    // gps_vins.position.x = msg->pose.position.x;
    // gps_vins.position.y = msg->pose.position.y;
    // gps_vins.position.z = msg->pose.position.z;

    // // 填充姿态信息（使用转换后的欧拉角）
    // gps_vins.orientation = msg->pose.orientation;


    // // 发布 Pose 消息
    // gps_vins_publisher.publish(gps_vins);
    
    body_gps_pose = transformer.transformToBodyFrame(*msg);

    // 发布 Pose 消息
    gps_vins_publisher.publish(body_gps_pose);
}

void BasicDev::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("Get imu data. time: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
}

void BasicDev::front_left_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);

    velcmd.twist.linear.x = 2.0;
    vel_publisher.publish(velcmd);

    // if(!cv_front_left_ptr->image.empty())
    // {
    //     ROS_INFO("Get front left image.: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    // }
}

void BasicDev::front_right_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    // if(!cv_front_right_ptr->image.empty())
    // {
    //     ROS_INFO("Get front right image.%f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    // }
}

void BasicDev::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pts);
    // ROS_INFO("Get lidar data. time: %f, size: %ld", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9, pts->size());
}

#endif