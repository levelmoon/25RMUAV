# 25RM无人机启动接口
# 首先修改模拟器路径下settings.json
`IntelligentUAVChampionshipSimulator` (定位延迟关键)
docker 到时候也需要修改，具体参考官方github
## vins_fusion （vins cmakelists参照官方的）
##### 启动路径 
`VINS-Fusion/vins_estimator/launch/vins_run.launch`
##### 话题说明
```
/vins_estimator/imu_propagate 为imu融合输出
/globalEstimator/global_odometry 为gps融合输出
以上全部已经进行了坐标变换
```
##### config文件
```
路径：VINS-Fusion/config/rmua25/rmua25.yaml
issue: 无人机过久不动定位会飘飞，实测是imu的问题，未进行准确标定
```

## basic_dev
##### 启动说明
`rosrun basic_dev basic_dev`
##### 功能 
`键盘控制节点，无人机起飞点终端输出（如：3），全局坐标转换节点
（注意：在上传为docker时，注释掉键盘控制节点）`
##### 话题说明
```
/pose 对gps进行了坐标变换，为与vins进行融合的全局坐标
```

## controller 控制接口 
##### 启动说明（可将launch 中的imu_gps_odometry 注释，这是debug用的）
`roslaunch controller_test controller_test.launch`
##### 功能 
`接收/globalEstimator/global_odometry定位，对无人机进行控制`
##### 控制接口 
```
X_des << 0, 0, 1, 0, 0, 0, 1, 0, 0.5, 0, 0, 0; //还需要写个坐标变换
issue：无人机的定位是全局的，这个接口也是全局的，也就是，假如我无人机向左转20度，用这个接口发送向前5m的命令，无人机是全局位置向前5m,还需进行机体坐标系的转换


这是每个控制接口含义，前三个为x,y,z
std::cout << "X_real = ["

<< Twb(0, 3) << ", " << Twb(1, 3) << ", " << Twb(2, 3) << ",\n"

<< msg->twist.twist.linear.x << ", " << -msg->twist.twist.linear.y << ", " << -msg->twist.twist.linear.z << ",\n"

<< "phi: " << phi << " theta: " << theta << " psi: " << psi << ",\n"

<< msg->twist.twist.angular.x << ", " << msg->twist.twist.angular.y << ", " << msg->twist.twist.angular.z << ",\n"

<< msg->pose.pose.orientation.w << ", " << msg->pose.pose.orientation.x << ", " << msg->pose.pose.orientation.y << ", " << msg->pose.pose.orientation.z<< "\n"

<< "]" << std::endl;
```
##### 注意
```
// 控制回调函数的调用频率，确保每隔一定数量的消息才进行实际的计算
// cb_cnt ++;
// if(cb_cnt / 100 < 10)return;

// 判断限制动力，不发布数据会悬停
if(X_real[0]<30.0) 

// const float phi = std::asin(T0flub(2, 1)); //pitch (180度数值为3.16)

// const float theta = std::atan2(-T0flub(2, 0)/std::cos(phi), T0flub(2, 2)/std::cos(phi)); //roll

// const float psi = std::atan2(-T0flub(0, 1)/std::cos(phi), T0flub(1, 1)/std::cos(phi)); //yaw



以上注释重要，其他注释可删，都是debug用的
```

## odometry（不使用，debug使用）

## 其他
end_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)可以算出终点的标号。
这样就可以直接在中央广场直接寻找对应标号数字。
具体参考`basic_dev/src/world_to_body_transformer.cpp`
```
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
```

