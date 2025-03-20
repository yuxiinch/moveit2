//路徑規劃並執行
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node "cartesian_path"
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("cartesian_path");

  // Create a ROS logger to log messages
  auto const logger = rclcpp::get_logger("cartesian_path");

  // We spin up a SingleThreadedExecutor（單執行緒） to get current pose of the robot later（確保即時獲得）
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  // 用spinner背景執行緒,確保可以持續處理 ROS2 事件
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt Move Group Interface for panda arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "small_arm");  //控制"small_arm"的機器人機械手臂

  //x - forward(+) and backward(-)
  //y - left(+) and right(-)
  //z - up(+) and down(-)

  //建立一個 vector 容器 來存放機械手臂的路徑方式點。
  // Variable to hold waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  // Current pose 
  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose; 

  //================================================================================================
  // Variable for first target pose waypoint(Cartesian 路徑的起點)
  geometry_msgs::msg::Pose target_pose = start_pose;
  // Move diagonally  //控制每軸移動
  
  // target_pose.position.x -= 0.3; //Forward
  target_pose.position.z -= 0.4; //Up

  // Add target pose to waypoints
  waypoints.push_back(target_pose);  
  //================================================================================================

  // Variable for next target pose
  geometry_msgs::msg::Pose target_pose2 = target_pose;

  // Move only along one axis
  target_pose2.position.y -= 0.1; //Right

  // Add next target pose to waypoints
  waypoints.push_back(target_pose2);
  //================================================================================================

  //最大步長為1cm
  //設定零跳躍閾值，防止不合理的路徑變化
  // We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as the max step in Cartesian translation
  // We will specify the jump threshold as 0.0, effectively disabling it
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  

  //computeCartesianPath 會嘗試規劃一條通過所有方式點的路徑
  //trajectory 會存放計算出的軌跡
  //fraction(完整度-0~1)
  // Computing the Cartesian path, which is stored in trajectory
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step,   jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing Cartesian path plan (%.2f%% achieved)", fraction * 100.0);


  //路徑規劃完成後,檢查是否可以執行路徑,然後執行路徑trajectory
  // Check if complete path is possible and execute the trajectory
  if(fraction == 1){
    move_group_interface.execute(trajectory);
  }
  
  // Shutdown
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
