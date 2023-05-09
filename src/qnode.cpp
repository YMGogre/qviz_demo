/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/qviz_demo/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace class1_ros_qt_demo {

/*****************************************************************************
** Implementation实现
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
  init_argv(argv),
  buffer(ros::Duration(10))
  {}

//定义静态成员变量
double QNode::clicked_point[2]{0.0, 0.0};

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

/**
 * @brief 使用本地环境变量进行连接
 * @retval 连接成功返回true，否则返回false
 * @attention 所有的操作在两个init方法中都要写一遍
 */
bool QNode::init() {
  //指定名称初始化一个ROS节点
	ros::init(init_argc,init_argv,"class1_ros_qt_demo");
	if ( ! ros::master::check() ) {
    //连接master失败直接返回false
		return false;
	}
  //启动节点
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  //创建节点处理句柄
  ros::NodeHandle n;

  // Add your ros communications here.
  /************
  *   发布者   *
  ************/
  //创建点击点话题发布者
  clicked_point_publisher = n.advertise<geometry_msgs::PointStamped>("clicked_point", 1000);
  //创建标记话题发布者
  marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  //创建标准点话题发布者
  standard_points_publisher = n.advertise<visualization_msgs::Marker>("standard_point", 1000);
  //创建路径话题发布者
  global_plan_publisher = n.advertise<nav_msgs::Path>("global_plan", 1000);
  //创建任务路径预览话题发布者
  task_path_preview_publisher = n.advertise<nav_msgs::Path>("/task_path_preview", 1000);
  //创建任务路径话题发布者
  task_path_publisher = n.advertise<nav_msgs::Path>("/task_path", 1000);

  /************
  *   订阅者   *
  ************/
  //创建标记点位话题订阅者
  clicked_point_sub = n.subscribe("/clicked_point", 1000, &QNode::clicked_point_callback, this);
  //创建sick_safetyscanners/scan1话题订阅者
  laserScan1Sub_ = n.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners/scan1", 1, &QNode::laser1Callback, this);  //本项目未用到
  //创建sick_safetyscanners/scan2话题订阅者
  laserScan2Sub_ = n.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners/scan2", 1, &QNode::laser2Callback, this);  //本项目未用到
  //创建map话题订阅者
  mapSub = n.subscribe("/map", 1000, &QNode::map_callback, this);

  //启动多线程，调用run()方法
	start();
	return true;
}

/**
 * @brief 使用参数列表提供的url进行连接
 * @param const std::string &master_url  ：主机的url地址字符串常量
 * @param const std::string &host_url    ：本机的url地址字符串常量
 * @retval 连接成功返回true，否则返回false
 */
bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"class1_ros_qt_demo");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
  /************
  *   发布者   *
  ************/
  //创建点击点话题发布者
  clicked_point_publisher = n.advertise<geometry_msgs::PointStamped>("clicked_point", 1000);
  //创建标记话题发布者
  marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  //创建标准点话题发布者
  standard_points_publisher = n.advertise<visualization_msgs::Marker>("standard_point", 1000);
  //创建路径话题发布者
  global_plan_publisher = n.advertise<nav_msgs::Path>("global_plan", 1000);
  //创建任务路径预览话题发布者
  task_path_preview_publisher = n.advertise<nav_msgs::Path>("/task_path_preview", 1000);
  //创建任务路径话题发布者
  task_path_publisher = n.advertise<nav_msgs::Path>("/task_path", 1000);


  /************
  *   订阅者   *
  ************/
  //创建标记点位话题订阅者
  clicked_point_sub = n.subscribe("/clicked_point", 1000, &QNode::clicked_point_callback, this);
  //创建sick_safetyscanners/scan1话题订阅者
  laserScan1Sub_ = n.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners/scan1", 1, &QNode::laser1Callback, this);  //本项目未用到
  //创建sick_safetyscanners/scan2话题订阅者
  laserScan2Sub_ = n.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners/scan2", 1, &QNode::laser2Callback, this);  //本项目未用到
  //创建map话题订阅者
  mapSub = n.subscribe("/map", 1000, &QNode::map_callback, this);

  //启动多线程，调用run()方法
	start();
	return true;
}

/**
 * @brief 重写覆盖父类(QThread类)的run()方法，在其中实现耗时操作以避免堵塞UI线程
 */
void QNode::run() {
  //循环频率
  ros::Rate loop_rate(20);   //20Hz

  /****************
  *   TF监听容器   *
  ****************/
  tf2_ros::TransformListener listener(buffer);
  log(Info,std::string("成功连接至ROS Master"));

//  ros::AsyncSpinner spinner(4);
//  spinner.start();
	while ( ros::ok() ) {
    get_curr_robot_pose(global_pose, buffer);
    //ROS事件循环(查询回调函数队列)
    ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/**
 * @brief 获取ROS各类输出流信息并打印
 * @param const LogLevel &level  ：打印信息类别枚举对象引用
 * @param const std::string &msg ：待打印的消息字符串常量
 */
void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  if(ros::isStarted())
  {
    switch ( level ) {
      case(Debug) : {
          ROS_DEBUG_STREAM(msg);
          logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
          break;
      }
      case(Info) : {
          ROS_INFO_STREAM(msg);
          logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
          break;
      }
      case(Warn) : {
          ROS_WARN_STREAM(msg);
          logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
          break;
      }
      case(Error) : {
          ROS_ERROR_STREAM(msg);
          logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
          break;
      }
      case(Fatal) : {
          ROS_FATAL_STREAM(msg);
          logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
          break;
      }
    }
  }
  else
  {
    logging_model_msg << "[INFO] 请先连接ROS Master";
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}



/**
 * @brief 发布clicked_point话题槽函数
 * @param double x  ———— X坐标
 * @param double y  ———— Y坐标
 */
void QNode::slot_publish_clicked_point(double x, double y)
{
  geometry_msgs::PointStamped point;
  //设置frame
  point.header.frame_id = "map";
  //设置时间戳
  point.header.stamp = ros::Time::now();
  point.point.x = x;
  point.point.y = y;
  point.point.z = 0;    //由于显示二维地图，所以设置z为0

  clicked_point_publisher.publish(point);
}

/**
 * @brief /clicked_point话题订阅者的回调函数，将收到的消息转发至/Visualization_marker话题
 */
void QNode::clicked_point_callback(const geometry_msgs::PointStamped &point_with_stamp)
{
  //为静态成员变量赋值
  clicked_point[0] = point_with_stamp.point.x;
  clicked_point[1] = point_with_stamp.point.y;
  _isClicked = true;

  if(isGenerateTask)
  {
    //设置frame(marker)
    points.header.frame_id = "map";
    //设置时间戳
    points.header.stamp = ros::Time::now();
    //设置动作
    points.action = visualization_msgs::Marker::ADD;
    //设置类型
    points.type = visualization_msgs::Marker::POINTS;
    //设置点的宽和高
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    //设置点的颜色
    points.color.g = 1.0f;
    points.color.a = 1.0;

    //设置frame(path)
    path.header.frame_id = "map";
    //设置时间戳
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_with_stamp;

    if(isUsingStandardPoint){
      StandardPoint nearestPoint = linear_nearest_neighbor_search(point_with_stamp.point.x, point_with_stamp.point.y);   //线性搜索求最近邻
      geometry_msgs::Point temp;
      temp.x = nearestPoint.x();
      temp.y = nearestPoint.y();
      points.points.push_back(temp);                    //最近邻标准点入栈
      pose_with_stamp.pose.position = temp;             //最近邻标准点作为path的PoseStamped的position
    }
    else{
      points.points.push_back(point_with_stamp.point);  //clicked_point点入栈
      pose_with_stamp.pose.position = point_with_stamp.point;   //clicked_point点作为path的PoseStamped的position
    }
    path.poses.push_back(pose_with_stamp);

    emit pointClicked();
    marker_publisher.publish(points);     //发布marker
    global_plan_publisher.publish(path);  //发布path
  }
}

/**
 * @brief 发布"global_plan"话题槽函数
 */
nav_msgs::Path QNode::slot_publish_global_plan()
{
  global_plan_publisher.publish(path);
  //返回path
  return path;
}



/**
 * @brief /map话题订阅者的回调函数
 */
void QNode::map_callback(const nav_msgs::OccupancyGrid& occupancyGrid)
{
  emit mapOpened();
}

/**
 * @brief sick_safetyscanners/scan1话题订阅者的回调函数
 * @attention 在本项目中，该函数并未使用到
 */
void QNode::laser1Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laserPoint1.header= msg->header;
  laserPoint1.angle_min = msg->angle_min;
  laserPoint1.angle_max = msg->angle_max;
  laserPoint1.angle_increment = msg->angle_increment;
  laserPoint1.time_increment = msg->time_increment;
  laserPoint1.scan_time = msg->scan_time;
  laserPoint1.ranges = msg->ranges;
}

/**
 * @brief sick_safetyscanners/scan2话题订阅者的回调函数
 * @attention 在本项目中，该函数并未使用到
 */
void QNode::laser2Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laserPoint2.header= msg->header;
  laserPoint2.angle_min = msg->angle_min;
  laserPoint2.angle_max = msg->angle_max;
  laserPoint2.angle_increment = msg->angle_increment;
  laserPoint2.time_increment = msg->time_increment;
  laserPoint2.scan_time = msg->scan_time;
  laserPoint2.ranges = msg->ranges;
}

/**
 * @brief 获取当前机器人位置方法
 * @param geometry_msgs::PoseStamped& global_pose ———— 全局位置变量
 * @param tf2_ros::Buffer& tf_                    ———— TF容器
 */
bool QNode::get_curr_robot_pose(geometry_msgs::PoseStamped& global_pose, tf2_ros::Buffer& tf_)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = ros::Time();
  ros::Time current_time = ros::Time::now();
  //获取机器人全局位姿
  try
  {
      // 尽可能使用当前时间（确保它不是将来的时间）
      if (tf_.canTransform("map", "base_link", current_time))
      {
          geometry_msgs::TransformStamped transform = tf_.lookupTransform("map", "base_link", current_time);
          tf2::doTransform(robot_pose, global_pose, transform);
      }
      // 否则使用最近的时间
      else
      {
          tf_.transform(robot_pose, global_pose, "map");
      }
  }
  catch (tf2::LookupException& ex)
  {
      ROS_ERROR_THROTTLE(1.0, "查找机器人姿态时出现错误(无可用转换): %s\n", ex.what());
      return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
      ROS_ERROR_THROTTLE(1.0, "查找机器人姿态时出现连接错误: %s\n", ex.what());
      return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
      ROS_ERROR_THROTTLE(1.0, "查找机器人姿态时出现外推错误: %s\n", ex.what());
      return false;
  }
  // 检查 global_pose 超时
  if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance)
  {
      ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS 转换超时. 当前时间: %.4f, global_pose 时间戳: %.4f, 允许偏差: %.4f",
                      current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance);
      return false;
  }
  //发出实时位置信号
  emit position(global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z);

  return true;
}

/**
 * @brief 位姿数据转换(将double类型数据乘1000后转换为int16_t类型以发送给PLC使用)
 * @param 全局姿态global_pose
 * @param x坐标x
 * @param y坐标y
 * @param 偏航角curYaw
 * @attention 在本项目中，该函数并未使用到
 */
void QNode::pose_data_converse(geometry_msgs::PoseStamped& global_pose, int16_t& x, int16_t& y, int16_t& curYaw)
{
  float xPose = global_pose.pose.position.x * 1000;
  float yPose = global_pose.pose.position.y * 1000;
  x = (int16_t)std::floor(xPose);
  y = (int16_t)std::floor(yPose);
  tf::Quaternion tfq;
  tf::quaternionMsgToTF(global_pose.pose.orientation, tfq);  // 把geomsg形式的四元数转化为tf形式，得到tfq
  double yaw = tf::getYaw(tfq);
  if (yaw < 0)
  {
      yaw += 2*M_PI;
  }
  curYaw = (int16_t)std::floor(yaw *1000);
}

/**
 * @brief 机器人避障方法(通过sick_safetyscanners/scan1话题数据判断避障)
 * @attention 在本项目中，该函数并未使用到
 */
std::vector<int16_t> QNode::scanObstacleDetectionToshortestDistanceAroundAGV()
{
    std::vector<int16_t> front_back_left_right;
    // 前面的激光雷达
    int length1 = laserPoint1.ranges.size();
    int length2 = laserPoint2.ranges.size();
    if ((length1 == 0) || (length2 == 0))
    {
        return {10000, 10000, 10000, 10000};
    }

    // std::cout<<"length1: "<<length1<<std::endl;
    std::vector<float> laser1_x;
    std::vector<float> laser1_yLeft;
    std::vector<float> laser1_yRight;
    std::vector<float> front_left_right_shortest_distance;

    for (int i = 0; i < length1; i++)
    {
        float angle = laserPoint1.angle_increment * i + laserPoint1.angle_min;
        float x = laserPoint1.ranges[i] * std::cos(angle);
        float y = laserPoint1.ranges[i] * std::sin(angle);
        if ((x > 0.15) && (!isinf(x)))
        {
            if (fabs(y) > 0.4)
            {
                float front_side_distance = hypotf32(x-0.15, fabs(y)-0.4);
                laser1_x.push_back(front_side_distance);
            }
            if (fabs(y) <= 0.4)
            {
                laser1_x.push_back(x-0.15);
            }
        }
        if ((x <= 0.15) && (y <= -0.4))
        {
            laser1_yLeft.push_back(y);
        }
        if ((x <= 0.15) && (y > 0.4))
        {
            laser1_yRight.push_back(y);
        }
    }

    if (laser1_x.size() == 0)
    {
        front_left_right_shortest_distance.push_back(10.0);
    }
    else if (laser1_x.size() > 0)
    {
        std::sort(laser1_x.begin(), laser1_x.end());
        front_left_right_shortest_distance.push_back(fabs(laser1_x.at(0)));
    }

    if (laser1_yLeft.size() == 0)
    {
        front_left_right_shortest_distance.push_back(10.0);
    }
    else if(laser1_yLeft.size() > 0)
    {
        std::sort(laser1_yLeft.rbegin(), laser1_yLeft.rend());
        front_left_right_shortest_distance.push_back(fabs(laser1_yLeft.at(0)));
    }

    if (laser1_yRight.size() == 0)
    {
        front_left_right_shortest_distance.push_back(10.0);
    }
    else if(laser1_yRight.size() > 0)
    {
        std::sort(laser1_yRight.begin(), laser1_yRight.end());
        front_left_right_shortest_distance.push_back(fabs(laser1_yRight.at(0)));
    }


    // 后面的激光雷达

    // std::cout<<"length2: "<<length2<<std::endl;
    std::vector<float> laser2_x;
    std::vector<float> laser2_yLeft;
    std::vector<float> laser2_yRight;
    std::vector<float> back_left_right_shortest_distance;

    for (int i = 0; i < length2; i++)
    {
        float angle = laserPoint2.angle_increment * i + laserPoint2.angle_min;
        float x = laserPoint2.ranges[i] * std::cos(angle);
        float y = laserPoint2.ranges[i] * std::sin(angle);
        if ((x > 0.15) && (!isinf(x)))
        {
            if (fabs(y) > 0.4)
            {
                float back_side_distance = hypotf32(x-0.15, fabs(y)-0.4);
                laser2_x.push_back(back_side_distance);
            }
            if (fabs(y) <= 0.4)
            {
                laser2_x.push_back(x-0.15);
            }
        }
        if ((x <= 0.15) && (y >= 0.4))
        {
            laser2_yLeft.push_back(y);
        }
        if ((x <= 0.15) && (y < -0.4))
        {
            laser2_yRight.push_back(y);
        }
    }

    if (laser2_x.size() == 0)
    {
        back_left_right_shortest_distance.push_back(10.0);
    }
    else if (laser2_x.size() > 0)
    {
        std::sort(laser2_x.begin(), laser2_x.end());
        back_left_right_shortest_distance.push_back(fabs(laser2_x.at(0)));
    }

    if (laser2_yLeft.size() == 0)
    {
        back_left_right_shortest_distance.push_back(10.0);
    }
    else if(laser2_yLeft.size() > 0)
    {
        std::sort(laser2_yLeft.begin(), laser2_yLeft.end());
        back_left_right_shortest_distance.push_back(fabs(laser2_yLeft.at(0)));
    }

    if (laser2_yRight.size() == 0)
    {
        back_left_right_shortest_distance.push_back(10.0);
    }
    else if(laser2_yRight.size() > 0)
    {
        std::sort(laser2_yRight.rbegin(), laser2_yRight.rend());
        back_left_right_shortest_distance.push_back(fabs(laser2_yRight.at(0)));
    }

    // std::vector<int16_t> front_back_left_right;
    int16_t front = (int16_t)(front_left_right_shortest_distance.at(0) * 1000);
    int16_t back = (int16_t)(back_left_right_shortest_distance.at(0) * 1000);
    front_back_left_right.push_back(front);
    front_back_left_right.push_back(back);

    // 左边最小距离
    if (front_left_right_shortest_distance.at(2) > back_left_right_shortest_distance.at(2))
    {
        int16_t back_right_modefy = (int16_t)((back_left_right_shortest_distance.at(2) - 0.4) * 1000);
        front_back_left_right.push_back(back_right_modefy);
    }
    if(front_left_right_shortest_distance.at(2) <= back_left_right_shortest_distance.at(2))
    {
        int16_t front_right_modefy = (int16_t)((front_left_right_shortest_distance.at(2) - 0.4) * 1000);
        front_back_left_right.push_back(front_right_modefy);
    }
    // 右边最小距离
    if (front_left_right_shortest_distance.at(1) > back_left_right_shortest_distance.at(1))
    {
        int16_t back_left_modefy = (int16_t)((back_left_right_shortest_distance.at(1) - 0.4) * 1000);
        front_back_left_right.push_back(back_left_modefy);
    }
    if (front_left_right_shortest_distance.at(1) <= back_left_right_shortest_distance.at(1))
    {
        int16_t front_left_modefy = (int16_t)((front_left_right_shortest_distance.at(1) - 0.4) * 1000);
        front_back_left_right.push_back(front_left_modefy);
    }

    return front_back_left_right;
}

/**
 * @brief 将地图(yaml)文件的路径信息传递给ROS参数服务器(参数名为"/yaml_file_path")
 * @attention 多次调用该函数则新的值会覆盖旧的值
 * @param filePath 文件路径字符串，比如"/home/user/map/map.yaml"
 */
void QNode::set_yaml_file_path(const QString& filePath)
{
  ros::param::set("/yaml_file_path", filePath.toStdString());
}

/**
 * @brief 获取ROS参数服务器(参数名为"/yaml_file_path")的地图(yaml)文件的路径信息
 * @attention 如果从未设置过该参数的值，则会返回一个空字符串
 * @retval 地图(yaml)文件路径字符串
 */
QString QNode::get_yaml_file_path()
{
  std::string filePath;
  ros::param::get("/yaml_file_path", filePath);
  return QString::fromStdString(filePath);
}

/**
 * @brief 显示所有标准点方法(通过标记数组MarkerArray图层显示)
 * @param 是否显示(visible)
 */
void QNode::show_all_standard_points(bool visible)
{
  //创建Marker消息类型对象
  visualization_msgs::Marker marker;
  //创建Point消息类型对象
  geometry_msgs::Point point;
  //设置frame
  marker.header.frame_id = "map";
  //设置时间戳
  marker.header.stamp = ros::Time::now();
  //设置动作
  marker.action = visualization_msgs::Marker::ADD;
  //设置类型
  marker.type = visualization_msgs::Marker::POINTS;
  //设置点的宽和高
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  //设置点的颜色
  marker.color.r = 1.0f;
  marker.color.a = 1.0;
  if(visible){
    QVector<StandardPoint>::Iterator p;
    for(p = standardPoints.begin(); p < standardPoints.end(); p++){
      point.x = p->x();
      point.y = p->y();
      point.z = 0.0;
      //点入栈
      marker.points.push_back(point);
    }
  }
  else{
    marker.points.clear();
  }
  //发布marker
  standard_points_publisher.publish(marker);
}

/**
 * @brief 使用线性搜索查找二维平面内距离目标坐标最近的标准点
 * @param 目标x坐标x
 * @param 目标y坐标y
 * @return 标准点(StandardPoint)类对象
 */
StandardPoint QNode::linear_nearest_neighbor_search(double x, double y)
{
  QVector<StandardPoint>::Iterator p;
  StandardPoint nearestPoint = standardPoints[0];           //使用StandardPoint对象保存当前最近的点
  double nearestDistance = pow((standardPoints[0].x() - x), 2) + pow((standardPoints[0].y() - y), 2); //使用一个double保存最近距离(二维欧式距离的平方)
  for(p = standardPoints.begin() + 1; p < standardPoints.end(); p++){
    //nearestPoint = ((pow((p->x() - x), 2) + pow((p->y() - y), 2)) < nearestDistance) ? *p : nearestPoint; //使用三目运算符判断最近邻
    double newDistance = pow((p->x() - x), 2) + pow((p->y() - y), 2);
    if(newDistance < nearestDistance){
      nearestPoint = *p;                //更新最近邻和最近距离
      nearestDistance = newDistance;
    }
  }

  return nearestPoint;
}

/**
 * @brief 如果points数组不为空的话，撤回一个点位
 */
void QNode::point_revocation()
{
  if(points.points.size() > 0){
    points.points.pop_back();
    path.poses.pop_back();
    emit pointRevoked();
    marker_publisher.publish(points);     //发布marker
    global_plan_publisher.publish(path);  //发布path
  }
}

/**
 * @brief 根据points成员变量的起点查找是否为标准点，如果是返回标准点名称，不是返回空字符串
 */
QString QNode::start_point_name_search()
{
  if(points.points.size() > 0){
    double coordinate_x = points.points[0].x;
    double coordinate_y = points.points[0].y;
    for(QVector<StandardPoint>::Iterator p = standardPoints.begin(); p < standardPoints.end(); p++){
      if(coordinate_x == (*p).x() && coordinate_y == (*p).y())
        return (*p).pointName();
    }
  }
  return "";
}

/**
 * @brief 根据points成员变量的终点查找是否为标准点，如果是返回标准点名称，不是返回空字符串
 */
QString QNode::end_point_name_search()
{
  if(points.points.size() > 0){
    double coordinate_x = points.points.back().x;
    double coordinate_y = points.points.back().y;
    for(QVector<StandardPoint>::Iterator p = standardPoints.begin(); p < standardPoints.end(); p++){
      if(coordinate_x == (*p).x() && coordinate_y == (*p).y())
        return (*p).pointName();
    }
  }
  return "";
}

}  // namespace class1_ros_qt_demo

