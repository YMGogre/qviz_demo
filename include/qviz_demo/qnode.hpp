/**
 * @file /include/class1_ros_qt_demo/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef class1_ros_qt_demo_QNODE_HPP_
#define class1_ros_qt_demo_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <ros/network.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <QVector>
#include <algorithm>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/PoseWithCovarianceStamped.h>    //引入amcl_pose的消息类型
#include <geometry_msgs/PoseStamped.h>                  //引入导航目标点的消息类型
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>                  //引入标记点消息类型
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>                              //引入路径消息类型
#include "../../OtherWidget/SavePoint/savepoint.h"

//引入tf包
#include "tf/transform_datatypes.h"
#include "tf2_ros/buffer.h"                             //
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//引入Lasercan包
#include "sensor_msgs/LaserScan.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace class1_ros_qt_demo {

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread {
  Q_OBJECT
public:
  static double clicked_point[];
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
  **********************/
  enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std::string &msg);
  inline void set_generate_task_state(bool state){
    isGenerateTask = state;
    if(isGenerateTask){ clear_marker_and_path(); }
  }
  inline void set_use_standard_point(bool state){
    isUsingStandardPoint = state;
  }
  inline bool isClicked(){
    return _isClicked;
  }
  inline void standardPoints_clear(){
    standardPoints.clear();
  }
  inline void standardPoints_append(StandardPoint &sp){
    standardPoints.append(sp);
  }
  inline void clear_marker_and_path()                                     //接口函数用于清空marker和path(在创建任务以及保存任务完成时都需要用到)
  {
    points.points.clear();                //清空marker
    marker_publisher.publish(points);     //发布marker(发布了清空后的Marker显示界面显示的点位才会同样清空)
    path.poses.clear();                   //清空path
    global_plan_publisher.publish(path);  //发布path
  }
  inline void publish_task_path_preview(const nav_msgs::Path& path){
    task_path_preview_publisher.publish(path);
  }
  inline void publish_task_path(const nav_msgs::Path& path){
    task_path_publisher.publish(path);
  }
  void slot_publish_clicked_point(double x, double y);                    //槽函数接口，用于连接MainWindow的“发布点位”按钮的点击信号
  nav_msgs::Path slot_publish_global_plan();                              //发布"global_plan"话题槽函数，用于连接MainWindow的“保存任务”按钮的点击信号
  bool get_curr_robot_pose(geometry_msgs::PoseStamped& global_pose, tf2_ros::Buffer& tf_);    //获取机器人实时位置函数(通过订阅TF实现)
  void pose_data_converse(geometry_msgs::PoseStamped& global_pose);                           //位置数据转换方法(不传给PLC直接发送实时位置信号到MainWindow)
  void pose_data_converse(geometry_msgs::PoseStamped& global_pose, int16_t& x, int16_t& y, int16_t& curYaw);    //位置数据转换方法(获取到的位置数据是double，要乘1000后再传给PLC)
  std::vector<int16_t> scanObstacleDetectionToshortestDistanceAroundAGV();                    //机器人避障方法(通过sick_safetyscanners/scan1话题数据判断避障)
  void set_yaml_file_path(const QString& filePath);                       //接口函数用于设置ROS参数服务器上的名为"yaml_file_path"的参数的值(该值用于指定当前提供地图服务的yaml文件路径)
  QString get_yaml_file_path();                                           //接口函数用于获取ROS参数服务器上的名为"yaml_file_path"的参数的值(该值用于指定当前提供地图服务的yaml文件路径)
  void show_all_standard_points(bool visible);
  StandardPoint linear_nearest_neighbor_search(double x, double y);
  void point_revocation();
  QString start_point_name_search();
  QString end_point_name_search();


Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void position(double x, double y, double z);          //自定义信号将实时坐标发送到MainWindow中
  void mapOpened();
  void pointClicked();
  void pointRevoked();


private:
	int init_argc;
  char** init_argv;
  const float transform_tolerance = 0.2;
  bool isGenerateTask = false;    //该bool值记录了当前用户是否处在创建任务过程
  bool isUsingStandardPoint = false;      //该bool值记录了在创建任务过程中是否使用标准点
  QVector<StandardPoint> standardPoints;  //使用一个StandardPoint类型的动态数组保存所有标准点
  bool _isClicked = false;        //该bool值记录了打开软件后用户是否曾经使用过“发布点位”的RViz工具
  //TF
  tf2_ros::Buffer buffer;
  geometry_msgs::PoseStamped global_pose;

  //实例化Marker消息类型的对象
  visualization_msgs::Marker points;


  //声明Path消息类型对象
  nav_msgs::Path path;

  sensor_msgs::LaserScan laserPoint1;
  sensor_msgs::LaserScan laserPoint2;

  //Publisher
  ros::Publisher clicked_point_publisher;     //声明一个点击点话题发布者
  ros::Publisher marker_publisher;            //声明一个标记话题发布者
  ros::Publisher standard_points_publisher;   //声明一个标准点话题发布者(这是一个自定义话题，使用的消息类型为visualization_msgs::Marker)
  ros::Publisher global_plan_publisher;       //声明一个规划路径话题发布者
  ros::Publisher task_path_preview_publisher; //声明一个任务路径预览话题发布者(这是一个自定义话题，使用的消息类型为nav_msgs::Path)
  ros::Publisher task_path_publisher;         //声明一个任务路径话题发布者(这是一个自定义话题，使用的消息类型为nav_msgs::Path)

  //Subscriber
  ros::Subscriber clicked_point_sub;      //声明一个clicked_point话题订阅者
  ros::Subscriber laserScan1Sub_;         //声明一个sick_safetyscanners/scan1话题订阅者
  ros::Subscriber laserScan2Sub_;         //声明一个sick_safetyscanners/scan2话题订阅者
  ros::Subscriber mapSub;                 //声明一个map话题订阅者

  QStringListModel logging_model;
  void clicked_point_callback(const geometry_msgs::PointStamped &point_with_stamp);
  void map_callback(const nav_msgs::OccupancyGrid& occupancyGrid);
  void laser1Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void laser2Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

}  // namespace class1_ros_qt_demo

#endif /* class1_ros_qt_demo_QNODE_HPP_ */
