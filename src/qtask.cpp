#include "../include/qviz_demo/qtask.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace class1_ros_qt_demo {

QTask::QTask() {}

QTask::QTask(QJsonObject obj)
{
  //设置frame(path)
  _path.header.frame_id = "map";
  //设置时间戳
  _path.header.stamp = ros::Time::now();
  init(obj);
}

/**
 * @brief 初始化方法，接受一个JSON对象并依据其中的键值对来初始化QTask对象的成员变量
 * @param JSON对象obj
 */
void QTask::init(QJsonObject obj)
{
  _id = obj.value("id").toString();
  _name = obj.value("name").toString();
  _start = obj.value("start").toString();
  _finish = obj.value("finish").toString();
  _description = obj.value("description").toString();
  QJsonArray ArrX = obj.value("x").toArray();
  QJsonArray ArrY = obj.value("y").toArray();
  QJsonArray ArrMarchMode = obj.value("mode_type").toArray();
  geometry_msgs::PoseStamped pose;
  for(int index = 0; index < ArrX.size(); index++)
  {
    if(ArrX.at(index).isDouble() && ArrY.at(index).isDouble())
    {
      double temp_x = ArrX.at(index).toDouble();
      _x.push_back(temp_x);
      pose.pose.position.x = temp_x;

      double temp_y = ArrY.at(index).toDouble();
      _y.push_back(temp_y);
      pose.pose.position.y = temp_y;

      int temp_marchMode = ArrMarchMode.at(index).toInt();
      _marchMode.push_back(temp_marchMode);

      _path.poses.push_back(pose);
    }
  }
}

//静态成员变量初始化
QVector<QTask> QTaskHandle::Tasks = QVector<QTask>();
QQueue<QTask> QTaskHandle::taskQueue = QQueue<QTask>();

QTaskHandle::QTaskHandle() {}

}  // namespace class1_ros_qt_demo
