#ifndef QTASK_H
#define QTASK_H

#include <QString>
#include <QVector>
#include <QQueue>
#include <QJsonObject>
#include <QJsonArray>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace class1_ros_qt_demo {

/*****************************************************************************
** Class
*****************************************************************************/
class QTask
{
public:
  QTask();
  QTask(QJsonObject obj);
  inline QString id(){
    return _id;             //获取任务id
  }
  inline QString name(){
    return _name;           //获取任务名称
  }
  inline QString start(){
    return _start;          //获取任务起始点
  }
  inline QString finish(){
    return _finish;         //获取任务终点
  }
  inline QString description(){
    return _description;    //获取任务描述
  }
  inline QVector<double> x(){
    return _x;              //获取任务路径点位x坐标动态数组
  }
  inline QVector<double> y(){
    return _y;              //获取任务路径点位y坐标动态数组
  }
  inline nav_msgs::Path path(){
    return _path;           //获取任务路径
  }

private:
  QString _id;
  QString _name;
  QString _start;
  QString _finish;
  QString _description;
  QVector<double> _x;
  QVector<double> _y;
  QVector<int> _marchMode;
  nav_msgs::Path _path;
  void init(QJsonObject obj);
};

class QTaskHandle
{
public:
  QTaskHandle();
  //使用QVector或者QQueue类模板时会去调用目标类(QTask类)的默认构造函数去初始化其中的元素，如果我们没有提供其他方法来创建目标类对象并填充，就需要在目标类内提供默认构造函数
  static QVector<QTask> Tasks;              //使用一个静态成员变量存储维护打开地图所属的任务数组，以保证唯一性
  static QQueue<QTask> taskQueue;           //使用一个静态成员变量存储任务队列(任务队列是处理任务的队列，当我们收到机器人可以执行任务的消息时，在回调函数内取出队首任务，将其对应的路径发送出去(通过ROS话题发布)并移出队列，直至任务队列清空)
  inline void push_back_task(const QTask &task){
    Tasks.push_back(task);      //向任务数组尾部添加一个任务(QTask)对象
  }
  inline QTask& last(){
    return Tasks.last();        //获取队尾元素
  }
  inline void clear_task_vector(){
    Tasks.clear();              //清空任务数组
  }
  inline QTask* query_task_by_id(QString id){
    for(QVector<QTask>::Iterator it = Tasks.begin(); it != Tasks.end(); it++){
      if((*it).id() == id){
        return &(*it);          //查询到则结束循环立刻返回
      }
    }
    return nullptr;             //循环结束还未查询到则返回空指针
  }
  inline QTask* query_task_by_name(QString name){
    for(QVector<QTask>::Iterator it = Tasks.begin(); it != Tasks.end(); it++){
      if((*it).name() == name){
        return &(*it);          //查询到则结束循环立刻返回
      }
    }
    return nullptr;             //循环结束还未查询到则返回空指针
  }


  inline void enqueue(const QTask &task){
    taskQueue.enqueue(task);    //向任务队列尾部添加项
  }
  inline QTask dequeue(){
    return taskQueue.dequeue(); //从任务队列队首取出项
  }
  inline bool isEmpty(){
    return taskQueue.isEmpty(); //判断队列是否为空
  }

private:
};

}  // namespace class1_ros_qt_demo

#endif // QTASK_H
