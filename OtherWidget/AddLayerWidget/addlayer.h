#ifndef ADDLAYER_H
#define ADDLAYER_H

#include <QDialog>
#include <ros/ros.h>
#include <QDebug>
#include <QTreeWidgetItem>
#include <QPushButton>

namespace Ui {
class AddLayer;
}

class AddLayer : public QDialog
{
  Q_OBJECT

public:
  //Layer Code
  enum LayerCode{
    null,
    grid,
    tf,
    laserscan,
    pointcloud2,
    robotmodel,
    map,
    path,
    pointstamped,
    marker,
    markerarray
  };

  explicit AddLayer(QWidget *parent = nullptr);
  void on_treeWidget_AllLayer_currentItemChanged();
  void on_treeWidget_ByTopic_currentItemChanged();
  void treeWidget_ByTopic_addItemByTopic(QTreeWidget *treeWidget, std::string name, std::string datatype);
  void slot_Accepted();
  static int getLayer(QWidget *parent = nullptr, const QString &title = QString("RViz"));
  static QString getDisplayName();
  static inline QString getTopicName(){
    return TopicToSubscribe;
  }
  static inline QStringList getTopicListByLayerCode(AddLayer::LayerCode c){
    switch (c) {
      case AddLayer::laserscan:
        return laserScanTopicList;
      break;
      case AddLayer::pointcloud2:
        return pointCloud2TopicList;
      break;
      case AddLayer::map:
        return mapTopicList;
      break;
      case AddLayer::path:
        return pathTopicList;
      break;
      case AddLayer::pointstamped:
        return pointStampedTopicList;
      break;
      case AddLayer::marker:
        return markerTopicList;
      break;
      case AddLayer::markerarray:
        return markerArrayTopicList;
      break;
      default:
        return QStringList();
      break;
    }
  }
  ~AddLayer();

private:
  Ui::AddLayer *ui;
  //使用QMap保存图层名与对应的图层枚举值编码
  QMap<QString, int> DisplayNameMap{
    {"Map", AddLayer::map},
    {"LaserScan", AddLayer::laserscan},
    {"PointCloud2", AddLayer::pointcloud2},
    {"Path", AddLayer::path},
    {"PointStamped", AddLayer::pointstamped},
    {"Marker", AddLayer::marker},
    {"MarkerArray", AddLayer::markerarray}
  };
  //使用QHash保存消息类型与对应的图层枚举值编码
  QHash<QString, int> MsgTypeHash{
    {"sensor_msgs/LaserScan", AddLayer::laserscan},
    {"sensor_msgs/PointCloud2", AddLayer::pointcloud2},
    {"nav_msgs/OccupancyGrid", AddLayer::map},
    {"nav_msgs/Path", AddLayer::path},
    {"geometry_msgs/PointStamped", AddLayer::pointstamped},
    {"visualization_msgs/Marker", AddLayer::marker},
    {"visualization_msgs/MarkerArray", AddLayer::markerarray}
  };


  static int LayerCodeToReturn;   //因为静态成员函数只能访问静态成员变量，所以声明为静态成员变量，该变量记录display图层编号
  static QString DisplayName;
  static QString TopicToSubscribe;
  static QStringList laserScanTopicList;
  static QStringList pointCloud2TopicList;
  static QStringList mapTopicList;
  static QStringList pathTopicList;
  static QStringList pointStampedTopicList;
  static QStringList markerTopicList;
  static QStringList markerArrayTopicList;

  //rviz layer
  QTreeWidgetItem* _rviz;
  QTreeWidgetItem* _grid;
  QTreeWidgetItem* _tf;
  QTreeWidgetItem* _laserscan;
  QTreeWidgetItem* _pointcloud2;
  QTreeWidgetItem* _robotmodel;
  QTreeWidgetItem* _map;
  QTreeWidgetItem* _path;
  QTreeWidgetItem* _pointstamped;
  QTreeWidgetItem* _marker;
  QTreeWidgetItem* _markerarray;

protected:
  void topicListAppendByTopicInfo(const ros::master::TopicInfo& info);
};

#endif // ADDLAYER_H
