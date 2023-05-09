/**
 * @file /include/class1_ros_qt_demo/main_window.hpp
 *
 * @brief Qt based gui for class1_ros_qt_demo.
 *
 * @date November 2010
 **/
#ifndef class1_ros_qt_demo_MAIN_WINDOW_H
#define class1_ros_qt_demo_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include <QComboBox>
#include <QtGui>
#include <QIcon>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMessageBox>
#include <QCheckBox>
#include <QColorDialog>
#include <QDebug>
#include <QSpinBox>
#include <QDebug>
#include <QTextStream>
#include <QProcess>
#include <QSystemTrayIcon>
#include <iostream>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "qrviz.hpp"
#include "qtask.h"
#include "../../CustomControl/MyPalette/mypalette.h"
#include "../../OtherWidget/AddLayerWidget/addlayer.h"
#include "../../OtherWidget/AddMapManager/addmapmanager.h"
#include "../../OtherWidget/GenerateTask/generatetask.h"
#include "../../OtherWidget/SavePoint/savepoint.h"
#include "../../OtherWidget/ShellSelectWidget/shellselect.h"


/*****************************************************************************
** Namespace
*****************************************************************************/
namespace class1_ros_qt_demo {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  void ReadSettings();    // Load up qt program settings at startup
  void WriteSettings();   // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

  /******************************************
  ** 向UI界面添加新图层相关控件方法
  *******************************************/
  QTreeWidgetItem* add_TreeWidgetItem_Grid(QTreeWidget* treeWidget, QString& displayName);
  QTreeWidgetItem* add_TreeWidgetItem_TF(QTreeWidget* treeWidget, QString& displayName);
  QTreeWidgetItem* add_TreeWidgetItem_LaserScan(QTreeWidget* treeWidget, QString& displayName, QString& topicName);
  QTreeWidgetItem* add_TreeWidgetItem_PointCloud2(QTreeWidget* treeWidget, QString& displayName, QString& topicName);
  QTreeWidgetItem* add_TreeWidgetItem_RobotModel(QTreeWidget* treeWidget, QString& displayName);
  QTreeWidgetItem* add_TreeWidgetItem_Map(QTreeWidget* treeWidget, QString& displayName, QString& topicName);
  QTreeWidgetItem* add_TreeWidgetItem_Path(QTreeWidget* treeWidget, QString& displayName, QString& topicName, QColor color = QColor(25, 255, 0));
  QTreeWidgetItem* add_TreeWidgetItem_PointStamped(QTreeWidget* treeWidget, QString& displayName, QString& topicName);
  QTreeWidgetItem* add_TreeWidgetItem_Marker(QTreeWidget* treeWidget, QString& displayName, QString& topicName);
  QTreeWidgetItem* add_TreeWidgetItem_MarkerArray(QTreeWidget* treeWidget, QString& displayName, QString& topicName);

  /******************************************
  ** 修改图层相关属性方法
  *******************************************/
  void slot_modify_grid_attribute();
  void slot_modify_TF_attribute();
  void slot_modify_LaserScan_attribute();
  void slot_modify_PointCloud2_attribute();
  void slot_modify_RobotModel_attribute();
  void slot_modify_Map_attribute();
  void slot_modify_Path_attribute();
  void slot_modify_PointStamped_attribute();
  void slot_modify_Marker_attribute();
  void slot_modify_MarkerArray_attribute();

  /******************************************
  ** 从UI界面移除待删除的图层相关控件方法
  *******************************************/
  void remove_TreeWidgetItem_Grid(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_TF(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_LaserScan(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_PointCloud2(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_RobotModel(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_Map(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_Path(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_PointStamped(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_Marker(QTreeWidget* treeWidget, QTreeWidgetItem* item);
  void remove_TreeWidgetItem_MarkerArray(QTreeWidget* treeWidget, QTreeWidgetItem* item);

  /******************************************
  ** 向UI界面添加一个地图管理相关控件方法
  *******************************************/
  void add_TableWidgetRow(QTableWidget* tablewidget, QString& mapPath, QString& mapName, QString mapDescription = "");
  void delete_TableWidgetRow(QTableWidget* tablewidget, int row);

  /******************************************
  ** 向UI界面添加一个任务管理相关控件方法
  *******************************************/
  void add_TableWidgetRow(QTableWidget* tablewidget, QString& taskID, QString& taskName, QString &start, QString &finish, QString &taskDescription);
  void remove_TableWidgetAllRows(QTableWidget* tablewidget);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
  void on_button_add_layer_clicked();
  void on_button_remove_layer_clicked();

  //docktreeWidget相关槽函数
  void slot_display_layer_description();
  void slot_enable_remove_button(QTreeWidgetItem* current);

  //Global Options相关槽函数
  void slot_set_global_options();

  //style改变槽函数
  void slot_onStyleComboBox_text_changed();

  //point size编辑结束槽函数
  void slot_onPointSizeLineEdit_edit_finished();

  //使用2D Pose Estimate工具槽函数
  void slot_set_initial_pose();

  //使用2D Nav Goal工具槽函数
  void slot_set_goal_pose();

  //更新实时位置槽函数
  void slot_update_pos(double x, double y, double z);

  //TaskManager任务管理窗口相关槽函数
  void slot_publish_point();
  void slot_task_path_preview(const QString& taskName);
  void slot_generate_task();
  void slot_save_task();

  //打开地图相关槽函数
  void on_QProcessErrorOccurred(QProcess::ProcessError errorcode);
  void QProcessOpenMap();   //QProcess执行rosrun map_server map_server命令(无法销毁ros节点)
  void GnomeOpenMap();      //终端执行rosrun map_server map_server命令(需ctrl + c手动销毁ros节点)
  void slot_map_opened();

  //打开任务和加载点位相关槽函数
  void OpenTasks(QString path_to_json);
  void slot_on_task_updated(QString taskname);
  void LoadStandardPoints(QString path_to_json);
  void slot_on_point_updated(StandardPoint& newPoint);
  void slot_show_all_standard_points(bool visible);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
  bool isUsingStandardPoint;            //该bool值决定了用户使用RViz发布点位工具
  QHash<QTreeWidgetItem*, rviz::Display*> m_LayerHash;

  //Global Options树控件
  QTreeWidgetItem* Global;
  QComboBox* FFCBox;                    //Fixed Frame ComboBox
  MyPalette* BPalette;                  //Background Palette

  QNode qnode;
  qrviz* myviz;
  ShellSelect* shellSelect;
  AddLayer* addLayer;
  //地图相关变量
  QString yamlFilePath;                           //地图yaml文件路径
  QString mapFileName;                            //地图名称(根据yaml文件名称获得或者根据用户输入获得)
  QProcess* mapServerProcess = new QProcess();
  //任务和点位相关变量
  bool isGenerateTask = false;
  double currRobotPosition[2]{0.00, 0.00};        //使用长度为2的double数组保存当前机器人位置(只保存x和y坐标)
  QVector<int> modeTypes;
  /**
   * @brief 逆序遍历QTreeWidgetItem所有子节点(包括儿子节点、孙子节点、曾孙节点...)并删除他们(包括从UI界面移除以及释放内存)
   * @attention 该方法会使用delete关键字释放内存，在调用该方法前请保证所有的子节点对象都是通过new关键字创建的
   * @param QTreeWidgetItem* item   ———— 待移除子节点的QTreeWidgetItem指针
   */
  inline void removeChildren(QTreeWidgetItem* item)
  {
    for (int i = item->childCount() - 1; i >= 0; i--) {
      if(item->child(i)->childCount() > 0)
      {
        removeChildren(item->child(i)); //递归删除子节点的子节点
      }
      //因为QTreeWidgetItem::takeChild(int index)方法会从UI界面移除index下标的项并返回它，所以我们直接delete其返回值
      delete item->takeChild(i);        //从UI界面移除项并释放其内存
    }
  }

  GenerateTask* gt_ptr;
  SavePoint* sp_ptr;

  };
}  // namespace class1_ros_qt_demo

#endif // class1_ros_qt_demo_MAIN_WINDOW_H
