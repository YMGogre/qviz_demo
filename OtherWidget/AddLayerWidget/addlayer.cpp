#include "addlayer.h"
#include "ui_addlayer.h"

AddLayer::AddLayer(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::AddLayer)
{
  ui->setupUi(this);
  ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false); //初始禁用"OK"按钮
  LayerCodeToReturn = AddLayer::null;
  DisplayName = "";
  TopicToSubscribe = "";
  laserScanTopicList = QStringList();
  pointCloud2TopicList = QStringList();
  mapTopicList = QStringList();
  pathTopicList = QStringList();
  pointStampedTopicList = QStringList();
  markerTopicList = QStringList();
  markerArrayTopicList = QStringList();   //这些静态成员变量比较特殊，本来静态成员变量是属于类的所以不要在构造函数里初始化它们，但这里我们需要在构造函数里初始化它们。因为它们本不应该被做成静态变量的，只是我想让其访问方便

  /*********************
   *  rviz tree rviz树（通过显示类型添加图层）
   *********************/
  _rviz = new QTreeWidgetItem(QStringList() << "rviz");
  _rviz->setIcon(0, QIcon("://rViz/Group.png"));
  ui->treeWidget_AllLayer->addTopLevelItem(_rviz);            //添加根节点
  _rviz->setExpanded(true);                                   //设置默认展开
  /*********************/
  _grid = new QTreeWidgetItem(QStringList() << "网格");
  _grid->setIcon(0, QIcon("://rViz/Grid.png"));
  _rviz->addChild(_grid);                                     //添加grid子节点
  /*********************/
  _tf = new QTreeWidgetItem(QStringList() << "坐标转换(TF)");
  _tf->setIcon(0, QIcon("://rViz/TF.png"));
  _rviz->addChild(_tf);                                       //添加tf子节点
  /*********************/
  _laserscan = new QTreeWidgetItem(QStringList() << "激光扫描(Laser Scan)");
  _laserscan->setIcon(0, QIcon("://rViz/LaserScan.png"));
  _rviz->addChild(_laserscan);                                //添加laserscan子节点
  /*********************/
  _pointcloud2 = new QTreeWidgetItem(QStringList() << "点云2");
  _pointcloud2->setIcon(0, QIcon("://rViz/PointCloud2.png"));
  _rviz->addChild(_pointcloud2);                              //添加pointcloud2子节点
  /*********************/
  _robotmodel = new QTreeWidgetItem(QStringList() << "机器人模型");
  _robotmodel->setIcon(0, QIcon("://rViz/RobotModel.png"));
  _rviz->addChild(_robotmodel);                               //添加robotmodel子节点
  /*********************/
  _map = new QTreeWidgetItem(QStringList() << "地图");
  _map->setIcon(0, QIcon("://rViz/Map.png"));
  _rviz->addChild(_map);                                      //添加map子节点
  /*********************/
  _path = new QTreeWidgetItem(QStringList() << "路径");
  _path->setIcon(0, QIcon("://rViz/Path.png"));
  _rviz->addChild(_path);                                     //添加path子节点
  /*********************/
  _pointstamped = new QTreeWidgetItem(QStringList() << "带点戳(Point Stamped)");
  _pointstamped->setIcon(0, QIcon("://rViz/PointStamped.png"));
  _rviz->addChild(_pointstamped);                             //添加pointstamped子节点
  /*********************/
  _marker = new QTreeWidgetItem(QStringList() << "标记");
  _marker->setIcon(0, QIcon("://rViz/Marker.png"));
  _rviz->addChild(_marker);                                   //添加marker子节点
  /*********************/
  _markerarray = new QTreeWidgetItem(QStringList() << "标记数组");
  _markerarray->setIcon(0, QIcon("://rViz/MarkerArray.png"));
  _rviz->addChild(_markerarray);                              //添加markerarray子节点
  /*********************/

  /*********************
   *  topic tree topic树（通过话题添加图层）
   *********************/
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);
  for(ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++){
    const ros::master::TopicInfo& info = *it;   //取出每一项并依次获取话题名和数据类型
    topicListAppendByTopicInfo(info);
    treeWidget_ByTopic_addItemByTopic(ui->treeWidget_ByTopic, info.name, info.datatype);
  }
  ui->treeWidget_ByTopic->expandAll();                        //展开所有项
  ui->treeWidget_ByTopic->sortItems(0, Qt::AscendingOrder);   //项排序

  /*********************
   *  connect 操作
   *********************/
  connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &AddLayer::slot_Accepted);
  connect(ui->treeWidget_AllLayer, &QTreeWidget::currentItemChanged, this, &AddLayer::on_treeWidget_AllLayer_currentItemChanged);
  connect(ui->treeWidget_ByTopic, &QTreeWidget::currentItemChanged, this, &AddLayer::on_treeWidget_ByTopic_currentItemChanged);
}

//定义静态成员变量
int AddLayer::LayerCodeToReturn = AddLayer::null;
QString AddLayer::DisplayName = "";
QString AddLayer::TopicToSubscribe = "";
QStringList AddLayer::laserScanTopicList = QStringList();
QStringList AddLayer::pointCloud2TopicList = QStringList();
QStringList AddLayer::mapTopicList = QStringList();
QStringList AddLayer::pathTopicList = QStringList();
QStringList AddLayer::pointStampedTopicList = QStringList();
QStringList AddLayer::markerTopicList = QStringList();
QStringList AddLayer::markerArrayTopicList = QStringList();

/**
 * @brief treeWidget_AllLayer“当前项改变”信号处理槽函数
 */
void AddLayer::on_treeWidget_AllLayer_currentItemChanged()
{
  if(ui->treeWidget_AllLayer->currentItem() == _rviz)
  {
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);   //当选中_rviz文件夹时，禁用"OK"按钮
    ui->textBrowser_Description->clear();                             //清空文本数据
    ui->lineEdit_DisplayName->clear();
  }
  else
  {
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
    switch(ui->treeWidget_AllLayer->topLevelItem(0)->indexOfChild(ui->treeWidget_AllLayer->currentItem()))
    {
      case 0:
        ui->textBrowser_Description->setText("网格\n\n\t沿地平面显示以目标参考系原点为中心的网格。");
        ui->lineEdit_DisplayName->setText("网格");
      break;
      case 1:
        ui->textBrowser_Description->setText("坐标变换\n\n\t显示坐标转换层次结构。");
        ui->lineEdit_DisplayName->setText("坐标转换(TF)");
      break;
      case 2:
        ui->textBrowser_Description->setText("激光扫描\n\n\t将来自sensor_msgs::LaserScan消息的数据显示为世界中的点，以点、公告板或立方体的形式绘制。");
        ui->lineEdit_DisplayName->setText("激光扫描(Laser Scan)");
      break;
      case 3:
        ui->textBrowser_Description->setText("点云2\n\n\t将来自sensor_msgs::PointCloud2消息的数据显示为世界中的点，以点、公告板或立方体的形式绘制。");
        ui->lineEdit_DisplayName->setText("点云2");
      break;
      case 4:
        ui->textBrowser_Description->setText("机器人模型\n\n\t以正确的姿态(由当前坐标变换定义)显示机器人的视觉表示。");
        ui->lineEdit_DisplayName->setText("机器人模型");
      break;
      case 5:
        ui->textBrowser_Description->setText("地图\n\n\t将来自nav_msgs::OccupancyGrid消息的数据显示为一个地平面的占用网格。");
        ui->lineEdit_DisplayName->setText("地图");
      break;
      case 6:
        ui->textBrowser_Description->setText("路径\n\n\t以线的形式显示nav_msgs::Path消息中的数据。");
        ui->lineEdit_DisplayName->setText("路径");
      break;
      case 7:
        ui->textBrowser_Description->setText("带点戳\n\n\t显示来自geometry_msgs/PointStamped消息的信息。");
        ui->lineEdit_DisplayName->setText("带点戳(Point Stamped)");
      break;
      case 8:
        ui->textBrowser_Description->setText("标记\n\n\t显示visualization_msgs::Marker消息。");
        ui->lineEdit_DisplayName->setText("标记");
      break;
      case 9:
        ui->textBrowser_Description->setText("标记数组\n\n\t在不假定话题名称以“_array”结尾的情况下显示visualization_msgs::MarkerArray消息。");
        ui->lineEdit_DisplayName->setText("标记数组");
      break;
    }
  }
}

/**
 * @brief treeWidget_ByTopic“当前项改变”信号处理槽函数
 */
void AddLayer::on_treeWidget_ByTopic_currentItemChanged()
{
  //没有设置图标的QTreeWidgetItem说明不是可以添加到RViz中显示的图层，此时禁用"OK"按钮
  if(ui->treeWidget_ByTopic->currentItem()->icon(0).isNull())
  {
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
    ui->textBrowser_Description->clear();
    ui->lineEdit_DisplayName->clear();
  }
  else
  {
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
    switch (DisplayNameMap.value(ui->treeWidget_ByTopic->currentItem()->text(0))) {
      case AddLayer::map:
        ui->textBrowser_Description->setText("地图\n\n\t将来自nav_msgs::OccupancyGrid消息的数据显示为一个地平面的占用网格。");
        ui->lineEdit_DisplayName->setText("地图");
      break;
      case AddLayer::laserscan:
        ui->textBrowser_Description->setText("激光扫描\n\n\t将来自sensor_msgs::LaserScan消息的数据显示为世界中的点，以点、公告板或立方体的形式绘制。");
        ui->lineEdit_DisplayName->setText("激光扫描(Laser Scan)");
      break;
      case AddLayer::pointcloud2:
        ui->textBrowser_Description->setText("点云2\n\n\t将来自sensor_msgs::PointCloud2消息的数据显示为世界中的点，以点、公告板或立方体的形式绘制。");
        ui->lineEdit_DisplayName->setText("点云2");
      break;
      case AddLayer::path:
        ui->textBrowser_Description->setText("路径\n\n\t以线的形式显示nav_msgs::Path消息中的数据。");
        ui->lineEdit_DisplayName->setText("路径");
      break;
      case AddLayer::pointstamped:
        ui->textBrowser_Description->setText("带点戳\n\n\t显示来自geometry_msgs/PointStamped消息的信息。");
        ui->lineEdit_DisplayName->setText("带点戳(Point Stamped)");
      break;
      case AddLayer::marker:
        ui->textBrowser_Description->setText("标记\n\n\t显示visualization_msgs::Marker消息。");
        ui->lineEdit_DisplayName->setText("标记");
      break;
      case AddLayer::markerarray:
        ui->textBrowser_Description->setText("标记数组\n\n\t在不假定话题名称以“_array”结尾的情况下显示visualization_msgs::MarkerArray消息。");
        ui->lineEdit_DisplayName->setText("标记数组");
      break;
    }
  }
}

/**
 * @brief 通过话题信息向QTreeWidget添加新项，整个项由话题名按照斜杠(/)分割构成，最后一级为话题使用的消息类型对应的display项
 * @example 比如一个名为“/test/topic/name”的话题发布的消息类型为“nav_msgs/Path”，那么该话题添加的项应该为“/test ==> /topic ==> /name ==> Path”四层QTreeWidgetItem
 * @attention 本软件支持显示的消息类型没有RViz支持的多，所有支持显示的已在该函数的switch-case语句中罗列。当然笔者是根据需求开发的，用户可以继续使用librviz在qrviz类中添加更多消息类型的显示支持！
 * @param 待添加新项的QTreeWidget treeWidget
 * @param 话题名称name(比如/clicked_point)
 * @param 话题使用的消息类型datatype(比如geometry_msgs/PointStamped)
 */
void AddLayer::treeWidget_ByTopic_addItemByTopic(QTreeWidget *treeWidget, std::string name, std::string datatype)
{
  QString topicName = QString::fromStdString(name);
  QString msgType = QString::fromStdString(datatype);
  //首先查询话题对应的消息类型在本软件是否支持显示
  if(MsgTypeHash.contains(msgType)){
    QStringList temp = topicName.split("/", QString::SkipEmptyParts);   //按照“/”斜杠分割字符串并且结果中不保留分割后得到的空字符串
    QTreeWidgetItem* topLevelItem = new QTreeWidgetItem(QStringList() << temp[0].prepend("/"));   //根据话题名字创建顶层节点
    treeWidget->addTopLevelItem(topLevelItem);      //添加顶层节点
    QTreeWidgetItem* parentItem = topLevelItem;     //使用一个QTreeWidgetItem*保存需要添加子节点的节点
    for (int index = 1; index < temp.size(); index++) {
       QTreeWidgetItem* tempItem = new QTreeWidgetItem(QStringList() << temp[index].prepend("/"));  //根据字符串列表项的数目创建子节点
       parentItem->addChild(tempItem);              //添加子节点
       parentItem = tempItem;                       //更新需要添加子节点的节点
    }
    QTreeWidgetItem* lastItem = new QTreeWidgetItem();
    switch (MsgTypeHash.value(msgType)) {
      case AddLayer::laserscan:
        lastItem->setText(0, "LaserScan");
        lastItem->setIcon(0, QIcon("://rViz/LaserScan.png"));
      break;
      case AddLayer::pointcloud2:
        lastItem->setText(0, "PointCloud2");
        lastItem->setIcon(0, QIcon("://rViz/PointCloud2.png"));
      break;
      case AddLayer::map:
        lastItem->setText(0, "Map");
        lastItem->setIcon(0, QIcon("://rViz/Map.png"));
      break;
      case AddLayer::path:
        lastItem->setText(0, "Path");
        lastItem->setIcon(0, QIcon("://rViz/Path.png"));
      break;
      case AddLayer::pointstamped:
        lastItem->setText(0, "PointStamped");
        lastItem->setIcon(0, QIcon("://rViz/PointStamped.png"));
      break;
      case AddLayer::marker:
        lastItem->setText(0, "Marker");
        lastItem->setIcon(0, QIcon("://rViz/Marker.png"));
      break;
      case AddLayer::markerarray:
        lastItem->setText(0, "MarkerArray");
        lastItem->setIcon(0, QIcon("://rViz/MarkerArray.png"));
      break;
    }
    lastItem->setData(0, Qt::UserRole, topicName);
    parentItem->addChild(lastItem);
  }
}

/**
 * @brief accepted()信号处理槽函数，处理用户确实选中了一个图层并点击了"OK"按钮的操作
 */
void AddLayer::slot_Accepted()
{
  if(ui->tabWidget->currentWidget() == ui->tab_ByDisplayType)
  {
    switch(ui->treeWidget_AllLayer->topLevelItem(0)->indexOfChild(ui->treeWidget_AllLayer->currentItem()))
    {
      case 0:
        LayerCodeToReturn = AddLayer::grid;
      break;
      case 1:
        LayerCodeToReturn = AddLayer::tf;
      break;
      case 2:
        LayerCodeToReturn = AddLayer::laserscan;
      break;
      case 3:
        LayerCodeToReturn = AddLayer::pointcloud2;
      break;
      case 4:
        LayerCodeToReturn = AddLayer::robotmodel;
      break;
      case 5:
        LayerCodeToReturn = AddLayer::map;
      break;
      case 6:
        LayerCodeToReturn = AddLayer::path;
      break;
      case 7:
        LayerCodeToReturn = AddLayer::pointstamped;
      break;
      case 8:
        LayerCodeToReturn = AddLayer::marker;
      break;
      case 9:
        LayerCodeToReturn = AddLayer::markerarray;
      break;
    }
  }
  else if(ui->tabWidget->currentWidget() == ui->tab_ByTopic)
  {
    LayerCodeToReturn = DisplayNameMap.value(ui->treeWidget_ByTopic->currentItem()->text(0));
    TopicToSubscribe = ui->treeWidget_ByTopic->currentItem()->data(0, Qt::UserRole).toString();
  }
  DisplayName = ui->lineEdit_DisplayName->text();
}

/**
 * @brief 获取新增图层方法，该方法是静态成员方法。调用该方法会打开新增图层对话框，用户可选择想要新增的图层以及设置要新增图层的名称
 * @return 一个表明不同图层的int编码
 */
int AddLayer::getLayer(QWidget *parent, const QString &title)
{
  AddLayer addLayer(parent);
  addLayer.setWindowTitle(title);
  addLayer.exec();
  return LayerCodeToReturn;
}

/**
 * @brief 获取图层显示名静态成员方法，如果你的代码中需要使用图层显示名，则可以调用此方法
 * @attention 调用该方法之前你需要先调用getLayer()方法新增一个图层，该方法才能返回有效的值，否则返回空字符串("")
 * @return 图层显示名字符串
 */
QString AddLayer::getDisplayName()
{
  return DisplayName;
}

/**
 * @brief 通过话题信息填充(需要订阅话题的)不同显示(display)类型的话题列表
 */
void AddLayer::topicListAppendByTopicInfo(const ros::master::TopicInfo& info)
{
  switch (MsgTypeHash.value(QString::fromStdString(info.datatype))) {
    case AddLayer::laserscan:
      laserScanTopicList.append(QString::fromStdString(info.name));
    break;
    case AddLayer::pointcloud2:
      pointCloud2TopicList.append(QString::fromStdString(info.name));
    break;
    case AddLayer::map:
      mapTopicList.append(QString::fromStdString(info.name));
    break;
    case AddLayer::path:
      pathTopicList.append(QString::fromStdString(info.name));
    break;
    case AddLayer::pointstamped:
      pointStampedTopicList.append(QString::fromStdString(info.name));
    break;
    case AddLayer::marker:
      markerTopicList.append(QString::fromStdString(info.name));
    break;
    case AddLayer::markerarray:
      markerArrayTopicList.append(QString::fromStdString(info.name));
    break;
  }
}

AddLayer::~AddLayer()
{
  delete ui;
}
