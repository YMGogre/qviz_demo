#include "generatetask.h"
#include "ui_generatetask.h"


/**
 * @brief 由nav_msgs::Path类型的路径(path)变量初始化成员变量的构造函数(单个任务对应单条路径，所以创建任务对象时需要传递路径对象)
 * @param 行进模式动态数组（要求与path中的poses等长）
 * @param yaml文件的路径_fileStoragePath
 */
GenerateTask::GenerateTask(nav_msgs::Path path, QVector<int> modeTypes, QString _fileStoragePath, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::GenerateTask)
{
  ui->setupUi(this);
  //设置tablewidget所有列自动拉伸
  ui->tableWidget_points->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  //"Ok"按钮默认无效
  ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
  if(_fileStoragePath.isEmpty()){             //如果提供的文件存储路径参数是空字符串(说明地图服务不是在本软件内启动的)
    struct passwd *pw = getpwuid(getuid());   //通过获取用户的密码条目从其中获取用户的主目录
    const char *homedir = pw->pw_dir;
    QString defaultPath = QString::fromUtf8(homedir).append("/UnknownMap.json");
    ui->fileStoragePath->setText(defaultPath);
    QMessageBox::warning(parent, "警告", "并未获取到正确的地图文件路径(地图服务是否是在本软件内启动的?)，当前生成的任务会暂存在根目录下，您可以在保存后将其剪切到对应地图文件所在的目录");
  }
  else
    ui->fileStoragePath->setText(_fileStoragePath.remove((_fileStoragePath.length() - 5), 5).append("Task.json")); //显示json文件的存储位置

  //成员变量由构造函数的参数初始化
  this->path = path;
  this->modeTypes = modeTypes;
  this->modeTypes.pop_front();      //队首出栈
  this->modeTypes.push_back(0);     //因为每条路线的终点机器人的运动模式一定是停下，所以最后要补个"0"(停止)控制字
  //从path中获取点位信息并显示出来
  int posesLength = this->path.poses.size();
  if(posesLength == 0){}
  else if (posesLength == 1) {    //只由一个点组成的路径终点即起点
    int row = ui->tableWidget_points->rowCount();
    ui->tableWidget_points->insertRow(row);
    ui->tableWidget_points->setItem(row, 0, new QTableWidgetItem("起点"));
    ui->tableWidget_points->setItem(row, 1, new QTableWidgetItem(QString::number(this->path.poses[0].pose.position.x, 'f')));
    ui->tableWidget_points->setItem(row, 2, new QTableWidgetItem(QString::number(this->path.poses[0].pose.position.y, 'f')));
    ui->tableWidget_points->setItem(row, 3, new QTableWidgetItem(marchModeParse(0)));
    row++;
    ui->tableWidget_points->insertRow(row);
    ui->tableWidget_points->setItem(row, 0, new QTableWidgetItem("终点"));
    ui->tableWidget_points->setItem(row, 1, new QTableWidgetItem(QString::number(this->path.poses[0].pose.position.x, 'f')));
    ui->tableWidget_points->setItem(row, 2, new QTableWidgetItem(QString::number(this->path.poses[0].pose.position.y, 'f')));
    ui->tableWidget_points->setItem(row, 3, new QTableWidgetItem(marchModeParse(0)));
  }
  else{                           //其余点划分起点、拐点和终点
    for (int row = 0; row < posesLength; row++) {
      ui->tableWidget_points->insertRow(row);
    }
    ui->tableWidget_points->setItem(0, 0, new QTableWidgetItem("起点"));
    ui->tableWidget_points->setItem(0, 1, new QTableWidgetItem(QString::number(this->path.poses[0].pose.position.x, 'f')));
    ui->tableWidget_points->setItem(0, 2, new QTableWidgetItem(QString::number(this->path.poses[0].pose.position.y, 'f')));
    ui->tableWidget_points->setItem(0, 3, new QTableWidgetItem(marchModeParse(this->modeTypes[0])));
    for (int index = 1; index < posesLength - 1; index++) {
      ui->tableWidget_points->setItem(index, 0, new QTableWidgetItem("拐点"));
      ui->tableWidget_points->setItem(index, 1, new QTableWidgetItem(QString::number(this->path.poses[index].pose.position.x, 'f')));
      ui->tableWidget_points->setItem(index, 2, new QTableWidgetItem(QString::number(this->path.poses[index].pose.position.y, 'f')));
      ui->tableWidget_points->setItem(index, 3, new QTableWidgetItem(marchModeParse(this->modeTypes[index])));
    }
    ui->tableWidget_points->setItem(posesLength - 1, 0, new QTableWidgetItem("终点"));
    ui->tableWidget_points->setItem(posesLength - 1, 1, new QTableWidgetItem(QString::number(this->path.poses[posesLength - 1].pose.position.x, 'f')));
    ui->tableWidget_points->setItem(posesLength - 1, 2, new QTableWidgetItem(QString::number(this->path.poses[posesLength - 1].pose.position.y, 'f')));
    ui->tableWidget_points->setItem(posesLength - 1, 3, new QTableWidgetItem(marchModeParse(0)));
  }

  //生成一个新的UUID(通用唯一标识码)
  QUuid uuid = QUuid::createUuid();
  //id栏展示唯一标识码
  ui->id->setText(uuid.toString());

  /*********************
  ** Connect Operation
  **********************/
  connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &GenerateTask::generateJSONandSave);
  connect(ui->lineEdit_taskName, &QLineEdit::textChanged, this, &GenerateTask::slot_Modify_buttonBox_status);
  connect(ui->comboBox_start, &QComboBox::currentTextChanged, this, &GenerateTask::slot_Modify_buttonBox_status);
  connect(ui->comboBox_finish, &QComboBox::currentTextChanged, this, &GenerateTask::slot_Modify_buttonBox_status);
  connect(ui->textEdit_description, &QTextEdit::textChanged, this, &GenerateTask::slot_Modify_buttonBox_status);
}

void GenerateTask::saveTaskToFile(nav_msgs::Path path, QVector<int> modeTypes, QString _fileStoragePath, QWidget *parent, const QString &title)
{
  GenerateTask generateTask(path, modeTypes, _fileStoragePath, parent);
  generateTask.setWindowTitle(title);
  generateTask.exec();
}

/**
 * @brief 获取任务信息
 */
QStringList GenerateTask::getTaskMessage()
{
  return QStringList() << ui->id->text()
                       << ui->lineEdit_taskName->text()
                       << ui->comboBox_start->currentText()
                       << ui->comboBox_finish->currentText()
                       << ui->textEdit_description->toPlainText();
}

/**
 * @brief 创建JSON文档并保存到JSON文件中
 */
void GenerateTask::generateJSONandSave()
{
  QStringList strList = getTaskMessage();
  class1_ros_qt_demo::QTaskHandle th;
  if(th.query_task_by_name(strList[1])){
    if(QMessageBox::question(this, "命名冲突", "任务名称已被占用，是否仍要保存？(同名任务可能会导致解析错误)") == QMessageBox::No){
      return;
    }
  }
  //构建坐标x和坐标y和行进方式的 JSON 数组
  QJsonArray ArrX;
  QJsonArray ArrY;
  QJsonArray ArrMarchMode;
  QJsonArray ArrPreviousTask;
  try{
    for (int index = 0; index < path.poses.size(); index++) {
      ArrX.append(path.poses[index].pose.position.x);
      ArrY.append(path.poses[index].pose.position.y);
      ArrMarchMode.append(modeTypes[index]);
      ArrPreviousTask.append(0);
    }
  }catch(QException e){
    qDebug() << e.what();
  }
  ArrPreviousTask.pop_back();
  ArrPreviousTask.push_back(60);

  /*
  {
    "id" : "xxx"
    "name" : "工装任务"
    "description" : "由产线1到工作台1的任务路线"
    "start" : "产线1"
    "finish" : "工作台1"
    "x" : [
      1.32,
      2.41,
      3.55,
      4.68
    ]
    "y" : [
      2.14,
      7.26,
      1.03,
      5.56
    ]
  }
  */
  //构建任务 JSON 对象
  QJsonObject taskJson;
  taskJson.insert("id", strList[0]);
  taskJson.insert("name", strList[1]);
  taskJson.insert("start", strList[2]);
  taskJson.insert("finish", strList[3]);
  taskJson.insert("description", strList[4]);
  taskJson.insert("x", QJsonValue(ArrX));
  taskJson.insert("y", QJsonValue(ArrY));
  taskJson.insert("mode_type", QJsonValue(ArrMarchMode));
  taskJson.insert("previous_task", QJsonValue(ArrPreviousTask));

  //创建任务对象并入栈
  th.push_back_task(class1_ros_qt_demo::QTask(taskJson));
  emit taskUpdated(strList[1]);

  //构建任务 JSON 文档
  QJsonDocument doc(taskJson);

  //将 JSON 文档转换为字节数组
  QByteArray data = doc.toJson(QJsonDocument::Compact) + "\n";

  //打开一个文件(如果没有，该方法会先创建文件再打开)
  QFile file(ui->fileStoragePath->text());
  if(file.open(QIODevice::WriteOnly | QIODevice::Append)){
    //写入数据
    file.write(data);
    file.close();
  }
  this->accept();
}

void GenerateTask::slot_Modify_buttonBox_status()
{
  if(ui->lineEdit_taskName->text().isEmpty() || ui->comboBox_start->currentText().isEmpty() || ui->comboBox_finish->currentText().isEmpty() || ui->textEdit_description->toPlainText().isEmpty())
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
  else
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
}

/**
 * @brief 设置起点名称方法
 */
void GenerateTask::setStartPointName(const QString& _startPointName)
{
  ui->comboBox_start->setCurrentText(_startPointName);
}

/**
 * @brief 设置终点名称方法
 */
void GenerateTask::setEndPointName(const QString& _endPointName)
{
  ui->comboBox_finish->setCurrentText(_endPointName);
}

GenerateTask::~GenerateTask()
{
  delete ui;
}
