#include "savepoint.h"
#include "ui_savepoint.h"

/**
 * @brief 保存点位类构造函数参数需提供一个文件存储路径，两个double坐标值，只能存储2维平面上的点(不存储z轴)
 * @param yaml文件的路径_fileStoragePath
 */
SavePoint::SavePoint(QString _fileStoragePath, double x, double y, QWidget *parent) :
  QDialog(parent),
  ui(new Ui::SavePoint),
  coordinate_x(x),
  coordinate_y(y)
{
  ui->setupUi(this);
  if(_fileStoragePath.isEmpty()){             //如果提供的文件存储路径参数是空字符串(说明地图服务不是在本软件内启动的)
    struct passwd *pw = getpwuid(getuid());   //通过获取用户的密码条目从其中获取用户的主目录
    const char *homedir = pw->pw_dir;
    QString defaultPath = QString::fromUtf8(homedir).append("/UnknownMap.json");
    ui->lineEdit_fileStoragePath->setText(defaultPath);
    QMessageBox::warning(parent, "警告", "并未获取到正确的地图文件路径(地图服务是否是在本软件内启动的?)，当前点位会暂存在根目录下，您可以在保存后将其剪切到对应地图文件所在的目录");
  }
  else
    ui->lineEdit_fileStoragePath->setText(_fileStoragePath.remove((_fileStoragePath.length() - 5), 5).append("Point.json"));  //显示json文件的存储位置
  QString coordinate("[%1, %2]");
  coordinate = coordinate.arg(QString::number(x, 'f', 2), QString::number(y, 'f', 2));  //根据构造函数参数提供的点显示坐标
  ui->lineEdit_pointCoordinate->setText(coordinate);

  /*********************
  ** Connect Operation
  **********************/
  connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &SavePoint::generateJSONandSave);
}

SavePoint::~SavePoint()
{
  delete ui;
}

void SavePoint::savePoint(QString _fileStoragePath, double x, double y)
{
  SavePoint s(_fileStoragePath, x, y);
  s.exec();
}

/**
 * @brief 将点位对象保存为Json格式到文本文件，保存的json格式为{"pointName" : [x, y]}，每个Json对象保存为一行，读文件时按行读取
 */
void SavePoint::generateJSONandSave()
{
  QJsonArray coordinate;                                          //创建Json数组
  coordinate.append(coordinate_x);                                //数组插入值
  coordinate.append(coordinate_y);
  QJsonObject pointJson;                                          //创建Json对象
  pointJson.insert("pointName", ui->lineEdit_pointName->text());  //插入键值对
  pointJson.insert("coordinate", coordinate);
  QJsonDocument doc(pointJson);                                   //构建Json文档
  QByteArray data = doc.toJson(QJsonDocument::Compact) + "\n";    //根据Json文档转字节数组(因为不会自动换行，所以最后补个换行符)
  QFile file(ui->lineEdit_fileStoragePath->text());               //打开一个文件(如果文件不存在则先创建该文件)
  if(file.open(QIODevice::WriteOnly | QIODevice::Append)){
    //写入数据
    file.write(data);
    file.close();
  }
  StandardPoint newPoint(ui->lineEdit_pointName->text(), coordinate_x, coordinate_y);
  emit pointUpdated(newPoint);
}

StandardPoint::StandardPoint(){}
StandardPoint::StandardPoint(QString pointName, double x, double y) : _pointName(pointName), _x(x), _y(y) {}
