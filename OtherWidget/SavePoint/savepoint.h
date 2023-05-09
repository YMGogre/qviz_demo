#ifndef SAVEPOINT_H
#define SAVEPOINT_H

#include <QDialog>
#include <QString>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QJsonParseError>
#include <QFile>
#include <QMessageBox>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

namespace Ui {
class SavePoint;
}
class StandardPoint;    //前向声明

class SavePoint : public QDialog
{
  Q_OBJECT

signals:
  void pointUpdated(StandardPoint& newPoint);

public:
  explicit SavePoint(QString _fileStoragePath, double x, double y, QWidget *parent = nullptr);  //构造函数中_fileStoragePath参数应传递地图文件(yaml文件)的完整路径(比如"/home/xxx/map/map1.yaml")
  static void savePoint(QString _fileStoragePath, double x, double y);
  void generateJSONandSave();
  ~SavePoint();

private:
  Ui::SavePoint *ui;
  double coordinate_x;
  double coordinate_y;
};

/**
 * @brief 使用StandardPoint(标准点)类来描述要保存的点
 */
class StandardPoint{
public:
  explicit StandardPoint();
  explicit StandardPoint(QString pointName, double x, double y);
  inline double x(){ return _x; }
  inline double y(){ return _y; }
  inline QString pointName(){ return _pointName; }
private:
  QString _pointName;
  double _x;
  double _y;
};

#endif // SAVEPOINT_H
