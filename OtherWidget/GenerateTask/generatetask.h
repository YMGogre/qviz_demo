#ifndef GENERATETASK_H
#define GENERATETASK_H

#include <QDialog>
#include <QPushButton>
#include <nav_msgs/Path.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QJsonParseError>
#include <QUuid>
#include <QFile>
#include <QDebug>
#include <QMessageBox>
#include <QString>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <QException>
#include "../../include/qviz_demo/qtask.h"

namespace Ui {
class GenerateTask;
}

class GenerateTask : public QDialog
{
  Q_OBJECT
signals:
  void taskUpdated(QString taskname);

public:
  explicit GenerateTask(nav_msgs::Path path, QVector<int> modeTypes, QString _fileStoragePath, QWidget *parent = nullptr);  //构造函数中_fileStoragePath参数应传递地图文件(yaml文件)的完整路径(比如"/home/xxx/map/map1.yaml")
  static void saveTaskToFile(nav_msgs::Path, QVector<int> modeTypes, QString, QWidget *parent = nullptr, const QString &title = QString("Generate Task"));
  void slot_Modify_buttonBox_status();
  void generateJSONandSave();
  void setStartPointName(const QString& _startPointName);
  void setEndPointName(const QString& _endPointName);
  inline QString marchModeParse(int modeType){
    switch (modeType) {
      case 0:
        return "停止";
      case 1:
        return "前进";
      case -1:
        return "后退";
      case 30:
        return "左移";
      case -30:
        return "右移";
      default:
        return "未知运动模式";
    }
  }
  ~GenerateTask();

private:
  Ui::GenerateTask *ui;
  nav_msgs::Path path;
  QVector<int> modeTypes;
  QStringList getTaskMessage();
};

#endif // GENERATETASK_H
