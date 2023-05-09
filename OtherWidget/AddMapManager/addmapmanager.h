#ifndef ADDMAPMANAGER_H
#define ADDMAPMANAGER_H

#include <QDialog>
#include <QFileDialog>

namespace Ui {
class AddMapManager;
}

class AddMapManager : public QDialog
{
  Q_OBJECT

public:
  explicit AddMapManager(QWidget *parent = nullptr);
  static QStringList getMapMessage(QWidget *parent = nullptr, const QString &title = QString("Map Manager Administrator"));
  void slot_Accepted();
  void slot_Modify_buttonBox_status();
  ~AddMapManager();

private:
  Ui::AddMapManager *ui;
  static QStringList mapMsg;
};

#endif // ADDMAPMANAGER_H
