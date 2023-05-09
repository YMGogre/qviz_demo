#ifndef SHELLSELECT_H
#define SHELLSELECT_H

#include <QDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QSettings>
#include <QProcess>
#include <QListWidgetItem>

namespace Ui {
class ShellSelect;
}

class ShellSelect : public QDialog
{
  Q_OBJECT

public:
  explicit ShellSelect(QWidget *parent = nullptr);
  void ReadSettings();
  void WriteSettings();
  void closeEvent(QCloseEvent *event);
  void on_btn_ExecuteShell_clicked();
  void on_btn_TerminateExecute_clicked();
  void on_btn_RemoveShell_clicked();
  void on_errorOccurred(QProcess::ProcessError errorcode);
  void on_btn_clearLog_clicked();
  ~ShellSelect();

private:
  Ui::ShellSelect *ui;
  QString ShellPathStr = "/home";
  QFileDialog qfiledialog;
  QHash<QListWidgetItem *, QProcess *>processHash;        //使用一个哈希表管理QProcess对象
};

#endif // SHELLSELECT_H
