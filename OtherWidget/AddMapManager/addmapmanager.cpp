#include "addmapmanager.h"
#include "ui_addmapmanager.h"

AddMapManager::AddMapManager(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::AddMapManager)
{
  ui->setupUi(this);
  ui->btn_BrowseMap->setIcon(QIcon("://icon/folder(2).svg"));
  mapMsg = QStringList();
  ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);

  /*********************
   *  connect 操作
   *********************/
  connect(ui->btn_BrowseMap, &QPushButton::clicked, this, [=](){
    ui->lineEdit_MapPath->setText(QFileDialog::getOpenFileName(this, "打开文件", "/home", "(*.pgm)"));
  });
  connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &AddMapManager::slot_Accepted);
  connect(ui->lineEdit_MapName, &QLineEdit::textChanged, this, &AddMapManager::slot_Modify_buttonBox_status);
  connect(ui->lineEdit_MapPath, &QLineEdit::textChanged, this, &AddMapManager::slot_Modify_buttonBox_status);
}

QStringList AddMapManager::mapMsg = QStringList();

AddMapManager::~AddMapManager()
{
  delete ui;
}

QStringList AddMapManager::getMapMessage(QWidget *parent, const QString &title)
{
  AddMapManager addMapManager(parent);
  addMapManager.setWindowTitle(title);
  addMapManager.exec();
  return mapMsg;
}

void AddMapManager::slot_Accepted()
{
  mapMsg.append(ui->lineEdit_MapPath->text());
  mapMsg.append(ui->lineEdit_MapName->text());
  mapMsg.append(ui->textEdit_MapDescription->toPlainText());
}

void AddMapManager::slot_Modify_buttonBox_status()
{
  if(!ui->lineEdit_MapPath->text().isEmpty() && !ui->lineEdit_MapName->text().isEmpty())
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
  else
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
}
