#include "shellselect.h"
#include "ui_shellselect.h"

ShellSelect::ShellSelect(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ShellSelect)
{
  ui->setupUi(this);

  /*********************
  ** Initialization
  **********************/
  ReadSettings();
  ui->btn_BrowseShell->setIcon(QIcon("://icon/folder(2).svg"));

  /*********************
  ** Connect Operation
  **********************/
  connect(ui->btn_BrowseShell, &QPushButton::clicked, this, [=](){
    //getOpenFileName()方法的参数1为parent，参数2为窗口标题，参数3为默认打开路径，参数4为文件格式过滤器
    ShellPathStr = qfiledialog.getOpenFileName(this, "打开文件", ShellPathStr, "(*.sh)");
    ui->lineEdit_ShellPath->setText(ShellPathStr);
  });

  connect(ui->btn_AddShell, &QPushButton::clicked, this, [=](){
    if(ui->lineEdit_ShellName->text().isEmpty() || ui->lineEdit_ShellPath->text().isEmpty())
    {
      QMessageBox::critical(this, "错误", "请填写完整信息！");
    }
    else
    {
      //添加项
      ui->ShelllistWidget->addItem(ui->lineEdit_ShellName->text());
      //设置tooltip为脚本路径信息
      ui->ShelllistWidget->item(ui->ShelllistWidget->count() - 1)->setToolTip(ui->lineEdit_ShellPath->text());
      //清空QLineEdit
      ui->lineEdit_ShellName->clear();
    }
  });

  connect(ui->btn_ExecuteShell, &QPushButton::clicked, this, &ShellSelect::on_btn_ExecuteShell_clicked);

  connect(ui->btn_ExecuteShellByTerminal, &QPushButton::clicked, this, [=](){
    //当脚本栏至少有一项被选中时，执行脚本按钮才有效
    if(ui->ShelllistWidget->currentItem() != NULL)
    {
      //同样检查文件是否存在
      QFileInfo fileInfo(ui->ShelllistWidget->currentItem()->toolTip());
      //判断
      if(fileInfo.exists() && fileInfo.isFile())
      {
        QString str("gnome-terminal -- bash -c '%1'&");                   //命令字符串
        str = str.arg(ui->ShelllistWidget->currentItem()->toolTip());     //脚本文件绝对路径插入到命令中
        std::system(str.toLatin1().data());                               //执行命令
      }
      else
      {
        QMessageBox::critical(this, "错误", "未找到该脚本文件，请检查文件是否存在！");
      }
    }
  });
  connect(ui->btn_TerminateExecute, &QPushButton::clicked, this, &ShellSelect::on_btn_TerminateExecute_clicked);
  connect(ui->btn_RemoveShell, &QPushButton::clicked, this, &ShellSelect::on_btn_RemoveShell_clicked);
  connect(ui->btn_clearLog, &QPushButton::clicked, this, &ShellSelect::on_btn_clearLog_clicked);
}

/**
 * @brief 使用配置文件(.ini)读取添加好的shell脚本项
 */
void ShellSelect::ReadSettings()
{
  //构造一个QSettings对象，用于从名为Qt_Shell的组织访问名为qt_shell_demo的应用程序的设置
  QSettings iniRead("Qt-Ros Package", "class1_ros_qt_demo");
  QStringList shell_name_list = iniRead.value("shell_names").toStringList();
  QStringList shell_path_list = iniRead.value("shell_paths").toStringList();
  for(int index = 0; index < shell_name_list.count(); index++)
  {
    ui->ShelllistWidget->addItem(shell_name_list[index]);
    ui->ShelllistWidget->item(index)->setToolTip(shell_path_list[index]);
  }
}

/**
 * @brief 使用配置文件(.ini)保存添加好的shell脚本项
 */
void ShellSelect::WriteSettings()
{
  QSettings iniWrite("Qt-Ros Package", "class1_ros_qt_demo");
  QStringList shell_name_list;
  QStringList shell_path_list;
  for(int index = 0; index < ui->ShelllistWidget->count(); index++)
  {
    //向字符串列表尾部添加新的字符串
    shell_name_list.append(ui->ShelllistWidget->item(index)->text());
    shell_path_list.append(ui->ShelllistWidget->item(index)->toolTip());
  }
  iniWrite.setValue("shell_names", shell_name_list);
  iniWrite.setValue("shell_paths", shell_path_list);
}

void ShellSelect::closeEvent(QCloseEvent *event)
{
  WriteSettings();    //在窗口关闭事件时调用保存配置方法
  QDialog::closeEvent(event);
}

/**
 * @brief “执行脚本”按钮点击处理槽函数
 */
void ShellSelect::on_btn_ExecuteShell_clicked()
{
  //当脚本栏至少有一项被选中时，执行脚本按钮才有效
  if(ui->ShelllistWidget->currentItem() != NULL)
  {
    //首先使用QFileInfo类判断文件是否真的存在于目标目录
    QFileInfo fileInfo(ui->ShelllistWidget->currentItem()->toolTip());
    //判断文件是否存在
    if(fileInfo.exists() && fileInfo.isFile())
    {
      //判断当前脚本是否已经在执行了(因为只有正在执行的脚本进程才会加入哈希表管理，所以查询哈希表有无该进程即可)
      if(processHash.contains(ui->ShelllistWidget->currentItem()))
      {
        QMessageBox::information(this, "提示", "当前脚本已经在执行了~_~");
      }
      else
      {
        //新增一个QProcess对象用于执行当前脚本文件
        QProcess *process = new QProcess();
        //当进程通过其标准输出通道(stdout)提供了新数据时，通过输出栏打印标准输出(注意对象数组成员需要用取地址运算符"&")
        connect(process, &QProcess::readyReadStandardOutput, this, [=](){
          ui->textBrowser_Output->append(QString(process->readAllStandardOutput().constData()));
        });
        connect(process, &QProcess::started, this, [=](){
          /* arg()方法会返回字符串中编号最低的位置标记替换为参数提供的字符串的字符串副本，
           * 比如QString("脚本 %1 已启动").arg("测试脚本")会返回这样一个字符串"脚本 测试脚本 已启动"*/
          ui->textBrowser_Output->append(QString("脚本 %1 已启动").arg(ui->ShelllistWidget->currentItem()->text()));
        });
        connect(process, &QProcess::errorOccurred, this, &ShellSelect::on_errorOccurred);
        //新增的QProcess对象加入哈希表管理
        processHash.insert(ui->ShelllistWidget->currentItem(), process);
        /* shell脚本是一种脚本语言，他不需要编译，而是直接运行解释器，将脚本作为解释器程序的参数运行的。
         * 所以start(const QString &program, const QStringList &arguments, QIODevice::OpenMode mode = ReadWrite)方法
         *    第一个参数“程序”应为脚本解释器程序，而脚本文件(.sh)应当作为运行参数传入(也就是start()方法的第二个参数)
         */
        //processes[currProcessIndex].start("/bin/bash", QStringList() << ui->ShelllistWidget->currentItem()->toolTip());
        process->start("/bin/bash", QStringList() << ui->ShelllistWidget->currentItem()->toolTip());
        /*
         * 当然，我们也可以通过setProgram(const QString &program)方法设置进程要使用的程序；
         * 通过setArguments(const QStringList &arguments)方法设置在启动进程时传递给被调用程序的参数；
         * 最后通过start(QIODevice::OpenMode mode = ReadWrite)启动进程。
         *
         * 比如上面一句代码可以改为下面这样：
         *    executeProcess->setProgram("/bin/bash");
         *    executeProcess->setArguments(QStringList() << ui->ShelllistWidget->currentItem()->toolTip());
         *    executeProcess->start();
         * 但显然，这样写更麻烦一点～
         */
        //等待启动完成
        process->waitForStarted();
      }
    }
    else
    {
      QMessageBox::critical(this, "错误", "未找到该脚本文件，请检查文件是否存在！");
    }
  }
}

/**
 * @brief “终止执行”按钮点击处理槽函数
 */
void ShellSelect::on_btn_TerminateExecute_clicked()
{
  //获取待移除的QListWidget项
  QListWidgetItem* terminateItem = ui->ShelllistWidget->currentItem();
  //检查哈希表是否正在管理当前脚本进程
  if(processHash.contains(terminateItem))
  {
    //根据QListWidget项获取其对应的进程
    QProcess* process = processHash.value(terminateItem);
    //如果进程正在执行，则先终止进程
    if(process->state() == QProcess::Running)
    {
      process->terminate();
      process->waitForFinished();
      ui->textBrowser_Output->append(QString("脚本 %1 已终止运行！").arg(ui->ShelllistWidget->currentItem()->text()));
    }
    //从哈希表中移除管理
    processHash.remove(terminateItem);
    //释放内存
    delete process;
  }
  else
  {
    //因为只有正在执行的脚本进程才会加入哈希表管理，所以查询哈希表有无该进程即可
    QMessageBox::information(this, "提示", "当前脚本没有在运行~_~");
  }
}

/**
 * @brief “移除脚本”按钮点击处理槽函数
 */
void ShellSelect::on_btn_RemoveShell_clicked()
{
  //获取待移除的QListWidget项
  QListWidgetItem* removeItem = ui->ShelllistWidget->currentItem();
  if(processHash.contains(removeItem))
  {
    //根据QListWidget项获取其对应的
    QProcess* process = processHash.value(removeItem);
    if(process->state() == QProcess::Running)
    {
        process->terminate();
        process->waitForFinished();
        ui->textBrowser_Output->append(QString("脚本 %1 已终止运行！").arg(ui->ShelllistWidget->currentItem()->text()));
    }
    processHash.remove(removeItem);
    delete process;
  }
  ui->ShelllistWidget->takeItem(ui->ShelllistWidget->row(removeItem));      //移除项(UI层面)
  delete removeItem;    //delete释放内存是必要的，避免内存泄漏
}

/**
 * @brief QProcess错误处理槽函数
 */
void ShellSelect::on_errorOccurred(QProcess::ProcessError errorcode)
{
  switch (errorcode)
  {
    case QProcess::FailedToStart:
      ui->textBrowser_Output->append(QString("进程启动失败，未找到程序或者缺少可执行权限"));
    break;
    case QProcess::Crashed:
      ui->textBrowser_Output->append(QString("进程运行中发生崩溃"));
    break;
    case QProcess::Timedout:
      ui->textBrowser_Output->append(QString("启动超时"));
    break;
    case QProcess::ReadError:
      ui->textBrowser_Output->append(QString("尝试从进程中读取时发生错误"));
    break;
    case QProcess::WriteError:
      ui->textBrowser_Output->append(QString("尝试向进程中写入时发生错误"));
    break;
    case QProcess::UnknownError:
      ui->textBrowser_Output->append(QString("未知错误"));
    break;
  }
}

/**
 * @brief 清除日志栏槽函数
 */
void ShellSelect::on_btn_clearLog_clicked()
{
  ui->textBrowser_Output->clear();
}

ShellSelect::~ShellSelect()
{
  delete ui;
}
