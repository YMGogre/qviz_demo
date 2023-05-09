#include "mypalette.h"
#include "ui_mypalette.h"

MyPalette::MyPalette(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MyPalette)
{
    ui->setupUi(this);
    //QGraphicsView的鼠标事件会传递给它所显示的QGraphicsScene，然后再传递给场景中的QGraphicsItem。我们设置该控件属性：传递给父对象做鼠标事件处理
    ui->ColorDisplay->setAttribute(Qt::WA_TransparentForMouseEvents);
    /*****************************************
     * 为自定义控件的子控件安装该自定义控件的事件过滤器
     ****************************************/
    ui->ColorLineEdit->installEventFilter(this);
    ui->btn_SelectColor->installEventFilter(this);
    /*使用正则表达式限制输入(正确输入格式应当形如 ———— "255; 255; 255"，最后允许跟随一个分号以及一个0~255的alpha通道值)
     * (2[0-4][0-9];?\\s?)  ———— 限制输入200～249正整数(尾部允许跟随0～1个分号";"和空格" ")
     * (25[0-5];?\\s?)      ———— 限制输入250～255正整数(尾部允许跟随0～1个分号";"和空格" ")
     * (1[0-9][0-9];?\\s?)  ———— 限制输入100～199正整数(尾部允许跟随0～1个分号";"和空格" ")
     * ([1-9]?[0-9];?\\s?)  ———— 限制输入0～99正整数(尾部允许跟随0～1个分号";"和空格" ")
    */
    ui->ColorLineEdit->setValidator(new QRegExpValidator(QRegExp("^((2[0-4][0-9];\\s?)|(25[0-5];\\s?)|(1[0-9][0-9];\\s?)|([1-9]?[0-9];\\s?)){0,3}((2[0-4][0-9])|(25[0-5])|(1[0-9][0-9])|([1-9]?[0-9]))?$")));

    connect(ui->ColorLineEdit, &QLineEdit::textChanged, this, [=](){
        //使用正则表达式删除字符串中所有空格
        QString colorStr = ui->ColorLineEdit->text().remove(QRegExp("\\s"));
        //使用split方法按";"分号分割字符串为字符串数组
        QStringList colorStrList = colorStr.split(";");
        //当LineEdit只有rgb的值时
        if(colorStrList.size() == 3)
        {
            //使用字符串数组内容初始化一个QColor对象并赋值给color成员变量
            color = QColor(colorStrList[0].toInt(), colorStrList[1].toInt(), colorStrList[2].toInt());
            //当颜色改变时，发送“颜色改变信号”
            emit colorchanged(color);
        }
        //当LineEdit除了rgb的值还有alpha通道的值时
        else if(colorStrList.size() == 4)
        {
            //使用字符串数组内容初始化一个QColor对象并赋值给color成员变量
            color = QColor(colorStrList[0].toInt(), colorStrList[1].toInt(), colorStrList[2].toInt(), colorStrList[3].toInt());
            //当颜色改变时，发送“颜色改变信号”
            emit colorchanged(color);
        }
        //获取调色板
        QPalette pal = ui->ColorDisplay->palette();
        //设置调色板
        pal.setColor(QPalette::Base, color);
        //设置控件底色
        ui->ColorDisplay->setPalette(pal);
    });

    //当文本框编辑结束时，根据color成员变量显示其RGB字符串
    connect(ui->ColorLineEdit, &QLineEdit::editingFinished, this, [=](){
        //当alpha通道值为255(默认)时，LineEdit默认不显示alpha通道值
        if(color.alpha() == 255)
        {
            QString colorStr = QString::number(color.red()) + "; " + QString::number(color.green()) + "; " + QString::number(color.blue());
            ui->ColorLineEdit->setText(colorStr);
        }
        else
        {
            QString colorStr = QString::number(color.red()) + "; " + QString::number(color.green()) + "; " + QString::number(color.blue()) + "; " + QString::number(color.alpha());
            ui->ColorLineEdit->setText(colorStr);
        }
    });

    connect(ui->btn_SelectColor, &QPushButton::clicked, this, [=](){
        //打开一个颜色对话框并将其返回值交给局部变量_color
        QColor _color = QColorDialog::getColor(Qt::white, this);
        //判断返回的颜色是否有效
        if(_color.isValid())
        {
            //有效则改变颜色并发送信号
            color = _color;
            //当颜色改变时，发送“颜色改变信号”
            emit colorchanged(color);
        }
        //根据对话框返回的颜色修改颜色展示方块的颜色
        QPalette pal = ui->ColorDisplay->palette();
        //设置调色板
        pal.setColor(QPalette::Base, color);
        //设置控件底色
        ui->ColorDisplay->setPalette(pal);
        //根据对话框返回的颜色获取RGB三色字符串
        QString colorStr = QString::number(color.red()) + "; " + QString::number(color.green()) + "; " + QString::number(color.blue());
        ui->ColorLineEdit->setText(colorStr);
    });
}

/**
 * @brief 获取当前颜色方法
 * @return QColor   ———— 返回当前QColor颜色对象
 */
QColor MyPalette::currentColor()
{
    return color;
}

/**
 * @brief 设置颜色方法
 * @param int r ———— 红色
 * @param int g ———— 绿色
 * @param int b ———— 蓝色
 * @param int a ———— Alpha通道值(0完全透明～255完全不透明)
 */
void MyPalette::setColor(int r, int g, int b, int a)
{
    if(a == 255)
    {
        //设置RGB三色字符串
        QString colorStr = QString::number(r) + "; " + QString::number(g) + "; " + QString::number(b);
        //setText()方法也会触发QLineEdit的textChanged信号
        ui->ColorLineEdit->setText(colorStr);
    }
    else
    {
        //设置RGB三色字符串
        QString colorStr = QString::number(r) + "; " + QString::number(g) + "; " + QString::number(b) + "; " + QString::number(a);
        //setText()方法也会触发QLineEdit的textChanged信号
        ui->ColorLineEdit->setText(colorStr);
    }
}

/**
 * @brief 设置颜色方法
 * @param QColor color ———— QColor颜色对象
 */
void MyPalette::setColor(QColor color)
{
    if(color.alpha() == 255)
    {
        //设置RGB三色字符串
        QString colorStr = QString::number(color.red()) + "; " + QString::number(color.green()) + "; " + QString::number(color.blue());
        //setText()方法也会触发QLineEdit的textChanged信号
        ui->ColorLineEdit->setText(colorStr);
    }
    else
    {
        //设置RGB三色字符串
        QString colorStr = QString::number(color.red()) + "; " + QString::number(color.green()) + "; " + QString::number(color.blue()) + "; " + QString::number(color.alpha());
        //setText()方法也会触发QLineEdit的textChanged信号
        ui->ColorLineEdit->setText(colorStr);
    }
}

/**
 * @brief 重写事件过滤器方法
 * @attention 这里简要说下为什么要重写该方法，这是因为当我们的自定义控件中有其他子控件时，
 * 点击子控件的点击响应是相应到子控件上的，而通常我们希望所有子控件的父对象(也就是自定义控
 * 件本身)也会在点击这些子控件时有响应
 * @param QMouseEvent *event ———— 事件对象
 */
bool MyPalette::eventFilter(QObject *watched, QEvent *event)
{
  //拦截子控件的鼠标按下或释放事件
  if(event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseButtonRelease)
  {
    //QEvent类型转换为QMouseEvent类型
    QMouseEvent *mouseevent = static_cast<QMouseEvent *>(event);
    this->QWidget::mousePressEvent(mouseevent);    //调用基类的mousePressEvent方法
    //如果事件的被监视对象是QPushButton
    if(qobject_cast<QPushButton *>(watched))
      this->setFocus();     //设置自定义控件本身获得焦点

    return false;     //传递事件给子控件，让其正常响应点击
    //return true;    //事件处理完毕(不传递事件给子控件，只响应自定义控件本身被选中)
  }
  //其他类型的事件交由基类处理
  return QWidget::eventFilter(watched, event);
}

MyPalette::~MyPalette()
{
    delete ui;
}
