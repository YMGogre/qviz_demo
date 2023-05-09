#ifndef MYPALETTE_H
#define MYPALETTE_H

#include <QObject>
#include <QWidget>
#include <QColorDialog>
#include <QColor>
#include <QPalette>
#include <QString>
#include <QRegExpValidator>
#include <QtUiPlugin/QDesignerExportWidget>
#include <QEvent>
#include <QMouseEvent>
#include <QPushButton>

namespace Ui {
class MyPalette;
}

class QDESIGNER_WIDGET_EXPORT MyPalette : public QWidget
{
    Q_OBJECT

public:
    explicit MyPalette(QWidget *parent = nullptr);
    ~MyPalette();
    QColor currentColor();
    void setColor(int r, int g, int b, int a = 255);
    void setColor(QColor color);

protected:
    bool eventFilter(QObject *watchedm, QEvent *event) override;    //重写事件过滤器方法

signals:
    void colorchanged(QColor);              //自定义颜色改变信号

private:
    Ui::MyPalette *ui;
    QColor color = QColor(Qt::white);
};

#endif // MYPALETTE_H
