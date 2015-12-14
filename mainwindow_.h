#ifndef MAINWINDOW__H
#define MAINWINDOW__H

#include <QMainWindow>

namespace Ui {
class MainWindow_;
}

class MainWindow_ : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow_(QWidget *parent = 0);
    ~MainWindow_();

private:
    Ui::MainWindow_ *ui;
};

#endif // MAINWINDOW__H
