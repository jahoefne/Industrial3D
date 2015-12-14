#include "mainwindow_.h"
#include "ui_mainwindow_.h"

MainWindow_::MainWindow_(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow_)
{
    ui->setupUi(this);
}

MainWindow_::~MainWindow_()
{
    delete ui;
}
