#include "mainwindow_.h"
#include "ui_mainwindow_.h"
#include <QFileDialog>
#include "Point3D.h"
#include <string.h>
#include <climits>
#include <cmath>
#include "PointCloud.h"

PointCloud *Cloud;

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
void MainWindow_::on_actionOpen_Dataset_triggered()
{
    Cloud = new PointCloud();
    Cloud->loadPointsFromFile("../data/bunny.xyz", 0, 0, 0);
}

void MainWindow_::on_actionQuit_Application_triggered()
{
    qApp->exit();
}

void MainWindow_::on_rangeButton_clicked()
{

}

void MainWindow_::on_smoothButton_clicked()
{

    Cloud->smooth(0.01);
}



