#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QProcess>
#include <QTextStream>
#include <QFileDialog>
#include <QMessageBox>
#include <fstream>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    QProcess *myProcess = new QProcess();

    myProcess->startDetached("/bin/bash", QStringList()<< "/home/zsombi/objtrackGUI/rosnode_kill_script.sh");

     QTextStream(stdout) << "ALL NODES have been killed!"<<endl;

    delete ui;
}

//ROSCORE
void MainWindow::on_commandLinkButton_clicked()
{
       QProcess *myProcess = new QProcess();

       myProcess->startDetached("/bin/bash", QStringList()<< "/home/zsombi/objtrackGUI/roscore.sh");

       QTextStream(stdout) << "Starting ROS master..."<<endl;
       ui->commandLinkButton_2->setEnabled(true); //Openni button activate
       ui->commandLinkButton_3->setEnabled(true); //Rviz button activate
       ui->commandLinkButton->setEnabled(false); //deactivate Roscore button

}

//OPENNI LAUNCH
void MainWindow::on_commandLinkButton_2_clicked()
{
    QProcess *myProcess = new QProcess();

    myProcess->startDetached("/bin/bash", QStringList()<< "/home/zsombi/objtrackGUI/openni2_launch.sh");

    QTextStream(stdout) << "Starting camera node ..."<<endl;

    ui->pushButton->setEnabled(true);
    ui->commandLinkButton_2->setEnabled(false);
}

//Rviz BUTTON
void MainWindow::on_commandLinkButton_3_clicked()
{
    QProcess *myProcess = new QProcess();

    myProcess->startDetached("/bin/bash", QStringList()<< "/home/zsombi/objtrackGUI/rviz.sh");

    QTextStream(stdout) << "Opening RViz ..."<<endl;

}

//START TRACKING ROSRUN
void MainWindow::on_pushButton_clicked()
{
    QString script("/home/zsombi/objtrackGUI/tracking_script.sh");
    //QString command_string = script+ " " +filepath;
    //QString c_option("-c");
    QProcess *myProcess = new QProcess();
    myProcess->startDetached("/bin/bash", QStringList()<<script);
    //qDebug() << myProcess->readAllStandardOutput();

    QTextStream(stdout) << "Tracking has started ..."<<endl;

     ui->pushButton->setEnabled(false);
}



//browse file
void MainWindow::on_pushButton_2_clicked()
{
    QString file_path = QFileDialog::getOpenFileName(this, "Open a PCD model file", "/home/zsombi/");
    QMessageBox::information(this,"..",file_path);
    filepath = file_path;

}

void MainWindow::on_pushButton_3_clicked()
{
  //QProcess *myProcess = new QProcess();

  //QString script("/home/zsombi/objtrackGUI/runpy.sh");
 // myProcess->start("/bin/bash", QStringList()<< script << filepath);

  QTextStream(stdout) << "running shell to execute python script ..."<<endl;



}
