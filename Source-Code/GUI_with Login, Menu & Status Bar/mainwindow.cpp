#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "secdialog.h"

#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QMessageBox>
#include <QGraphicsView>
#include <QStackedWidget>
#include <QGraphicsScene>
#include <QDataStream>
#include <QFile>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->statusBar->addPermanentWidget(ui->label_2);
     ui->statusBar->addPermanentWidget(ui->progressBar);

//    QPixmap file("C:\Data\Edit\Nayeem.jpg");
//        QStackedWidget  *temp = new QStackedWidget();
//        QGraphicsScene *scene = new QGraphicsScene(this);
//        QGraphicsView *view = new QGraphicsView(scene);
//            QGraphicsPixmapItem img(file);
//            scene.addItem(&img);
//            temp->addWidget(&view);
//            setCentralWidget(view);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_pressed()
{
    ui->label->setText("WE ARE THE UNICORNS");
}

void MainWindow::on_actionNew_Project_triggered()
{

    QString filepath = QFileDialog::getOpenFileName(this, QString("Open Image"), QString(""), QString("Images (*.png *.jpg *.bmp)"));
   if(QString::compare(filepath,QString())!=0)
   {
      QImage im;
      bool valid = im.load(filepath);
      if (valid)
      {
        im = im.scaledToWidth(ui-> image -> width(), Qt::SmoothTransformation );

        ui-> image ->setPixmap(QPixmap :: fromImage (im));

        QLabel label;
      //  QPixmap pixmap("C:\Data\Edit\Nayeem1.jpg");
//        label.setPixmap(pixmap);
//        label.setMask(pixmap.mask());

        label.show();
       //ui-> label-> im (1,0);
        //gridLayout->addWidget(new QPushButton("Start scan"), 0, 0);
       }
       else
      {
         //
      }


    }
}

void MainWindow::on_pushButton_2_clicked()
{

    Dialog newwindow;
    newwindow.setModal(true);
    newwindow.exec();
  // hide();
//    secDialog = new secDialog(this);
//    secDialog-> show();

}


void MainWindow::on_pushButton_5_clicked()
{


}

void MainWindow::on_pushButton_Login_clicked()
{
    QString username = ui->lineEdit_username->text();
        QString password = ui->lineEdit_password->text();

        if(username ==  "unicorn" && password == "unicorn")
        {
            QMessageBox::information(this, "Login", "Username and password is correct");

            //status bar with 5 seconds display
            ui->statusBar->showMessage("Username and password is correct", 5000);

            //status bar with label

            ui->label_2->setText("Username and password is correct");
            //hide();
           // dialog = new secDialog(this);
      //    dialog ->show();
        }
        else
        {
            QMessageBox::warning(this,"Login", "Username and password is not correct");
              ui->statusBar->showMessage("Username and password is not correct", 5000);
              ui->label_2->setText("Username and password is not correct");
        }


}
