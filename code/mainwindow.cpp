#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDir>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->StartStop->setStyleSheet("background-color:red");

    ui->label_2->setText(QDir::currentPath());

    ConfigApp();
    InitValues();
    ConfigIo();
    CreateObjects();
    Connections();
    ConnectToServer();

    digitalWrite(16,LOW);
    digitalWrite(12,LOW);
    emit this->StartStart();
}

MainWindow::~MainWindow()
{
    digitalWrite(16,LOW);
    digitalWrite(12,LOW);
    softPwmWrite(29,0);
    digitalWrite(29,LOW);
    softPwmWrite(12,0);
    softPwmWrite(16,0);
    delete ui;
}

void MainWindow::ConfigApp()
{
    qDebug() << "main thread --> make main thread config";
    QSettings ini("config.ini",QSettings::IniFormat);
    _ioFan = ini.value("pinout/fan_io_pin",0).toInt();
    _ioElm = ini.value("pinout/element_io_pin",0).toInt();
    _cs = ini.value("pinout/cs_io_pin",0).toInt();
    _sclk = ini.value("pinout/sclk_io_pin",0).toInt();
    _miso = ini.value("pinout/miso_io_pin",0).toInt();
    _mosi = ini.value("pinout/mosi_io_pin",0).toInt();

    _setpoint = ini.value("app/set_point",0).toDouble();
    _max = ini.value("app/max_point",0).toDouble();
    _min = ini.value("app/min_point",0).toDouble();

    _Rnominal = ini.value("rtd/rnominal",0).toDouble();
    _Rref = ini.value("rtd/rref",0).toDouble();

    _delay = ini.value("app/temp_check_timer",0).toInt();

    _ServerIP = ini.value("Network/server_ip","0""").toString();
    _ServerPort = ini.value("Network/server_port",0).toInt();


    _kp = ini.value("pid/Kp",0).toDouble();
    _ki = ini.value("pid/Ki",0).toDouble();
    _kd = ini.value("pid/Kd",0).toDouble();

    qDebug() << "main thread --> Server IP :" << _ServerIP;
    qDebug() << "main thread --> Server Port :" << _ServerPort;


    qDebug() << "main thread --> fan pin:" << _ioFan;
    qDebug() << "main thread --> heater pin:" << _ioElm;

    socketState = -1;


    qDebug() << "main thread --> cs pin:" << _cs;
    qDebug() << "main thread --> sclk pin:" << _sclk;
    qDebug() << "main thread --> miso pin:" << _miso;
    qDebug() << "main thread --> mosi pin:" << _mosi;

    qDebug() << "main thread --> delay: " << _delay;
    qDebug() << "main thread --> setpoint: " << _setpoint;
    qDebug() << "main thread --> maxpoint: " << _max;
    qDebug() << "main thread --> minpoint: " << _min;

    qDebug() << "main thread --> Kp: " << _kp;
    qDebug() << "main thread --> Ki: " << _ki;
    qDebug() << "main thread --> Kd: " << _kd;

}

void MainWindow::InitValues()
{
    qDebug() << "main thread --> make init values";
    fan = false;
    elm = false;
    start_stop = false;

    connect_toHost = false;
}

void MainWindow::ConfigIo()
{
    qDebug() << "main thread --> make input output pins";
    wiringPiSetupPhys();

    qDebug() << "softPwmWrite heater = " << softPwmCreate(12,0,100);
    qDebug() << "softPwmWrite cooler = " << softPwmCreate(16,0,100);

    softPwmWrite(12,0);
    softPwmWrite(16,0);

    pinMode(29,OUTPUT);
    pullUpDnControl(27,PUD_UP);
    digitalWrite(29,LOW);
}

void MainWindow::CreateObjects()
{
    myRtdHandler = new RtdHandler;
    myRtdThread = new QThread;

    socket = new QTcpSocket(this);
    socketState = 0;
}

void MainWindow::Connections()
{
    qDebug() << "main thread --> make signal slot connection";

    connect(this,SIGNAL(StartStart()),
            this,SLOT(StartApp()),
            Qt::QueuedConnection);

    connect(this,SIGNAL(ConfigRtdHandle(bool,int,int,int,int,int,double,double,int,int,double,double,double,double)),
            myRtdHandler,SLOT(GetConfigFromMain(bool,int,int,int,int,int,double,double,int,int,double,double,double,double)),
            Qt::QueuedConnection);

    connect(this,SIGNAL(StartStopRtdThread(bool)),
            myRtdHandler,SLOT(On_Off_Slot(bool)),
            Qt::QueuedConnection);

    qRegisterMetaType<QAbstractSocket::SocketState>();
    connect(socket,SIGNAL(stateChanged(QAbstractSocket::SocketState)),
            this,SLOT(SocketStateChange(QAbstractSocket::SocketState)),
            Qt::QueuedConnection);
    
    connect(socket,SIGNAL(readyRead()),
            this,SLOT(ReadSocket()),
            Qt::QueuedConnection);

    connect(myRtdHandler,SIGNAL(SendTemprature(double)),
            this,SLOT(GetTemprature(double)),
            Qt::QueuedConnection);

    connect(this,SIGNAL(ChangeSetpoint(double)),
            myRtdHandler,SLOT(GetSetPoint(double)),
            Qt::QueuedConnection);

    connect(this,SIGNAL(ServerConnection(bool)),
            myRtdHandler,SLOT(ServerConnection(bool)),
            Qt::QueuedConnection);
}

void MainWindow::ConnectToServer()
{
    socket->connectToHost(QHostAddress(_ServerIP),_ServerPort);
}

void MainWindow::StartApp()
{
    qDebug() << "main thread --> run rtd thread: ";
    RunRtdThread();
}

void MainWindow::RunRtdThread()
{
    myRtdHandler->moveToThread(myRtdThread);
    myRtdThread->start();
    emit this->ConfigRtdHandle(false,_cs,_sclk,_mosi,_miso,_delay,_Rnominal,_Rref,_ioFan,_ioElm,_setpoint,_kp,_ki,_kd);
    digitalWrite(29,HIGH);

}

void MainWindow::SocketStateChange(QAbstractSocket::SocketState socketSate)
{
    switch(socketSate){
        case 0:{
            //QAbstractSocket::UnconnectedState
            qDebug() << "main thread --> socketSate 0: " << socketSate;
            ui->label->setText("main thread --> socketSate 0: UnconnectedState");
            connect_toHost = false;
            emit this->ServerConnection(connect_toHost);
            ConnectToServer();
            break;
        }
        case 1:{
            //QAbstractSocket::HostLookupState
            qDebug() << "main thread --> socketSate 1: " << socketSate;
            break;
        }
        case 2:{
            //QAbstractSocket::ConnectingState
           qDebug() << "main thread --> socketSate 2: " << socketSate;
            break;
        }
        case 3:{
            //QAbstractSocket::ConnectedState
            qDebug() << "main thread --> socketSate 3: " << socketSate;
            ui->label->setText("main thread --> socketSate 0: ConnectedState");
            QString message = "myip:" + socket->localAddress().toString() + " ";
            qDebug() << "main thread --> message : " << message;
            socket->write(message.toLocal8Bit().data(),message.length());
            connect_toHost = true;
            emit this->ServerConnection(connect_toHost);
            break;
        }
        case 4:{
            qDebug() << "main thread --> socketSate 4: " << socketSate;
            break;
        }
        case 5:{
            qDebug() << "main thread --> socketSate 5: " << socketSate;
            break;
        }
        case 6:{
            qDebug() << "main thread --> socketSate 6: " << socketSate;
            break;
        }
    }
}

void MainWindow::ReadSocket()
{
    QString tmp="";
    while(socket->bytesAvailable())
            tmp = socket->readAll();
    qDebug() << "received message : " << tmp;

    if(tmp == "start") {
        emit this->StartStopRtdThread(true);
        QString message = "get-ON-command ";
        socket->write(message.toLocal8Bit().data(),message.length());
        return;
    }
    if(tmp == "stop") {
        emit this->StartStopRtdThread(false);
        QString message = "get-OFF-command ";
        socket->write(message.toLocal8Bit().data(),message.length());
        return;
    }
    if(tmp.contains("sp=")){
        QString temp = tmp.mid(3,tmp.count()-3);
        double sp = temp.toDouble();
        qDebug() << "receive set point : " << sp;
        if(sp > _max) sp = _max;
        if(sp < _min) sp = _min;
        emit this->ChangeSetpoint(sp);
        QString message = "get-SP-command ";
        socket->write(message.toLocal8Bit().data(),message.length());
        return;
    }
}

void MainWindow::GetTemprature(double pv)
{
    QString message = QString::number(pv) + " ";
    socket->write(message.toLocal8Bit().data(),message.length());
}

void MainWindow::on_StartStop_clicked()
{
    if(!start_stop){
        start_stop = true;
        ui->StartStop->setStyleSheet("background-color:green");
    }
    else{
        start_stop = false;
        ui->StartStop->setStyleSheet("background-color:red");
    }
    emit this->StartStopRtdThread(start_stop);
}
