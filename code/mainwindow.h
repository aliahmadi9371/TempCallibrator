#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QSettings>

#include "wiringPi.h"
#include "rtdhandler.h"

#include <QTcpSocket>
#include <QHostAddress>

// Constants for the module config
#define MAX31856_CONFIG_REG           0x00
#define MAX31856_CONFIG_BIAS          0x80
#define MAX31856_CONFIG_MODEAUTO      0x40
#define MAX31856_CONFIG_MODEOFF       0x00
#define MAX31856_CONFIG_1SHOT         0x20
#define MAX31856_CONFIG_3WIRE         0x10
#define MAX31856_CONFIG_24WIRE        0x00
#define MAX31856_CONFIG_FAULTSTAT     0x02
#define MAX31856_CONFIG_FILT50HZ      0x01
#define MAX31856_CONFIG_FILT60HZ      0x00
#define MAX31856_RTDMSB_REG           0x01
#define MAX31856_RTDLSB_REG           0x02
#define MAX31856_HFAULTMSB_REG        0x03
#define MAX31856_HFAULTLSB_REG        0x04
#define MAX31856_LFAULTMSB_REG        0x05
#define MAX31856_LFAULTLSB_REG        0x06
#define MAX31856_FAULTSTAT_REG        0x07
#define MAX31865_FAULT_HIGHTHRESH     0x80
#define MAX31865_FAULT_LOWTHRESH      0x40
#define MAX31865_FAULT_REFINLOW       0x20
#define MAX31865_FAULT_REFINHIGH      0x10
#define MAX31865_FAULT_RTDINLOW       0x08
#define MAX31865_FAULT_OVUV           0x04
#define RTD_A 3.9083e-3
#define RTD_B (-5.775e-7)




namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void ConfigApp();
    void InitValues();
    void ConfigIo();
    void CreateObjects();
    void Connections();
    void ConnectToServer();
    void RunRtdThread();

    QString _ServerIP;
    int _ServerPort;

    int _ioFan;
    int _ioElm;

    int _cs;
    int _sclk;
    int _miso;
    int _mosi;

    int _delay;

    double _kp;
    double _ki;
    double _kd;

    double _setpoint;
    double _max;
    double _min;

    double _Rnominal;
    double _Rref;

    bool fan;
    bool elm;

    bool start_stop;

    bool connect_toHost;

private:
    Ui::MainWindow *ui;

    RtdHandler* myRtdHandler;
    QThread* myRtdThread;

    QTcpSocket *socket;
    int socketState;

signals:
    void ConfigRtdHandle(bool Wire,int cs,int sclk,int mosi, int miso, int delay, double Rnominal, double Rref, int fan, int elm, double set_piont, double kp, double ki, double kd);
    void StartStopRtdThread(bool on_off);
    void StartStart();
    void ChangeSetpoint(double setpoint);
    void ServerConnection(bool connection);


public slots:
    void StartApp();
    void SocketStateChange(QAbstractSocket::SocketState socketSate);
    void ReadSocket();
    void GetTemprature(double pv);


private slots:
    void on_StartStop_clicked();
};

#endif // MAINWINDOW_H
