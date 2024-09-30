#ifndef RTDHANDLER_H
#define RTDHANDLER_H

#include <QObject>
#include <QDebug>
#include <QtMath>
#include "wiringPi.h"
#include "softPwm.h"



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

typedef enum max31865_numwires {
    MAX31865_2WIRE = 0,
    MAX31865_3WIRE = 1,
    MAX31865_4WIRE = 0
}max31865_numwires_t;

class RtdHandler : public QObject
{
    Q_OBJECT
private:
    bool On_Off;
    bool On_Off_BK;
    double Set_Point;
    double Set_Point_BK;
public:
    explicit RtdHandler(QObject *parent = nullptr);

    bool StartStop;
    bool StartedStoped;
    bool startpid;

    int _cs;
    int _sclk;
    int _miso;
    int _mosi;

    QTime *timer;
    int dt; //ms
    int now;
    int last;
    int pastlast;
    int counter;

    int _ioFan;
    int _ioElm;

    int _delay;

    double _Rnominal;
    double _Rref;

    void MainThread();
    void fullproc();

    bool begin(max31865_numwires_t wires);
    uint8_t readFault();
    void clearFault();
    uint16_t readRTD();
    void setWires(max31865_numwires_t wires);
    void autoConvert(bool b);
    void enableBias(bool b);
    double temperature(double RTDnominal, double refResistor);
    uint8_t readRegister8(uint8_t addr);
    void writeRegister8(uint8_t addr, uint8_t reg);

    void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
    uint16_t readRegister16(uint8_t addr);
    uint8_t spixfer(uint8_t x);

    double readTemp();
    int calculatePID(double PV);
    void ForcePWM(int duty);

    int count;
    bool first;

    double ReadTemprature;
    double PID_error;
    double prevous_error;
    double elapsedTime, Time, timePrev;
    double allTime;
    double PID_value;
    double allPID_error;

    double PID_p;
    double PID_i;
    double PID_d;

    double _kp;
    double _ki;
    double _kd;

    bool Server_Connection;


signals:
    void SendTemprature(double pv);


public slots:
    void GetConfigFromMain(bool Wire, int cs, int sclk, int mosi, int miso, int delay, double Rnominal, double Rref, int fan, int elm, double set_point, double kp, double ki, double kd);
    void On_Off_Slot(bool on_off);
    void GetSetPoint(double setpoint);
    void ServerConnection(bool connection);
};

#endif // RTDHANDLER_H
