#include "rtdhandler.h"
#include <QCoreApplication>
#include <math.h>
#include <QTime>
#include <QDebug>

RtdHandler::RtdHandler(QObject *parent) : QObject(parent)
{

}

void RtdHandler::GetConfigFromMain(bool Wire, int cs, int sclk, int mosi, int miso, int delay, double Rnominal, double Rref, int fan, int elm, double set_point, double kp, double ki, double kd)
{
    wiringPiSetupPhys();
    qDebug() << "rtd Thread: --> start thread";

    On_Off_BK = false;
    On_Off = false;

    startpid = false;

    Set_Point = -1;
    Set_Point_BK = -1;

    count = 0;

    _delay = delay;
    _Rnominal = Rnominal;
    _Rref = Rref;

    first = false;

    ReadTemprature = 0;
    PID_error = 0;
    prevous_error = 0;
    elapsedTime = 0;
    Time = 0;
    timePrev = 0;
    PID_value = 0;
    allTime = 0;
    allPID_error = 0;

    PID_p = 0;
    PID_i = 0;
    PID_d = 0;

    _kp = kp;
    _ki = ki;
    _kd = kd;


    _cs = cs;
    _sclk = sclk;
    _mosi = mosi; //sdi
    _miso = miso; //sdo
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    pinMode(_sclk, OUTPUT);
    digitalWrite(_sclk, LOW);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);

    _ioFan = fan;
    _ioElm = elm;
//    qDebug() << "softPwmWrite heater = " << softPwmCreate(_ioElm,0,100);
//    qDebug() << "softPwmWrite cooler = " << softPwmCreate(_ioFan,0,100);

    Set_Point = set_point;

    timer = new QTime;

    qDebug() << "rtd Thread: --> _cs pin : " << _cs;
    qDebug() << "rtd Thread: --> _sclk pin : " << _sclk;
    qDebug() << "rtd Thread: --> _mosi pin : " << _mosi;
    qDebug() << "rtd Thread: --> _miso pin : " << _miso;

    qDebug() << "rtd Thread: --> _ioFan pin : " << _ioFan;
    qDebug() << "rtd Thread: --> _ioElm pin : " << _ioElm;

    qDebug() << "rtd Thread: --> _Rnominal : " << _Rnominal;
    qDebug() << "rtd Thread: --> _Rref : " << _Rref;

    qDebug() << "rtd Thread: --> _delay : " << _delay;

    qDebug() << "rtd thread --> Kp: " << _kp;
    qDebug() << "rtd thread --> Ki: " << _ki;
    qDebug() << "rtd thread --> Kd: " << _kd;



    if(Wire){
        qDebug() << "rtd Thread: --> 3 wires";
        if(begin(MAX31865_3WIRE))
            MainThread();
        else
            qDebug() << "rtd Thread: --> can not began rtd";
    }
    else{
        qDebug() << "rtd Thread: --> 4 or 2 wires";
        if(begin(MAX31865_4WIRE))
            MainThread();
        else
            qDebug() << "rtd Thread: --> can not began rtd";
    }
}

void RtdHandler::MainThread()
{
//    digitalWrite(12,HIGH);
    while(1){
        QCoreApplication::processEvents(QEventLoop::AllEvents);
        if(On_Off){
            fullproc();
        }
        else{
            softPwmWrite(16,0);
            softPwmWrite(12,0);
        }
    }
}

double RtdHandler::readTemp()
{
    uint16_t rtd = readRTD();
    double ratio = rtd;
    ratio /= 32768;

    uint8_t fault = readFault();
    if (fault) {
        qDebug() << "rtd Thread: --> fault";
      if (fault & MAX31865_FAULT_HIGHTHRESH) {
         qDebug() << "Rtd Thread: --> fault: RTD High Threshold";
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) {
         qDebug() << "rtd Thread: --> fault: RTD Low Threshold";
      }
      if (fault & MAX31865_FAULT_REFINLOW) {
         qDebug() << "rtd Thread: --> fault: REFIN- > 0.85 x Bias";
      }
      if (fault & MAX31865_FAULT_REFINHIGH) {
         qDebug() << "rtd Thread: --> fault: REFIN- < 0.85 x Bias - FORCE- open";
      }
      if (fault & MAX31865_FAULT_RTDINLOW) {
         qDebug() << "rtd Thread: --> fault: RTDIN- < 0.85 x Bias - FORCE- open";
      }
      if (fault & MAX31865_FAULT_OVUV) {
         qDebug() << "rtd Thread: --> fault: Under/Over voltage";
      }
      clearFault();
      return -1000;
    }
    else{
        clearFault();
        return temperature(_Rnominal, _Rref);
    }
}

int RtdHandler::calculatePID(double PV)
{
    PID_error = Set_Point - PV;
    timePrev = Time;
    Time = timer->elapsed();
    elapsedTime = (Time - timePrev) / 1000;
    allTime = allTime + elapsedTime;
    int duty = 0;

    PID_p = _kp * PID_error;

    allPID_error = allPID_error + PID_error;


    PID_i = _ki * allPID_error * allTime;

    PID_d = _kd * ( (PID_error - prevous_error)/elapsedTime );
    prevous_error  = PID_error;

    PID_value = PID_p + PID_i + PID_d;


    if (PV > (Set_Point + 0.5) || PV < Set_Point - 0.5){
        allTime = 0;
        allPID_error=0;
    }


    if(PID_value > 100) PID_value = 100;
    if(PID_value < -100) PID_value = -100;

    duty = (int)PID_value;


    qDebug() << "*********************************************";
    qDebug() << "PID_error = " <<PID_error;
    qDebug() << "elapsedTime = " << elapsedTime;
    qDebug() << "allTime = " << allTime;
    qDebug() << "allPID_error = " << allPID_error;
    qDebug() << "PID_p " << PID_p;
    qDebug() << "PID_i " << PID_i;
    qDebug() << "PID_d " << PID_d;
    qDebug() << "PID_value " << PID_value;
    qDebug() << "duty " << duty;
    qDebug() << "*********************************************";

    return duty;
}

void RtdHandler::ForcePWM(int duty)
{
    if(duty>0){
        softPwmWrite(12,abs(duty));
        softPwmWrite(16,0);
    }
    else{
        softPwmWrite(16,abs(duty));
        softPwmWrite(12,0);
    }
}

void RtdHandler::fullproc()
{
    if(On_Off){
        if(Server_Connection){
            if(startpid){
                if(!first){
                    qDebug() << "start";
                    first = true;
                    now = 0;
                    last = 0;
                    timer->start();
                    qDebug() << "start";
                }
                double temp = readTemp();
                qDebug() << "pv = " << temp;

                if(temp >= Set_Point){ emit this->SendTemprature(((double)((int)(temp*100)))/100); }
                else if(temp < Set_Point){emit this->SendTemprature(((double)(floor(temp*100+0.5)))/100);}
                int duty = calculatePID(temp);
                ForcePWM(duty);

                delayMicroseconds(250000);
            }
            else{
                qDebug() << "not start PID process ";
                double temp = readTemp();
                qDebug() << "sp = " << Set_Point;
                qDebug() << "pv = " << temp;
                if(temp >= Set_Point){ emit this->SendTemprature(((double)((int)(temp*100)))/100); }
                else if(temp < Set_Point){emit this->SendTemprature(((double)(floor(temp*100+0.5)))/100);}
                if (temp > (Set_Point + 0.5) || temp < Set_Point - 0.5){
                    startpid = false;
                    first = false;
                    if(temp > (Set_Point + 0.3)){
                        ForcePWM(-100);
                    }
                    else if(temp < Set_Point - 0.3){
                        ForcePWM(100);
                    }
                }
                else{
                    qDebug() << "start PID process ";
                    startpid = true;
                }
                delayMicroseconds(250000);
            }
        }
        else{
            first = false;
            startpid = false;
            On_Off = false;
            ForcePWM(0);
        }
    }
    else{
        first = false;
        startpid = false;
        ForcePWM(0);
    }

//    if(On_Off && Server_Connection){
//        if(!first){
//            qDebug() << "start";
//            first = true;
//            now = 0;
//            last = 0;
//            timer->start();
//            qDebug() << "start";
//        }

//        double temp = readTemp();
//        qDebug() << "pv = " << temp;
//        if(temp >= Set_Point){ emit this->SendTemprature(((double)((int)(temp*100)))/100); }
//        else if(temp < Set_Point){emit this->SendTemprature(((double)(floor(temp*100+0.5)))/100);}
//        int duty = calculatePID(temp);
//        ForcePWM(duty);

//        delayMicroseconds(250000);
//    }
//    else{
//        first = false;
//        ForcePWM(0);
//    }
}

void RtdHandler::On_Off_Slot(bool on_off)
{
    On_Off_BK = On_Off;
    On_Off = on_off;
    first = false;
}

void RtdHandler::GetSetPoint(double setpoint)
{
    Set_Point = setpoint;
    first = false;
    startpid = false;
    qDebug() << "rtd Thread: --> Set_Point changed : " << Set_Point;
}

void RtdHandler::ServerConnection(bool connection)
{
    Server_Connection = connection;
    On_Off_Slot(false);
}

bool RtdHandler::begin(max31865_numwires_t wires) {

    for (uint8_t i = 0; i < 16; i++) {
        // readRegister8(i);
    }

    setWires(wires);
    enableBias(false);
    autoConvert(false);
    clearFault();

    return true;
}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t RtdHandler::readFault(void) {
  return readRegister8(MAX31856_FAULTSTAT_REG);
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void RtdHandler::clearFault(void) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void RtdHandler::enableBias(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void RtdHandler::autoConvert(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO; // disable autoconvert
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void RtdHandler::setWires(max31865_numwires_t wires) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31856_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
double RtdHandler::temperature(double RTDnominal, double refResistor) {
  double Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD();
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  double rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t RtdHandler::readRTD(void) {
  clearFault();
  enableBias(true);
  delay(10);
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;
  writeRegister8(MAX31856_CONFIG_REG, t);
  delay(65);

  uint16_t rtd = readRegister16(MAX31856_RTDMSB_REG);

  // remove fault
  rtd >>= 1;

  return rtd;
}

/**********************************************/

uint8_t RtdHandler::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t RtdHandler::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

void RtdHandler::readRegisterN(uint8_t addr, uint8_t buffer[],
                                      uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set

  digitalWrite(_cs, LOW);

  spixfer(addr);

  // Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
  while (n--) {
    buffer[0] = spixfer(0xFF);
    // Serial.print(" 0x"); Serial.print(buffer[0], HEX);
    buffer++;
  }
  // Serial.println();

  digitalWrite(_cs, HIGH);
}

void RtdHandler::writeRegister8(uint8_t addr, uint8_t data) {
    digitalWrite(_sclk, LOW);

  digitalWrite(_cs, LOW);

  spixfer(addr | 0x80); // make sure top bit is set
  spixfer(data);

  // Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x");
  // Serial.println(data, HEX);

  digitalWrite(_cs, HIGH);
}

uint8_t RtdHandler::spixfer(uint8_t x) {


  // software spi
  // Serial.println("Software SPI");
  uint8_t reply = 0;

  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    digitalWrite(_sclk, HIGH);
    digitalWrite(_mosi, x & (1 << i));
    digitalWrite(_sclk, LOW);
    if (digitalRead(_miso))
      reply |= 1;
  }

  return reply;
}


