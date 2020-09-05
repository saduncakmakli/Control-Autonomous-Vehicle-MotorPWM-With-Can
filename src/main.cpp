#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

//UNO İCİN
//MOSI 11 İLE KISA DEVRE!
//MISO 12 İLE KISA DEVRE!
//SCK 13 İLE KISA DEVRE!
//ISCP ÜZERİNDEN SPI KULLANIRKEN BU PINLERI KULLANMA!
//SMD OLMAYAN DIP SOKETLİ ARDUİNOLARDA 2.ICSP PINLARI 11,12,13DEN BAĞIMSIZ KULLANILABİLİR.
//UNO PWM PINLER 3,5,6,9,10
//UNO INTERRUPT PINLER 2,3

const int spiCSPin = 10; //12 MISO, 11 MOSI, 13 SCK
MCP_CAN CAN(spiCSPin);

const int motorPWM = 9;

//PID DEFINITION
void PID_Hesapla(float control_in, float control_now, unsigned int positive_PID_value_map_range, unsigned int negative_PID_value_map_range);
float error = 0;
float last_error = 0;
float P = 0;
float I = 0;
float D = 0;

const int multiple_max = 100;
const double P_max = multiple_max * 10.0;
const double I_max = multiple_max * 1.0;
const double D_max = multiple_max * 10.0;
const double PID_max = multiple_max * 10.0;
const double KP = 1.0;
const double KI = 0.0001;
const double KD = 0.01;
float PID = 0.0;

void setup() 
{
  //SERIAL BEGIN
  Serial.begin(9600);

  //MOTOR SETUP
  pinMode(motorPWM,OUTPUT);

  //CAN CONNETION BEGIN
  while (CAN_OK != CAN.begin(CAN_500KBPS))
  {
    Serial.println("CAN BUS Init Failed");
    delay(100);
  }
  Serial.println("CAN BUS  Init OK!");
}

void loop() 
{
  
}

void PID_Hesapla(float control_in, float control_now, unsigned int positive_PID_value_map_range, unsigned int negative_PID_value_map_range)
{
  last_error = error;
  error = control_in - control_now; //PID ile kontrol edilecek değişken control now

  P = KP * error; //Oransal Kontrol
  if (P > P_max)
    P = P_max;
  else if (P < -P_max)
    P = -P_max;

  I += KI * error; //Integralsal Kontrol
  if (I > I_max)
    I = I_max;
  else if (I < -I_max)
    I = -I_max;

  D = KD * (error - last_error); //Diferansiyel Kontrol
  if (D > D_max)
    D = D_max;
  else if (D < -D_max)
    D = -D_max;

  PID = P + I + D;
  if (PID > PID_max)
    PID = PID_max;
  else if (PID < -PID_max)
    PID = -PID_max;

  if (PID > 0)
    PID = map(PID, 0, PID_max, 0, positive_PID_value_map_range);
  else if (PID < 0)
    PID = map(PID, -PID_max, 0, -negative_PID_value_map_range, 0);
}