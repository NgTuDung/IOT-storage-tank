#define BLYNK_TEMPLATE_ID "TMPL6YaqN5-06"
#define BLYNK_TEMPLATE_NAME "CAM BIEN DO MUC NUOC"
/* 2 dòng define đầu dùng để kết nối được với dữ liệu đã set up trước trên ứng dụng Blynk
  với các chức năng và các dữ liệu app khác nhau sẽ có những hệ thống khác nhau */
#include <SimpleKalmanFilter.h>

#define BLYNK_FIRMWARE_VERSION        "0.1.0"
#define BLYNK_PRINT Serial
#define APP_DEBUG
#define USE_NODE_MCU_BOARD
#define HIGH_ACCURACY
#include "BlynkEdgent.h"
#include <VL53L0X.h>
#include <Wire.h>

unsigned long timeUpdate=millis();
double  V, Vout;
double dx;
double Adjust_Volume;
int swt_out = 0, mode = 0;
float KF_dx=0, RE_KF_dx=0;
VL53L0X sensor;
WidgetLED LEDCONNECT(V0);
#define THE_TICH V1
#define DIEU_CHINH V2
#define THOAT_NUOC V3
#define DOI_CHEDO V4

/*Khai báo cho thư viện đi kèm với 3 thông số nhiễu
  - Nhiễu quá trình
  - Nhiễu đo lường
  - Sai số ước lượng ban đầu
*/
SimpleKalmanFilter KF(2,2,0.01);

/* CHỖ NÃY SETUP CHIỀU RỘNG, CHIỀU CAO CỦA BÌNH NƯỚC. ĐƠN VỊ MILIMET */
double chieu_rong  = 140; // chiều rộng bình nước
double chieu_cao = 200; // chiều cao bình nước


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  BlynkEdgent.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(200000); //Tăng timeout budget lên 200ms
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
}

BLYNK_WRITE(V2)
{
  Adjust_Volume = param.asFloat(); //Điều chỉnh gửi giá trị giới hạn về ESP8266
}

BLYNK_WRITE(V3)
{
  swt_out = param.asInt(); //Nhận tín hiệu đóng ngắt của bơm thoát nước
}

BLYNK_WRITE(V4)
{
  mode = param.asInt(); //Nhận tín hiệu đổi chế độ
}


void keepwaterlevel()
{
    dx = sensor.readRangeSingleMillimeters(); //ĐỌc giá trị khoảng cách đo được từ cảm biến xuống mực nước
    KF_dx = KF.updateEstimate(dx); //Lọc giá trị của khoảng cách đo được qua bộ lọc Kalman
    RE_KF_dx = 0.9238 * KF_dx - 0.606; //Tíếp tục lọc giá trị vừa cho ra qua phương trình hồi quy

    V = (chieu_cao - RE_KF_dx) * chieu_rong * chieu_rong /1000; 
    Blynk.virtualWrite(THE_TICH, V);

    if(swt_out==0 && V > Adjust_Volume)
      {
              digitalWrite(D7, LOW);
              digitalWrite(D6, LOW); 
              dx = sensor.readRangeSingleMillimeters();
              KF_dx = KF.updateEstimate(dx);
              RE_KF_dx = 0.9238 * KF_dx - 0.606;
              V = (chieu_cao - RE_KF_dx) * chieu_rong * chieu_rong /1000;
              Blynk.virtualWrite(THE_TICH, V);

      }
      
    if(swt_out==1 && V > Adjust_Volume)
      {
          digitalWrite(D7, HIGH);
          digitalWrite(D6, LOW);
          while(swt_out==1 && V > Adjust_Volume)
            {
              dx = sensor.readRangeSingleMillimeters();
              KF_dx = KF.updateEstimate(dx);
              RE_KF_dx = 0.9238 * KF_dx - 0.606;
              V = (chieu_cao - RE_KF_dx) * chieu_rong * chieu_rong /1000;
              Blynk.virtualWrite(THE_TICH, V);
              delay(200);
            }   
      }
    if(swt_out == 0 && V < Adjust_Volume)
    {
      digitalWrite(D7, LOW);
      digitalWrite(D6, HIGH);
      while(swt_out==0 && V < Adjust_Volume)
            {
              dx = sensor.readRangeSingleMillimeters();
              KF_dx = KF.updateEstimate(dx);
              RE_KF_dx = 0.9238 * KF_dx - 0.606;
              V = (chieu_cao - RE_KF_dx) * chieu_rong * chieu_rong /1000;
              Blynk.virtualWrite(THE_TICH, V);
              delay(200);
            } 

    }
    if(swt_out == 1 && V < Adjust_Volume)
    {
      digitalWrite(D7, HIGH);
      digitalWrite(D6, HIGH);
      while(swt_out==1 && V < Adjust_Volume)
            {
              dx = sensor.readRangeSingleMillimeters();
              KF_dx = KF.updateEstimate(dx);
              RE_KF_dx = 0.9238 * KF_dx - 0.606;
              V = (chieu_cao - RE_KF_dx) * chieu_rong * chieu_rong /1000;
              Blynk.virtualWrite(THE_TICH, V);
              delay(200);
            } 

    }
}

void requiredwater() 
{
    dx=sensor.readRangeSingleMillimeters();
    KF_dx = KF.updateEstimate(dx);
    RE_KF_dx = 0.9238 * KF_dx - 0.606;
    V = (chieu_cao - RE_KF_dx) * chieu_rong * chieu_rong /1000;
    Blynk.virtualWrite(THE_TICH, V);
    
    Vout = V - Adjust_Volume;
    if(swt_out == 1)
    {
      if(Adjust_Volume > V)
      {
        Blynk.virtualWrite(THOAT_NUOC, 0);
        swt_out = 0;
      }
      else
      {
        digitalWrite(D7, HIGH);
        while(V > Vout)
        {
          dx=  sensor.readRangeSingleMillimeters();
          KF_dx = KF.updateEstimate(dx);
          RE_KF_dx = 0.9238 * KF_dx - 0.606;
          V = (chieu_cao - RE_KF_dx) * chieu_rong * chieu_rong /1000;
          Blynk.virtualWrite(THE_TICH, V);
          delay(200);
        }
        digitalWrite(D7, LOW);
        Blynk.virtualWrite(THOAT_NUOC, 0);
        swt_out = 0;
      }
    } 
}


void loop() 
{
  BlynkEdgent.run();  

  
  if(millis()-timeUpdate > 500)
  {
    if(LEDCONNECT.getValue()) LEDCONNECT.off();
    else LEDCONNECT.on();    

    if(mode==1)
    {
      requiredwater();
    }
    else
    {
      keepwaterlevel();
    }
    
    timeUpdate=millis();
  }
}
/*
  millis() là thời gian chạy của một vòng truyền tín hiệu mà ESP8266 thực hiện của chương trình
  Cứ mỗi 500 ms thì sẽ thực hiện chức năng trong vòng điều kiện, đồng thời sẽ đẩy lên blink
*/
