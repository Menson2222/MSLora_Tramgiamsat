//                       _oo0oo_
//                      o8888888o
//                      88" . "88
//                      (| -_- |)
//                      0\  =  /0
//                    ___/`---'\___
//                  .' \\|     |// '.
//                 / \\|||  :  |||// \
//                / _||||| -:- |||||- \
//               |   | \\\  -  /// |   |
//               | \_|  ''\---/''  |_/ |
//               \  .-\__  '-'  ___/-. /
//             ___'. .'  /--.--\  `. .'___
//          ."" '<  `.___\_<|>_/___.' >' "".
//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//         \  \ `_.   \_ __\ /__ _/   .-` /  /
//     =====`-.____`.___ \_____/___.-`___.-'=====
//                       `=---='
//
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//            Phật phù hộ, không bao giờ BUG
//                Nam mô a di đà phật
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* MSLora - He thong giam sat moi truong nuoi tom 
      ung dung thi giac may tinh, IoT va mang
             truyen thong Lora*/
// DESIGN BY MENSON - Huynh Manh Sang 
// PHONE: 0869053063
// Facebook: https://www.facebook.com/profile.php?id=100076267646838&mibextid=ZbWKwL (Huynh Sang)
#define ERA_DEBUG
// ERA AUTOKEN
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"
#define ERA_AUTH_TOKEN "23bcf085-6e6b-4c34-b907-4b6d016a958a" 

// ADD LIBRARY
#include <WiFi.h>
#include <HardwareSerial.h> 
#include <Arduino.h>
#include <ERa.hpp>
#include <ERa/ERaTimer.hpp>

//UART ESP32 - NEXTION HMI
#define TXHMI 17
#define RXHMI 16

//UART ESP32 - LORA
#define TXLORA 4
#define RXLORA 2

//ERORR
#define LEDRED 13
#define LEDGREEN 12

//RELAY PIN
#define WATERPUMP 14
#define OXYGEN 27
#define FEED 26
#define WATERFAN 25

// ĐỊNH NGHĨA BIẾN
char ssid[32];
char password[32];
bool check = 0;
bool vscheck=0;
bool erorr=0;
uint8_t sec = 0;
int TIMEPUMPON = 0;
int TIMEPUMPOFF = 0;
int TIMEFANON = 0;
int TIMEFANOFF = 0;
int TIMEOXYGENON = 0;
int TIMEOXYGENOFF = 0;
int TIMEFEEDON = 0;
int TIMEFEEDOFF = 0;
int TIMEVSON = 0;
int TIMENOW = 0;
int TDSMAX = 50000; 
int TDSMIN = 0;
int PHMAX = 50000;  
int PHMIN = 0;  
int DOMAX = 50000;  
int DOMIN = 0;  
int TOMAX = 50000;  
int TOMIN = 0;  
int ECMAX = 50000;  
int ECMIN = 0;
float floatEC = 0;
float floatTDS = 0;
float floatDO = 0;
float floatTO = 0;
float floatPH = 0;
String CHECKBUTTON = "";
String Data_From_Lora = "";
String EC = "000.00";
String TDS = "000.00";
String DO = "000.00";
String TO = "000.00";
String PH = "000.00";
String ERORR = "";

// SETUP CỔNG GIAO TIẾP UART 1,2
HardwareSerial nextion(2); 
HardwareSerial lora(1); 

ERaTimer timer;
void TaskEra(void * pvParameters);

ERA_WRITE(V0) {
    uint8_t value = param.getInt();
    digitalWrite(WATERPUMP, value ? HIGH : LOW);
    ERa.virtualWrite(V0, digitalRead(WATERPUMP));
}
ERA_WRITE(V1) {
    uint8_t value = param.getInt();
    digitalWrite(OXYGEN, value ? HIGH : LOW);
    ERa.virtualWrite(V1, digitalRead(OXYGEN));
}
ERA_WRITE(V2) {
    uint8_t value = param.getInt();
    digitalWrite(WATERFAN, value ? HIGH : LOW);
    ERa.virtualWrite(V2, digitalRead(WATERFAN));
}
ERA_WRITE(V3) {
    uint8_t value = param.getInt();
    digitalWrite(FEED, value ? HIGH : LOW);
    ERa.virtualWrite(V3, digitalRead(FEED));
}
ERA_WRITE(V4) {
    uint8_t value = param.getInt();
    if(value==1){
    digitalWrite(LEDGREEN,HIGH);
    lora.print("ONNNN");
    vscheck=1;
    }
    ERa.virtualWrite(V4, vscheck);
}

void timerEvent() {
    ERA_LOG("Timer", "Uptime: %d", ERaMillis() / 1000L);
    ERa.virtualWrite(V0, digitalRead(WATERPUMP));
    ERa.virtualWrite(V1, digitalRead(OXYGEN));
    ERa.virtualWrite(V2, digitalRead(WATERFAN));
    ERa.virtualWrite(V3, digitalRead(FEED)); 
    ERa.virtualWrite(V4, vscheck);    
    ERa.virtualWrite(V5, floatEC);
    ERa.virtualWrite(V6, floatTDS);
    ERa.virtualWrite(V7, floatDO);
    ERa.virtualWrite(V8, floatPH);
    ERa.virtualWrite(V9, floatTO);
    ERa.virtualWrite(V10, erorr);                                     
}

void setup() {
  Serial.begin(9600);
  pinMode(LEDRED,OUTPUT);
  pinMode(LEDGREEN,OUTPUT);
  pinMode(WATERPUMP,OUTPUT);
  pinMode(OXYGEN,OUTPUT);
  pinMode(FEED,OUTPUT);
  pinMode(WATERFAN,OUTPUT);
  nextion.begin(9600, SERIAL_8N1, 16, 17); 
  lora.begin(9600, SERIAL_8N1, 2, 4); 
  WiFi.mode(WIFI_STA);
  while (check==0)
  {
    connectToWiFi();
  }
  configTime(25200,0, "pool.ntp.org", "time.nist.gov");
  while (!time(nullptr)) {
  delay(1000);
  }
  xTaskCreatePinnedToCore (TaskEra, "Task Era NTP", 10000, NULL, 1, NULL, 0);
}

                                                                //HÀM TIMER CHECK
void timeCheck() {
time_t now = time(nullptr);
  struct tm* p_tm = localtime(&now);
  nextion.print("day.val=");
  nextion.print(p_tm->tm_mday);
  giveHMI(); 
  nextion.print("mon.val=");
  nextion.print(p_tm->tm_mon + 1);
  giveHMI();
  nextion.print("year.val=");
  nextion.print(p_tm->tm_year + 1900);
  giveHMI();
  nextion.print("hour.val=");
  nextion.print(p_tm->tm_hour);
  giveHMI();
  nextion.print("min.val=");
  nextion.print(p_tm->tm_min);
  giveHMI();
  TIMENOW = (p_tm->tm_hour)*100 + p_tm->tm_min;
  sec = p_tm->tm_sec;
}

                                                          // HÀM TÁCH CHUỖI THEO THỨ TỰ 
String splitString(String str, String delim, uint16_t pos) {
  String tmp = str;
  for (int i = 0; i < pos; i++) {
    tmp = tmp.substring(tmp.indexOf(delim) + 1);
    if (tmp.indexOf(delim) == -1
        && i != pos - 1)
      return "";
  }
  return tmp.substring(0, tmp.indexOf(delim));
}

                                              // HÀM BẮT BUỘC PHẢI CÓ KHI GỬI MỘT CHUỖI ĐẾN HMI
void giveHMI()
{
  nextion.write(0xff);
  nextion.write(0xff);
  nextion.write(0xff);
}

                                                              // HÀM KẾT NỐI WIFI
void connectToWiFi() {
    while (nextion.available() > 0) {
    String readinput = nextion.readStringUntil('\n'); 
    String input = readinput.substring(4, readinput.length() - 4);
    int commaIndex = input.indexOf(';'); 
    if (commaIndex > 0) {
      String ssidStr = input.substring(0, commaIndex); 
      String passStr = input.substring(commaIndex + 1); 
      ssidStr.trim();
      passStr.trim();
      ssidStr.toCharArray(ssid, sizeof(ssid)); 
      passStr.toCharArray(password, sizeof(password));    
  WiFi.begin(ssid, password); 
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
  nextion.print("page 37");
  giveHMI();
  delay(1000);
  }
  if (WiFi.status() == WL_CONNECTED) {
  nextion.print("page 38");
  giveHMI();
   check=1;
  } else {
  nextion.print("page 39");
  giveHMI();
   check=0;
 }
}  else{
  }
 }
}
                                                       // HÀM LƯU THỜI GIAN ON/OFF
void ONOFF(String CHECK="",uint8_t val = 0) {
  int numString = CHECK.indexOf(';');
  String firstPart = CHECK.substring(numString-4, numString);
  String secondPart = CHECK.substring(numString + 1, numString + 5);
  int firstNumber = firstPart.toInt();
  int secondNumber = secondPart.toInt();
  if(val==1){ 
  TIMEPUMPON=firstNumber;
  TIMEPUMPOFF=secondNumber;
  }
  if(val==2){ 
  TIMEFANON=firstNumber;
  TIMEFANOFF=secondNumber;
  }
  if(val==3){ 
  TIMEOXYGENON=firstNumber;
  TIMEOXYGENOFF=secondNumber;
  }
  if(val==4){ 
  TIMEFEEDON=firstNumber;
  TIMEFEEDOFF=secondNumber;
  }
  if(val==5){ 
  TIMEVSON=firstNumber;
  }
}

                                                      // HÀM LƯU NGƯỠNG MIN, MAX 
void SETNGUONG(String CHECK="",uint8_t val = 0) {
  int numString = CHECK.indexOf(';');
  String firstPart = CHECK.substring(numString-5, numString);
  String secondPart = CHECK.substring(numString + 1, numString + 6);
  int firstFloat = firstPart.toInt();
  int secondFloat = secondPart.toInt();
 if (val == 1) { 
    TDSMAX = firstFloat;
    TDSMIN = secondFloat;
  }
  if (val == 2) {
    PHMAX = firstFloat;
    PHMIN = secondFloat;   
  }
  if (val == 3) {
    DOMAX = firstFloat;
    DOMIN = secondFloat;   
  }
  if (val == 4) {
    TOMAX = firstFloat;
    TOMIN = secondFloat;   
  }
  if (val == 5) {
    ECMAX = firstFloat;
    ECMIN = secondFloat;   
  }
}

                                                      // HÀM CHECK DỮ LIỆU TỪ HMI
void checkbutton() {
  if (nextion.available()) {
    CHECKBUTTON = nextion.readString();
 if (CHECKBUTTON.indexOf("WATERPUMP") >= 0) {
      digitalWrite(WATERPUMP, !digitalRead(WATERPUMP));
 } 
 if (CHECKBUTTON.indexOf("WATERFAN") >= 0) {
      digitalWrite(WATERFAN, !digitalRead(WATERFAN));
 }
 if (CHECKBUTTON.indexOf("OXYGEN") >= 0) {
      digitalWrite(OXYGEN, !digitalRead(OXYGEN));
 } 
 if (CHECKBUTTON.indexOf("FEED") >= 0) {
      digitalWrite(FEED, !digitalRead(FEED));
 }
if (CHECKBUTTON.indexOf("SENSORVS") >= 0) {
    digitalWrite(LEDGREEN,HIGH);
    lora.print("ONNNN");
   vscheck=1;
 }
 if (CHECKBUTTON.indexOf("TIMEPUMP") >= 0) {
 ONOFF(CHECKBUTTON,1); 
 } 
 if (CHECKBUTTON.indexOf("TIMEFAN") >= 0) {
 ONOFF(CHECKBUTTON,2);  
 } 
 if (CHECKBUTTON.indexOf("TIMEOXY") >= 0) {
 ONOFF(CHECKBUTTON,3); 
 } 
 if (CHECKBUTTON.indexOf("TIMEFED") >= 0) {
 ONOFF(CHECKBUTTON,4);  
    }  
 if (CHECKBUTTON.indexOf("TIMEVS") >= 0) {
 ONOFF(CHECKBUTTON,5);  
    }      
 if (CHECKBUTTON.indexOf("SETTDS") >= 0) {
 SETNGUONG(CHECKBUTTON,1);  
    }     
 if (CHECKBUTTON.indexOf("SETPH") >= 0) {
 SETNGUONG(CHECKBUTTON,2);    
    }  
 if (CHECKBUTTON.indexOf("SETDO") >= 0) {
 SETNGUONG(CHECKBUTTON,3);    
    }  
 if (CHECKBUTTON.indexOf("SETTO") >= 0) {
 SETNGUONG(CHECKBUTTON,4);    
    }  
 if (CHECKBUTTON.indexOf("SETEC") >= 0) {
 SETNGUONG(CHECKBUTTON,5);    
    }                  
  } 
}

                                                      // HÀM CHECK DỮ LIỆU TỪ LORA
void updateDataToScreen() {
  if(lora.available()){
  Data_From_Lora = "";
  Data_From_Lora = lora.readString();
  EC = splitString(Data_From_Lora, ";", 0);
  floatEC = EC.toFloat();  
  TDS = splitString(Data_From_Lora, ";", 1);
  floatTDS = TDS.toFloat();  
  DO = splitString(Data_From_Lora, ";", 2);
  floatDO = DO.toFloat();  
  TO = splitString(Data_From_Lora, ";", 3);
  floatTO = TO.toFloat();  
  PH = splitString(Data_From_Lora, ";", 4);
  floatPH = PH.toFloat();  
  ERORR = splitString(Data_From_Lora, ";", 5);
  }
  if(Data_From_Lora.indexOf("OFFFF") >= 0)
  {
    vscheck=0;
    digitalWrite(LEDGREEN,LOW);
  }
  nextion.print("giamsat.ec.txt=" + String('"') + EC + String('"'));
  giveHMI();
  nextion.print("giamsat.tds.txt=" + String('"') + TDS + String('"'));
  giveHMI();
  nextion.print("giamsat.do.txt=" + String('"') + DO + String('"'));
  giveHMI();
  nextion.print("giamsat.to.txt=" + String('"') + TO + String('"'));
  giveHMI();
  nextion.print("giamsat.ph.txt=" + String('"') + PH + String('"'));
  giveHMI();
}

                                                      // HÀM UPDATE TRẠNG THÁI NÚT NHẤN
void updatebutton() {
  if(digitalRead(WATERPUMP)==0) {
  nextion.print("bt0.val=0");
  giveHMI();
  } else {
  nextion.print("bt0.val=1");
  giveHMI();
  }
  if(digitalRead(OXYGEN)==0) {
  nextion.print("bt1.val=0");
  giveHMI();
  } else {
    nextion.print("bt1.val=1");
    giveHMI();
    }
  if(digitalRead(FEED)==0) {
  nextion.print("bt2.val=0");
  giveHMI();
  } else {
    nextion.print("bt2.val=1");
    giveHMI();
    }
  if(digitalRead(WATERFAN)==0){
  nextion.print("bt3.val=0");
  giveHMI();
  } else {
    nextion.print("bt3.val=1");
    giveHMI();
  }
  if(vscheck==0){
  nextion.print("bt4.val=0");
  giveHMI();
  } else {
    nextion.print("bt4.val=1");
    giveHMI();
  }
}

                                                          // HÀM TIME ON/OFF
void checkOnoff()
{ 
  if(TIMEPUMPON==TIMENOW && sec==1) {
  digitalWrite(WATERPUMP,HIGH);
  }
  if(TIMEPUMPOFF==TIMENOW && sec==1){
  digitalWrite(WATERPUMP,LOW);
  }
  if(TIMEFANON==TIMENOW && sec==1) {
  digitalWrite(WATERFAN,HIGH);
  }
  if(TIMEFANOFF==TIMENOW && sec==1){
  digitalWrite(WATERFAN,LOW);
  }
  if(TIMEOXYGENON==TIMENOW && sec==1) {
  digitalWrite(OXYGEN,HIGH);
  }
  if(TIMEOXYGENOFF==TIMENOW && sec==1){
  digitalWrite(OXYGEN,LOW);
  }
  if(TIMEFEEDON==TIMENOW && sec==1) {
  digitalWrite(FEED,HIGH);
  }
  if(TIMEFEEDOFF==TIMENOW && sec==1) {
  digitalWrite(FEED,LOW);
  }    
  if(TIMEVSON==TIMENOW && sec==1) {
  lora.print("ONNNN");
  digitalWrite(LEDGREEN,HIGH);
  vscheck=1;
  }    
}

                                                     // HÀM CHECK NGƯỠNG MIN MAX
void CheckMinMax() {
    // Nhân các giá trị float với hệ số 100 để so sánh với các giá trị int
    if (floatEC * 100 > ECMAX || floatEC * 100 < ECMIN) { // EC
        nextion.print("ec.val=1");
        giveHMI();
    } else {
        digitalWrite(LEDRED, LOW);
        nextion.print("ec.val=0");
        giveHMI();
    }

    if (floatTDS * 100 > TDSMAX || floatTDS * 100 < TDSMIN) { // TDS
        nextion.print("tds.val=1");
        giveHMI();
    } else {
        digitalWrite(LEDRED, LOW);
        nextion.print("tds.val=0");
        giveHMI();
    }

    if (floatDO * 100 > DOMAX || floatDO * 100 < DOMIN) { // DO
        nextion.print("do.val=1");
        giveHMI();
    } else {
        digitalWrite(LEDRED, LOW);
        nextion.print("do.val=0");
        giveHMI();
    }

    if (floatTO * 100 > TOMAX || floatTO * 100 < TOMIN) { // TO
        nextion.print("to.val=1");
        giveHMI();
    } else {
        digitalWrite(LEDRED, LOW);
        nextion.print("to.val=0");
        giveHMI();
    }

    if (floatPH * 100 > PHMAX || floatPH * 100 < PHMIN) {  // PH
        nextion.print("ph.val=1");
        giveHMI();
    } else {
        digitalWrite(LEDRED, LOW);
        nextion.print("ph.val=0");
        giveHMI();
    }

    if (ERORR.indexOf("CCCC") >= 0) {  // ERORR
        erorr = 0;
        nextion.print("erorr.val=1");
        giveHMI();
    } else {
        erorr = 1;
        digitalWrite(LEDRED, LOW);
        nextion.print("erorr.val=0");
        giveHMI();
    }
    if (ERORR.indexOf("CCCC") >= 0 || floatPH * 100 > PHMAX || floatPH * 100 < PHMIN || floatTO * 100 > TOMAX || floatTO * 100 < TOMIN ||  floatDO * 100 > DOMAX || floatDO * 100 < DOMIN ||  floatTDS * 100 > TDSMAX || floatTDS * 100 < TDSMIN ||  floatEC * 100 > ECMAX || floatEC * 100 < ECMIN) {
        digitalWrite(LEDRED, HIGH);
    }
}

                                                             // TASK ERA CHẠY CORE 0
void TaskEra(void * pvParameters) {
   (void) pvParameters;
    ERa.begin(ssid, password);
    timer.setInterval(1000L, timerEvent);
    for(;;) {
        ERa.run();
        timer.run();
        vTaskDelay(10);      
    }
}

void loop() {
 checkbutton(); 
 updatebutton(); 
 timeCheck();
 checkOnoff();
 updateDataToScreen();
 CheckMinMax();
 vTaskDelay(10);
}

