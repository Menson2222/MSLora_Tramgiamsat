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

// ADD LIBRARY
#include "SoftwareSerial.h"
#include "DFRobot_EC.h"
#include <EEPROM.h>
#include <OneWire.h> 
#include <avr/pgmspace.h>
#include <DallasTemperature.h>

//UART ESP32 - LORA
#define TXLORA 8
#define RXLORA 7

//RELAY
#define LEDRED 2
#define VSCB 6

//ANALOG PIN
#define DO_PIN A3
#define TO_PIN A4
#define EC_PIN A6
#define PH_PIN A5
#define VREF 5000  
#define Offset 1.20    
#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}
#define ReceivedBufferLength 20
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40   
char receivedBuffer[ReceivedBufferLength+1]; 
byte receivedBufferIndex = 0;

#define SCOUNT  30          
int analogBuffer[SCOUNT];    
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

#define SaturationDoVoltageAddress 22         
#define SaturationDoTemperatureAddress 26    
float SaturationDoVoltage,SaturationDoTemperature;
float averageVoltage;

const float SaturationValueTab[41] PROGMEM = {      
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};

//ERORR PIN
#define NGHIENG_PIN 3
#define WATER_PIN 4

// ĐỊNH NGHĨA BIẾN
float ECValue = 0;       
int TDSValue = 0; 
float PHValue = 0; 
float DOValue = 0; 
float TOValue = 0;
float voltage,ecValue,temperature = 25;
int pHArray[ArrayLenth];   
int pHArrayIndex=0;

String Result = "";  
String Data_From_Lora = "";
String ERORR = "";

// SETUP CỔNG GIAO TIẾP UART 2
SoftwareSerial lora(RXLORA, TXLORA); 
DFRobot_EC ec;
OneWire oneWire(TO_PIN); 
DallasTemperature TempSensor(&oneWire);

void setup() {
  Serial.begin(9600); 
  Serial.print("CHECK");
  ec.begin();
  TempSensor.begin();
  lora.begin(9600); 
  lora.print("000.00;000.00;000.00;000.00;000.00");
  pinMode(LEDRED,OUTPUT);
  pinMode(VSCB,OUTPUT);
  pinMode(NGHIENG_PIN, INPUT_PULLUP);
  pinMode(WATER_PIN, INPUT_PULLUP);
  pinMode(EC_PIN, INPUT);
  pinMode(DO_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(TO_PIN, INPUT);
  readDoCharacteristicValues();
} 

void ECcheck() {
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U) 
    { 
      float temperature = TOValue;
      timepoint = millis();
      voltage = analogRead(EC_PIN)/1024.0*5000;   
      ECValue =  ec.readEC(voltage,temperature);  
      Serial.print("temperature:");
      Serial.print(temperature,1);
      Serial.print("^C  EC:");
      Serial.print(ECValue,2);
      Serial.println("ms/cm");
    }
    ec.calibration(voltage,temperature);     
     if (ECValue >= 0.1 && ECValue <= 5) TDSValue = ECValue * 640;  //Công thức chuyển đổi EC sang TDS (mS/cm hay dS/m => mg/L hay ppm)
     else TDSValue = ECValue * 800;
}

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

void PHcheck() {
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(PH_PIN);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      PHValue = pHValue;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    Serial.print("Voltage:");
        Serial.print(voltage,2);
        Serial.print("    pH value: ");
    Serial.println(PHValue,2);
        printTime=millis();
  }
}

float TOcheck() {
  TempSensor.requestTemperatures();
  float tempC = TempSensor.getTempCByIndex(0);  // Gọi một lần duy nhất
  Serial.print(tempC);  // In giá trị nhiệt độ
  TOValue = tempC;  // Gán cho biến TOValue
  return tempC;  // Trả về giá trị nhiệt độ
}

void CheckDATA() {
    if (lora.available()) {
    Serial.println("Cleaning ");
    Data_From_Lora = lora.readString();
    Serial.println(Data_From_Lora);
    if (Data_From_Lora.indexOf("ONNNN") >= 0) {
      Serial.println("Cleaning...");
      digitalWrite(VSCB, HIGH);
      delay(8000);
      digitalWrite(VSCB, LOW);
      Serial.println("Cleaned!");
      lora.print("000.00;000.00;000.00;000.00;000.00;OFFFF");
      delay(3000);
      lora.print("000.00;000.00;000.00;000.00;000.00");
    } else {
      digitalWrite(VSCB, LOW);
    }
  }
}

void ERORRcheck() {
bool a = digitalRead(WATER_PIN);
bool b = digitalRead(NGHIENG_PIN);
//Serial.println("a=" + String(a));
//Serial.println("b=" + String(b));
if (a==1 && b==1) {
ERORR = "AAAAA";
digitalWrite(LEDRED,LOW);
}
if (a==0 || b==0 || (a==0 && b==0)) {
ERORR = "CCCCC";
digitalWrite(LEDRED,HIGH);
 }
}

void GiveDATA() {
   static unsigned long timepoint = millis();
    if(millis()-timepoint>2000) 
    { 
  Result = String(ECValue) + ";" + String(TDSValue) + ";" + String(DOValue) + ";" + String(TOValue) + ";" + String(PHValue) + ";" + ERORR;
  Serial.println(Result);
  lora.print(Result);
  Serial.println("Sent data!");
        timepoint = millis();
    }
}

void DOcheck()
{
     static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(DO_PIN);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT)
         analogBufferIndex = 0;
   }

   static unsigned long tempSampleTimepoint = millis();
   if(millis()-tempSampleTimepoint > 500U)  // every 500 milliseconds, read the temperature
   {
      tempSampleTimepoint = millis();
      temperature = TOcheck();  // add your temperature codes here to read the temperature, unit:^C
   }

   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }
averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the value more stable by the median filtering algorithm
      Serial.print(F("Temperature:"));
      Serial.print(temperature,1);
      Serial.print(F("^C"));
      DOValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageVoltage / SaturationDoVoltage;
      Serial.print(F(",  DO Value:"));
      Serial.print(DOValue,2);
      Serial.println(F("mg/L"));
   }

   if(serialDataAvailable() > 0)
   {
      byte modeIndex = uartParse();  //parse the uart command received
      doCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
   }
}

boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while ( Serial.available() > 0 )
  {
    if (millis() - receivedTimeOut > 500U)
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
    {
    receivedBufferIndex = 0;
    strupr(receivedBuffer);
    return true;
    }else{
        receivedBuffer[receivedBufferIndex] = receivedChar;
        receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
    byte modeIndex = 0;
    if(strstr(receivedBuffer, "CALIBRATION") != NULL)
        modeIndex = 1;
    else if(strstr(receivedBuffer, "EXIT") != NULL)
        modeIndex = 3;
    else if(strstr(receivedBuffer, "SATCAL") != NULL)
        modeIndex = 2;
    return modeIndex;
}

void doCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean doCalibrationFinishFlag = 0,enterCalibrationFlag = 0;
    float voltageValueStore;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;

      case 1:
      enterCalibrationFlag = 1;
      doCalibrationFinishFlag = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
      Serial.println();
      break;

     case 2:
      if(enterCalibrationFlag)
      {
         Serial.println();
         Serial.println(F(">>>Saturation Calibration Finish!<<<"));
         Serial.println();
         EEPROM_write(SaturationDoVoltageAddress, averageVoltage);
         EEPROM_write(SaturationDoTemperatureAddress, temperature);
         SaturationDoVoltage = averageVoltage;
         SaturationDoTemperature = temperature;
         doCalibrationFinishFlag = 1;
      }
      break;

        case 3:
        if(enterCalibrationFlag)
        {
            Serial.println();
            if(doCalibrationFinishFlag)
Serial.print(F(">>>Calibration Successful"));
            else
              Serial.print(F(">>>Calibration Failed"));
            Serial.println(F(",Exit Calibration Mode<<<"));
            Serial.println();
            doCalibrationFinishFlag = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}

int getMedianNum(int bArray[], int iFilterLen)
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
      bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
      for (i = 0; i < iFilterLen - j - 1; i++)
          {
        if (bTab[i] > bTab[i + 1])
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void readDoCharacteristicValues(void)
{
    EEPROM_read(SaturationDoVoltageAddress, SaturationDoVoltage);
    EEPROM_read(SaturationDoTemperatureAddress, SaturationDoTemperature);
    if(EEPROM.read(SaturationDoVoltageAddress)==0xFF && EEPROM.read(SaturationDoVoltageAddress+1)==0xFF && EEPROM.read(SaturationDoVoltageAddress+2)==0xFF && EEPROM.read(SaturationDoVoltageAddress+3)==0xFF)
    {
      SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
      EEPROM_write(SaturationDoVoltageAddress, SaturationDoVoltage);
    }
    if(EEPROM.read(SaturationDoTemperatureAddress)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+1)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+2)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+3)==0xFF)
    {
      SaturationDoTemperature = 25.0;   //default temperature is 25^C
      EEPROM_write(SaturationDoTemperatureAddress, SaturationDoTemperature);
    }
}

void loop() {
  TOcheck();
  PHcheck();
  ECcheck();
  DOcheck();
  CheckDATA();
  GiveDATA();
  ERORRcheck();
}


