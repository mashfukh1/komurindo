/*
    LIBRARY
*/
#include <Adafruit_VC0706.h>
#include "I2Cdev.h"
#include "ADXL345.h"
#include "Wire.h"

/*
    CONFIG CONSTANT VALUE
*/
#define GPS_BAUD    38400
#define TRX_BAUD    57600
#define IMU_BAUD    57600
#define TRX_INTERVAL    26000
#define GPS_RATE    1000000
#define PARASUT_ON  300

#define IMU_CONF    "#osct#o0"
#define IMG_RES_CONF VC0706_160x120
// TELCOM
#define START       'A'       //mulai transmisi
#define STOP        'B'       //Stop transmisi
#define PICTURE     'C'       //ambil gambar
#define NULL        '0'       //bersihkan telcom

//TELCOM ADV
#define CAM_STAT    'Z'
#define IMU_STAT    'Y'
#define GPS_STAT    'X'

//INDEX SETUP
#define CAM_INDEX   0
#define GPS_INDEX   1
#define IMU_INDEX   2

/*
    PIN CONFIGURATION
*/
#define GPS   Serial1
#define TRX   Serial
#define IMU   Serial2
#define CAMERA_PIN  Serial3
#define PARASUT_SENS  A8
/*
    GLOBAL VARIABEL
*/
struct dataGPS{
  float
    longi,    // longitude
    lati,     // latitude
    alti,     // altitude
    speed,    // speed
    head;     // heading

  int
    sat;

  uint8_t
    fixQ,
    hour,
    minute,
    second;
} dataGPS;


bool
  camStat = false,
  gpsStat = false,
  imuStat = false;

uint8_t imgSize;

char camVersion[100];

volatile unsigned char
    telcom = NULL;

unsigned long gpsRTime, imuRTime;

/*
    PEWARISAN SIFAT
*/
Adafruit_VC0706 CAM = Adafruit_VC0706(&CAMERA_PIN);
ADXL345 accel;
/*
    FUNCTION PROTOTYPE
*/
void setup(void);
void loop(void);
void serialEvent(void);
void serialEvent1(void);
void initDataGPS(void);
void camSetup(void);
void camCheck(void);
void imuSetup(void);
void imuCheck(void);
void gpsCheck(void);
void updateGPS(void);
void sendGPS(void);
void sendIMU(void);
void takePicture(void);
void tryOut(void);
void clrTelcom(void);
uint16_t parasutStat(void);

/*
    FUNCTION IMPELENTATION
*/

//Konfigurasi arduino
void setup(void){
  Wire.begin();
  TRX.begin(TRX_BAUD);
  GPS.begin(GPS_BAUD);
  pinMode(PARASUT_SENS, INPUT_PULLUP);
  delay(500);
  imuSetup();
  camSetup();
  accel.initialize();
  accel.testConnection();
  delay(100);
  accel.setRange(ADXL345_RANGE_8G);
  gpsRTime = imuRTime = micros();
 }

//fungsi utama
void loop(void){
  sendGPS();
//  static unsigned long ctime;
//  static String dataIMU;
//  static int16_t ax, ay, az;
//
//  if(IMU.available()){
//    dataIMU = IMU.readStringUntil('\n');
//    IMU.print("#f");
//    accel.getAcceleration(&ax, &ay, &az);
//  }
//    
//  switch (telcom) {
//    case NULL:
//      delay(50);
//      break;
//
//    case START:
//      ctime = micros();
//      if((ctime - gpsRTime) > GPS_RATE){
//        sendGPS();
//        gpsRTime = micros();
//      }
//      if((ctime - imuRTime) > TRX_INTERVAL){
//        TRX.print("I,");
//        TRX.print(ax);
//        TRX.print(',');
//        TRX.print(ay);
//        TRX.print(',');
//        TRX.print(az);
//        TRX.print(',');     
//        TRX.println(dataIMU);
//        imuRTime = micros();
//      }
//      break;
//
//    case STOP:
//        delay(50);
//      break;
//
//    case PICTURE:
//      takePicture();
//      clrTelcom();
//      telcom = START;
//      break;
//
//    case CAM_STAT:
//      camCheck();
//      telcom = NULL;
//      break;
//
//    case IMU_STAT:
//      imuCheck();
//      telcom = NULL;
//      break;
//
//    case GPS_STAT:
//      gpsCheck();
//      telcom = NULL;
//      break;
//  }
}

// inisialisasi dataGPS
void initDataGPS(void){
  dataGPS.longi = 0;
  dataGPS.lati = 0;
  dataGPS.alti = 0;
  dataGPS.speed = 0;
  dataGPS.head = 0;
  dataGPS.sat = 0;
  dataGPS.hour = 0;
  dataGPS.minute = 0;
  dataGPS.second = 0;
}

//  Interrupt telecommand (Antena)
void serialEvent2(void){
  if(TRX.available()){
    telcom = TRX.read();
  }
}

void clrTelcom(void){
  while(TRX.available())
    (void)TRX.read();
}

// Interrupt GPS
void serialEvent1(void){
  while (GPS.available())
    gpsENC(GPS.read());
}

//Konfigruasi dan inisialisasi kamera
void camSetup(void){
  char * ver;
  if(CAM.begin())
    camStat = 1;
  else
    camStat = 0;

  ver = CAM.getVersion();
  if(ver)
    sprintf(camVersion, "%s", ver);
  else
    sprintf(camVersion, "%s", "Failed to get Version");

  CAM.setImageSize(VC0706_160x120);
  imgSize = CAM.getImageSize();
  //delay(500);
  //CAM.setBaud115200();
}

//Check status kamera
void camCheck(void){
  if (camStat){
    TRX.println("Camera OK");

    TRX.println("Camera Version: ");
    TRX.print(camVersion);

    TRX.print("Image size: ");
    if (imgSize == VC0706_640x480) TRX.println("640x480");
    else if (imgSize == VC0706_320x240) TRX.println("320x240");
    else if (imgSize == VC0706_160x120) TRX.println("160x120");
    else{
      TRX.print(imgSize);
      TRX.print("\t");
      TRX.println("SIZE NOT RECOGNIZED");
    }
  }
  else
    TRX.println("Camera Failed, Check Wiring and Program");
}

void imuSetup(void){
  IMU.begin(IMU_BAUD);
  delay(100);

  IMU.print(IMU_CONF);
  delay(50);
  IMU.print("#f");
}

void imuCheck(void){
  long timeOut;

  timeOut = millis();
  delay(10);
  IMU.print("#f");

  imuStat = true;
  while(IMU.available() < 0){
    if(millis() - timeOut > 3000){
      imuStat = false;
      break;
    }
  }

  if(imuStat){
    TRX.print("IMU OK, data: ");
    TRX.println(IMU.readStringUntil('\n'));
  }
  else
    TRX.println("IMU Failed, Check Wiring and Program");
}


void gpsCheck(void){
  if(gpsStat){
    TRX.print("GPS OK: ");
  }
  else{
    TRX.println("GPS FAILED, Check Wiring and Program!");
  }
}

/*
void updateGPS(void){
  if(gpsENC.location.isValid()){
    dataGPS.longi = gpsENC.location.lng();
    dataGPS.lati = gpsENC.location.lat();
  }

  if(gpsENC.altitude.isValid())
    dataGPS.alti = gpsENC.altitude.meters();

  if(gpsENC.speed.isValid())
    dataGPS.speed = gpsENC.speed.kmph();

  if(gpsENC.course.isValid())
    dataGPS.head = gpsENC.course.deg();

  if(gpsENC.satellites.isValid())
    dataGPS.sat = gpsENC.satellites.value();

  if(gpsENC.time.isValid()){
    dataGPS.hour = gpsENC.time.hour();
    dataGPS.minute = gpsENC.time.minute();
    dataGPS.second = gpsENC.time.second();
  }
}
*/

void sendGPS(void){
  String packet;
  TRX.print("G,");
  TRX.print(dataGPS.longi, 6); TRX.print(",");
  TRX.print(dataGPS.lati, 6); TRX.print(",");
  TRX.print(dataGPS.alti, 2); TRX.print(",");
  TRX.print(dataGPS.speed, 2); TRX.print(",");
  TRX.print(dataGPS.head, 2); TRX.print(",");
  TRX.print(dataGPS.sat); TRX.print(",");
  TRX.print(dataGPS.fixQ); TRX.print(",");
  TRX.print(dataGPS.hour); TRX.print(",");
  TRX.print(dataGPS.minute); TRX.print(",");
  TRX.print(dataGPS.second); TRX.print(",");
  TRX.println(digitalRead(PARASUT_SENS));
}

void sendIMU(void){

}

void takePicture(void){
  CAM.resumeVideo();
  delay(50);
  if (!CAM.takePicture())
    TRX.println(0);

  long imglen = CAM.frameLength();
  uint8_t *buf;
  long len;

  TRX.println(imglen);


  char ct = 0;

  while(imglen > 0){
    len = min(64, imglen);
    buf = CAM.readPicture(len);

    int i = 0;
    imglen -= len;

    TRX.write(buf, len);
  }
}

uint16_t parasutStat(void){
  return (analogRead(PARASUT_SENS));
}

