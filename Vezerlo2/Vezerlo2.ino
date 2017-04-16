#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Math.h>
#include "Time.h"
#include "Solarlib.h"
#include "Adafruit_PWMServoDriver.h"
#include <SPI.h>
#include <SD.h>

#define timeZoneOffset +2

#define SDCardCsPin 4
#define SDCardClock 13

#define ServoAddress 0x6F
#define SERVOMIN 90
#define SERVOMAX 440
#define SZOGMIN 10
#define SZOGMAX 170
#define SZOGKOZEP 90

#define SZOGDEFAULT 90

#define ServoED 0
#define ServoKNY 1

#define voltmeter_pin A0

static const int RXPin = 9, TXPin = 8;
static const uint32_t GPSBaud = 9600;
static const double szogenkent=(SERVOMAX-SERVOMIN)/180.0;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(ServoAddress);

// For stats that happen every 5 seconds
unsigned long last = 0UL;
int degreeED;
int degreeKNY;


int old_ED=90;
int old_KNY=90;

File myFile;
int x=0;

int szamlalo=0;
//********************************************************************************************
//Alap beállítások
void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);

  pwm.begin();
  pwm.setPWMFreq(50);

  
  //yield();
//  servo_bolintas();
  
  
  if (!SD.begin(SDCardClock,SDCardCsPin)){
    Serial.println("SD Error");
  }
}

//********************************************************************************************
//Servo alap bólintása
/*void servo_bolintas(){
  delay(1000);
  moveServos(SZOGKOZEP,SZOGKOZEP);
  delay(1000);
  //Észak-dél bólintás
  moveServos(SZOGMIN,SZOGKOZEP);
  delay(1000);
  moveServos(SZOGMAX,SZOGKOZEP);
  delay(1000);
  //vissza
  moveServos(SZOGKOZEP,SZOGKOZEP);
  delay(1000);
  //Kelet-Nyugat bólintás
  moveServos(SZOGKOZEP,SZOGMIN);
  delay(1000);
  moveServos(SZOGKOZEP,SZOGMAX);
  delay(1000);
  //vissza
  moveServos(SZOGKOZEP,SZOGKOZEP);
  delay(1000);
}*/

//********************************************************************************************
//Fő loop
void loop() {
  while (ss.available() > 0)
    gps.encode(ss.read());

  if(x==1){
  szamlalo++;
  delay(10000);
}

if(szamlalo==60){
  x=0;
  szamlalo=0;
}
  
  if ((gps.location.isUpdated())&&(gps.date.isValid())&&(gps.time.isValid())&&(x==0))
  {
	//***********************************
	//****Bemozgatott mérés
	//Létrehozom a változókat
	double latt=gps.location.lat();
	double lngg=gps.location.lng();
	uint8_t dayy=gps.date.day();
	uint8_t monthh=gps.date.month();
	uint16_t yearr=gps.date.year();
	uint8_t hourr=gps.time.hour();
	uint8_t minutee=gps.time.minute();
	uint8_t secondd=gps.time.second();
	uint8_t centisecondd=gps.time.centisecond();
	//kiszámítom a szögeket
    calculateDegree(latt,lngg,
                    dayy,monthh,yearr,
                    hourr,minutee,secondd,centisecondd,
                    degreeED,degreeKNY);
	//Bemozgatom a szervót
	moveServos(degreeED,degreeKNY);
	delay(5000);
	//mérés
	float data=metric();
	//mentés
	printto_normal(yearr,monthh,dayy,hourr,minutee,degreeED,degreeKNY,data);
	delay(1000);
  Serial.println(degreeED);
  Serial.println(degreeKNY);

	//***********************************
	//*****Adott pozícióban mérés
	//Bemozgatás
	moveServos(140,SZOGKOZEP);
	delay(5000);
	//mérés
	data=metric();
	//mentés
	printto_notnormal(yearr,monthh,dayy,hourr,minutee,degreeED,degreeKNY,data);
	delay(1000);
  x=1;
  }
}

//**************************************************************************************************
//Pozíciószámítás
void calculateDegree(double lat, double lng,
                     uint8_t day, uint8_t month, uint16_t year,
                     uint8_t hour, uint8_t minute, uint8_t second, uint8_t centisecond,
                     int &degreeED, int &degreeKNY){      
   tmElements_t t1;
   t1.Second=second; 
   t1.Minute=minute; 
   t1.Hour=hour; 
   //uint8_t Wday;   // day of week, sunday is day 1
   t1.Day=day;
   t1.Month=month; 
   t1.Year=year-1970; 
   time_t t=makeTime(t1);
   initSolarCalc(timeZoneOffset,lat,lng);
   setTime(t);
   int alfa=getSAA(t);
   bool E_D;
   if(alfa>180){
    E_D=true;
   }else{
    E_D=true;
   }
   if(alfa>=0&&alfa<90){
    //E_D=true;
   }else {
    if(alfa>=90&&alfa<=180){
      //E_D=false;
      alfa=179-alfa;
    }else {
      if(alfa>180&&alfa<=270){
        //E_D=false;
        alfa-=181;
      }else {
        if(alfa>270&&alfa<=360){
          //E_D=true;
          alfa=269-alfa;
          //alfa-=181;
        }
      }
    }
   }   
   double beta_rad=getSEA(t)*M_PI/180.0;
   double alfa_rad=alfa*M_PI/180.0;
   degreeED=atan(cos(M_PI/2-beta_rad)/cos(alfa_rad)/cos(beta_rad))*180.0/M_PI;
   if(E_D)
    degreeED=180-degreeED;

   //Eddig az Észak-Dél szöget számoltam
   //*************************************************************************************************
   //Kelet-Nyugat
   alfa=getSAA(t);
   bool K_NY;
   if(alfa>180){
    K_NY=true;
   }else{
    K_NY=false;
   }
   if(alfa>0&&alfa<=90){
    alfa=90-alfa;
   }else {
    if(alfa>90&&alfa<=180){
      alfa-=90;
    }else {
      if(alfa>180&&alfa<=270){
        alfa=270-alfa;
      }else {
        if(alfa>270&&alfa<=360){
          alfa-=270;
        }
      }
    }
   }
   alfa_rad=alfa*M_PI/180.0;
   degreeKNY=atan(cos(M_PI/2-beta_rad)/cos(alfa_rad)/cos(beta_rad))*180.0/M_PI;
   if(K_NY)
    degreeKNY=180-degreeKNY;
}

//********************************************************************************************************
//Szervo-k bemozgatása szögekre
bool moveServos(int degreeED, int degreeKNY){
  if(degreeED<SZOGMIN||degreeED>SZOGMAX||degreeKNY<SZOGMIN||degreeKNY>SZOGMAX){
  
  }else{
    int ertek;
    if(old_ED<degreeED){
      for(int i=old_ED;i<=degreeED;i++){
        ertek=i*szogenkent+SERVOMIN;
        pwm.setPWM(ServoED,0,ertek);
        delay(10);
      }
    }else{
      for(int i=old_ED;i>=degreeED;i--){
        ertek=i*szogenkent+SERVOMIN;
        pwm.setPWM(ServoED,0,ertek);
        delay(10);
      }
    }
    old_ED=degreeED;
    delay(500);
    if(old_KNY<degreeKNY){
      for(int i=old_KNY;i<=degreeKNY;i++){
        ertek=i*szogenkent+SERVOMIN;
        pwm.setPWM(ServoKNY,0,ertek);
        delay(10);
      }
    }else{
      for(int i=old_KNY;i>=degreeKNY;i--){
        ertek=i*szogenkent+SERVOMIN;
        pwm.setPWM(ServoKNY,0,ertek);
        delay(10);
      }
    }
    old_KNY=degreeKNY;
  }
}

//**************************************************************************************************
//Mérés, visszaadja a mért feszültséget
float metric(){
  int sensorvalue=analogRead(voltmeter_pin);
  float b=sensorvalue*(5.0/1023.0);
  Serial.println(b);
  return b;
}                                   

//**************************************************************************************************
//Fájlba belementi az adatot  
void printto_normal(const int &dayy, const int &monthh, const int &yearr, 
            const int &hourr, const int &minn, 
            const int &degreeED, const int &degreeKNY, const float &volt){
  myFile=SD.open("Normal.txt", FILE_WRITE);
  Serial.println("Create testfile");
  if (myFile) {
    Serial.println("Created and writed");
    
    myFile.print(dayy);
    myFile.print(".");
    myFile.print(monthh);
    myFile.print(".");
    myFile.print(yearr);
    myFile.print(".");
    myFile.print("\t");
    myFile.print(hourr);
    myFile.print(":");
    myFile.print(minn);
    myFile.print("\t");
    myFile.print(degreeED);
    myFile.print("\t");
    myFile.print(degreeKNY);
    myFile.print("\t");
    myFile.print(volt);
    myFile.print("\n");
    myFile.close();
  }else{
    Serial.println("Not created");
  }
}

//**************************************************************************************************
//Fájlba belementi az adatot  
void printto_notnormal(const int &dayy, const int &monthh, const int &yearr, 
            const int &hourr, const int &minn, 
            const int &degreeED, const int &degreeKNY, const float &volt){
  myFile=SD.open("Beall.txt", FILE_WRITE);
  Serial.println("Create testfile");
  if (myFile) {
    Serial.println("Created and writed");
    myFile.print(dayy);
    myFile.print("\t");
    myFile.print(monthh);
    myFile.print("\t");
    myFile.print(yearr);
    myFile.print("\t");
    myFile.print(hourr);
    myFile.print(":");
    myFile.print(minn);
    myFile.print("\t");
    myFile.print(degreeED);
    myFile.print("\t");
    myFile.print(degreeKNY);
    myFile.print("\t");
    myFile.print(volt);
    myFile.print("\t");
    myFile.println("\n");
    myFile.close();
  }else{
    Serial.println("Not created");
  }
}

