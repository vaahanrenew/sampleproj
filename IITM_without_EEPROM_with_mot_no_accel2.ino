//Problem observed is that millis loop runs after srt wakeup of 1 hour

// Created: 13-08-2016 19:39:44
// Author: VAAHAN RENEW ENERGY PVT. LTD.

//(HIGH,LOW) combination implies from LM1 to LM2 direction
//(HIGH,HIGH) combination implies from LM2 to LM1 direction
#include <TinyGPS++.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Time.h>
#include  <SPI.h>
#include "LowPower.h"     
#include <RTClib.h>    
#include <MMA_7455.h>
#include <avr/wdt.h>


//const int chipSelect = 10;    //CS moved to pin 10 on the arduino
const int artificial_sqw=9;
const int IN1 = 4;//5;  // MCU Digital Pin 9 to IN1 on L298n Board
const int IN2 = 11;//8;  // MCU Digital Pin 8 to IN2 on L298n Board

unsigned long button_time = 0;  
unsigned long last_button_time = 0;  

byte Alarmhour;
byte Alarmminute;
byte Alarmday;
byte tMSB = 0;
byte tLSB = 0;

char CycleTimeStamp[ ] = "0000/00/00,00:00"; //16 ascii characters (without seconds)
char temp, NOW;
char xVal, yVal, zVal; //Variables for the values from the sensor
char xVal0;

volatile double SampleIntervalMinutes;
volatile boolean clockInterrupt = false;  
//this flag is set to true when the RTC interrupt handler is executed

//variables for reading the DS3231 RTC temperature register
float temp3231;
long Error[10];


#define DS3231_I2C_ADDRESS 0x68
#define GPSBAUD 9600 //GPS baud rate usually 4800 or 9600.
#define Rx 7 //SoftwareSerial Rx pin
#define Tx 6 //SoftwareSerial Tx pin
#define DS3231_I2C_ADDRESS 0x68

double fractional_year; //Fractional year
double eqtime; //Equation of Time
double decl; //Solar declination angle
double time_offset; //Time offset
double tst; //True solar time
double sha; //Solar hour angle
double sza; //Solar Zenith angle
double sea;
double saa; //Solar Azimuth angle
double ha; //Hour angle
double gamma=180;//tracker axis azimuth
double srt;
double sst;
double sn; //Solar noon
double latitude;
double longitude;
double timezone;
double c;
double stta;
double spta;
double holdtime;
double sea0;
double srtX;
double sstX;
double t_s=0.001;
double u=0;
double u_1=0;
double k_p;
double k_i;
double k_d;
double ref_0;
double e_new=0.0;
double hrtable[11]={7, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18};
double ref[11] = {45, 35, 25, 15, 5, 0, -5, -15, -25, -35 ,-45};
double DesiredPosition=0.0;
//R is the rate of actuation depending on the length of the actuator
double R=2.00; 
double average_time[8];

int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int j;
int RTC_INTERRUPT_PIN = 2;
int gpsChkSum, rtcChkSum;
int buttonState = 0;         // variable for reading the pushbutton status
int S0=A0; 
int S1=A1;
int S2=A2;
int S3=A3;

RTC_DS3231 RTC;
SoftwareSerial gpss(Rx, Tx);
TinyGPSPlus gps;
SoftwareSerial Relay(IN1,IN2);

MMA_7455 mySensor = MMA_7455(); //Make an instance of MMA_7455


// ===                   FIRST INSTALL RUN                  ===
// ================================================================
void setup(){
  Serial.begin(9600);    // Open serial communications and wait for port to open:
  delay(500);
  Wire.begin();          // start the i2c interface for the RTC
  RTC.begin();           // start the RTC
  gpss.begin(GPSBAUD);
  Serial.println();
  updateRTC(); // Gather data, update.
  pinMode(IN1, OUTPUT); //Motor relay
  pinMode(IN2, OUTPUT); //Motor relay
  pinMode(artificial_sqw,OUTPUT);
  digitalWrite(artificial_sqw,LOW);
  mySensor.initSensitivity(2); // Good for "Tilt" measurements
  xVal0 = mySensor.readAxis('x'); //Read out the 'x' Axis
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
} // end of setup

// ================================================================
// ===                    EVERYDAY ITERATIVE LOOP                     ===
// ================================================================
void loop() {
  Serial.println("loop");
  //Generate SQUARE WAVE
for(int m=0;m<25;m++)
{
digitalWrite(artificial_sqw,HIGH);
delay(50);
digitalWrite(artificial_sqw, LOW);
delay(50);
digitalWrite(artificial_sqw,HIGH);
delay(50);
digitalWrite(artificial_sqw, LOW);
delay(50);
digitalWrite(artificial_sqw,HIGH);
delay(50);
digitalWrite(artificial_sqw, LOW);
delay(50);
digitalWrite(artificial_sqw,HIGH);
delay(50);
digitalWrite(artificial_sqw, LOW);
}
DateTime now = RTC.now(); 
double x=(double)now.hour()+((double)(now.minute())/60);
Serial.println("x");
int d0=now.day();
int d1=d0%10;
astro(x);
//check if its a day that ends with number 5
if(millis()<100000 || (x==11 && d1==5))
{
 if(x<sstX && x>c)
 {
  Serial.println("MILLS");
  gpss.begin(GPSBAUD);
    Serial.println();
    updateRTC(); // Gather data, update.
  if (rtcChkSum == gpsChkSum)
   {
    Serial.println("GPS");
    delay(500);
//    while (1) {}; //Success, do nothing. Forever.
   }
//Run motor 130 secs so that it has reached its limit point LM1.
  move_130();
  delay(500);
  mySensor.initSensitivity(2); // Good for "Tilt" measurements
  xVal0 = mySensor.readAxis('x'); //Read out the 'x' Axis
  NOW=xVal0;
  Serial.println(xVal0,DEC);

if(NOW>=0)
{
int error2=45-NOW;
 mySensor.initSensitivity(2); // Good for "Tilt" measurements
 mySensor.calibrateOffset(error2,0,+50);
 NOW = mySensor.readAxis('x'); //Read out the 'x' Axis
Serial.println("err2");
Serial.println(error2);
}
if(NOW<0)
{
    int error3=45+abs(NOW);
    mySensor.initSensitivity(2); // Good for "Tilt" measurements
    mySensor.calibrateOffset(error3,0,+50);
    NOW = mySensor.readAxis('x'); //Read out the 'x' Axis
    Serial.println("err3");
    Serial.println(error3);
}
delay(500);
  Serial.println(NOW,DEC);
  Serial.println("AFTmove");
//theta_i correction once in 10 days below:
  for(int i=0;i<10;i++)
  {
                average_time[i]={};
                average_time[i]=(hrtable[i]+hrtable[i+1])/2.0;
    if(hrtable[i]<=x && x<=average_time[i])
         {            
                 Serial.println("hrtable");
                 Serial.println(hrtable[i]);
                 ref_0=-(double)ref[i];
                 Serial.println("ref_0");
                 Serial.println(ref_0);
                 e_new=ref_0-NOW;
         }
    else if(average_time[i]<=x && x<=hrtable[i+1])
    {
                 ref_0=-(double)((ref[i]+ref[i+1])/2);
                 e_new=ref_0-NOW;
                 Serial.println("ref_0");
                 Serial.println(ref_0);
                 Serial.println("xVal");
                 Serial.println(NOW,DEC);
    }
  }

if(e_new<0)
{
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
Serial.println("Err pos");
double ll=abs(e_new)*R*1000.00;
Serial.println("ref");
Serial.println(ref_0);
Serial.println("xVal");
Serial.println(NOW,DEC);
Serial.println(ll);
sleep(ll);//Relays have to go back to LOW LOW
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
}
if(e_new>0)
{
digitalWrite(IN1, HIGH);
digitalWrite(IN2, HIGH);
Serial.println("Err neg");
double ll=abs(e_new)*R*1000.00;
Serial.println("ref");
Serial.println(ref_0);
Serial.println("xVal");
Serial.println(NOW,DEC);
Serial.println(ll);
sleep(ll);
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
}
}
}//end of 10 days code
SampleIntervalMinutes=30.0;

 //This part reads the time and disables the RTC alarm
   // We set the clockInterrupt in the ISR, deal with that now:
  if (clockInterrupt) {
    if (RTC.checkIfAlarm(1)) {       //Is the RTC alarm still on?
      RTC.turnOffAlarm(1);              //then turn it off.
    }
    //Serial.print("RTC Alarm on INT-0 triggered at ");
    //Serial.println(CycleTimeStamp);
    clockInterrupt = false;                //reset the interrupt flag to false
  }
  // read the RTC temp register and print that out
  // Note: the DS3231 temp registers (11h-12h) are only updated every 64seconds
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);                     //the register where the temp data is stored
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);   //ask for two bytes of data
  Relay.begin(GPSBAUD);
  MorningAlarm();
  mySensor.initSensitivity(2); // Good for "Tilt" measurements
  xVal0 = mySensor.readAxis('x'); //Read out the 'x' Axis
  temp=xVal0;
  Serial.println(temp,DEC);
  Serial.println("aft Mrngalrm");
  Relay.end();
//combine below two in one if loop

if(x>=sstX)
{
  SampleIntervalMinutes=30.0;
  Serial.println("ins x>sst ");
  Serial.println(sstX);
  MorningAlarm(); //This should bring the panel back to flat position
}

//What happens at the immediate point of next day sunrise time
if(x<c)
{
  SampleIntervalMinutes=30.0;
  Serial.println("inside x<c");
  MorningAlarm(); //WHY
}
//This brings the panels from 0degree flat position to the east limit switch position right after sunrise time, here indicated by 'c'
  if((x>c+0.4) && x<(c+1.0)){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    sleep(100000);
  delay(500);
  mySensor.initSensitivity(2); // Good for "Tilt" measurements
  xVal0 = mySensor.readAxis('x'); //Read out the 'x' Axis
  temp=xVal0;
  Serial.println(temp,DEC);
  Serial.println("inside morn hr");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  SampleIntervalMinutes=30.0;
  Serial.println(sstX);
  }
//Where is the code to move the motors in between c and sst?
Alarmhour = now.hour();
Alarmminute = now.minute() + SampleIntervalMinutes;
Alarmday = now.day();

// check for roll-overs
if (Alarmminute > 59) { //error catching the 60 rollover!
  Alarmminute = 0;
  Alarmhour = Alarmhour + 1;
  if (Alarmhour > 23) {
    Alarmhour = 0;
    // put ONCE-PER-DAY code here -it will execute on the 24 hour rollover
  }
}
// then set the alarm
RTC.setAlarm1Simple(Alarmhour, Alarmminute);
RTC.turnOnAlarm(1);
if (RTC.checkAlarmEnabled(1)) {
  Serial.print("Goingsleepfor:");
  Serial.print(SampleIntervalMinutes);
  Serial.println("min");
  Serial.println();                                      //just adds a carriage return
}
delay(100); //this delay is only here so we can see the LEDÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢s it is totally optional!
  // Enable interrupt on pin2 & attach it to rtcISR function:
  attachInterrupt(0, rtcISR, LOW);
  // Enter power down state with ADC module disabled to save power:
  LowPower.powerDown(SLEEP_FOREVER, ADC_ON, BOD_ON);
  //processor starts HERE AFTER THE RTC ALARM WAKES IT UP
  detachInterrupt(0); // immediately disable the interrupt on waking
}// this is the END of the MAIN LOOP


void rtcISR() {
    clockInterrupt = true;
  }
// ================================================================
// ===                    GPS RTC UPDATE LOOP                     ===
// ================================================================
void updateRTC() {
  //Turn GPS_power and RTC_power to high here?
  unsigned long timer; //Time out. This could actually be annoying.
  int error = 0; // no data counter.
  byte a = 0; // "..." line counter
  bool updated = 0; // do, while flag.
  Serial.println("Findg GPS");
//  Serial.println("...");
  delay(30000); // give the GPS a little time to sort its shit out
  //Do the GPS thing until timeout or success.
  do { 
    while (gpss.available() > 0) //read GPS data
      gps.encode(gpss.read());
      //if both time and date are ready, set the time. 
    if (gps.time.isUpdated() && gps.date.isUpdated()) {  
      byte Year = gps.date.year();
      byte Month = gps.date.month();
      byte Day = gps.date.day();
      byte Hour = gps.time.hour();
      byte Minute = gps.time.minute();
      byte Second = gps.time.second();
      setTime(Hour, Minute, Second, Day, Month, Year);
      latitude = gps.location.lat(); // *latitude in degrees (double)
      longitude = gps.location.lng(); // *longitude in degrees (double)
      int timezone1= (longitude/15.0);
      timezone= timezone1+(((longitude/15)-timezone1)*100/60);//time zone 
      adjustTime(timezone * SECS_PER_HOUR); //Set time to current 
     //Set the RTC to GPS time.
      if (1) { // Enter if conditions to confirm update. Too lazy.
        Wire.beginTransmission(DS3231_I2C_ADDRESS);
        Wire.write(0); // set next input to start at the seconds register
        Wire.write(decToBcd(second())); // set seconds
        Wire.write(decToBcd(minute()));  // set minutes
        Wire.write(decToBcd(hour())); // set hours
        Wire.write(decToBcd(weekday())); // set day of week (1=Sunday, 7=Saturday)
        Wire.write(decToBcd(day())); // set date (1 to 31)
        Wire.write(decToBcd(month())); // set month
        Wire.write(decToBcd(year() - 2000));//-2000 to return only last 2 year digits for RTC. I'll be dead long before this matters.
        Wire.endTransmission();
 //       Serial.println();
        Serial.println ("RTCUpdat");
      }
      updated = 1; //update flag true to break while loop
      //Retrive the updated RTC Time.
  
      Serial.println();
      Serial.println("GPS T");
      Serial.print(hour());
      Serial.print(":");
      if (minute() < 10)
      {
        Serial.print("0");
      }
      Serial.print(minute());
      Serial.print(":");
      if (second() < 10)
      {
        Serial.print("0");
      }
      Serial.print(second());
      Serial.print(" ");
      Serial.print(day());
      Serial.print("/");
      Serial.print(month());
      Serial.print("/");
      Serial.print(year());
      Serial.print("Day of week");
      Serial.print(weekday());
      Serial.print(" ");
      switch (weekday()) {
        case 1:
          Serial.println("Sunday");
          break;
        case 2:
          Serial.println("Monday");
          break;
        case 3:
          Serial.println("Tuesday");
          break;
        case 4:
          Serial.println("Wednesday");
          break;
        case 5:
          Serial.println("Thursday");
          break;
        case 6:
          Serial.println("Friday");
          break;
        case 7:
          Serial.println("Saturday");
          break;
      }
     byte _second, _minute, _hour, dayOfWeek, dayOfMonth, _month, _year;
      readDS3231time(&_second, &_minute, &_hour, &dayOfWeek, &dayOfMonth, &_month, &_year); 
      rtcChkSum = _second + _minute + _hour + dayOfWeek + dayOfMonth + _month + _year;
      gpsChkSum = second() + minute() + hour() + weekday() + day() + month() + year() - 2000;
      // Spew the RTC time
      Serial.println();
      Serial.println("RTCTIME");
      Serial.print(_hour, DEC);
        Serial.print(":");
        if (_minute < 10)
        {
          Serial.print("0");
       }
        Serial.print(_minute, DEC);
        Serial.print(":");
        if (_second < 10)
        {
          Serial.print("0");
        }
        Serial.print(_second, DEC);
        Serial.print(" ");
        Serial.print(dayOfMonth, DEC);
        Serial.print("/");
        Serial.print(_month, DEC);
        Serial.print("/");
        Serial.print(_year + 2000, DEC);
        Serial.print("day of week");
        Serial.print(dayOfWeek);
        Serial.print(" ");
        switch (dayOfWeek) {
          case 1:
            Serial.println("Sunday");
            break;
          case 2:
            Serial.println("Monday");
            break;
          case 3:
            Serial.println("Tuesday");
            break;
          case 4:
            Serial.println("Wednesday");
            break;
          case 5:
            Serial.println("Thursday");
            break;
          case 6:
            Serial.println("Friday");
            break;
          case 7:
            Serial.println("Saturday");
            break;
        }
      }
    else {
      if (gps.charsProcessed() < 20) {
        Serial.println(("WARNING:No GPS data"));
        if (error > 10) {
          Serial.println("Clearly something is wrong");
          error = 0;
          delay(1000);
        }
        Serial.println("Findg GPS");
        delay(500);
        error++;
      }
      else {
        if (a == 0) {
          Serial.println("GPSfnd");
          timer = millis();
        }
        Serial.print("...");
        a++;
        if (a > 15) {
          Serial.println();
          a = 1;
        }
/////Delete this block to make program run indefinetly until RTC is updated.///
//        if (millis() - timer > 10000) {
//          Serial.println("");
//          Serial.println("Timeout. Try again.");
//          failure();
//        }
      }
    }
  }
  while (!updated);
}

byte decToBcd(byte val)
{
  return ( (val / 10 * 16) + (val % 10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}
// ================================================================
// ===                    MOTOR +28DEG RESET                     ===
// ================================================================
void move_130()
{
  Serial.println("move_130");
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, HIGH);
  delay(60000);  // Delay to 130 seconds
  digitalWrite(IN1, LOW); // Turn the motor off
  digitalWrite(IN2, LOW); // Turn the motor off 
  delay(10000);
  digitalWrite(IN1, HIGH); // Turn HIGH motor A
  digitalWrite(IN2, HIGH);
  delay(60000);  // Delay to 130 seconds
  digitalWrite(IN1, LOW); // Turn the motor off
  digitalWrite(IN2, LOW); // Turn the motor off 
  delay(10000);
}


// ================================================================
// ===                   ASTRO EQUATIONS                    ===
// ================================================================
double astro(double Hour)
{Serial.println("astro");

   int D[]={31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //REPLACE WITH PROGMEM
   DateTime now = RTC.now();
   if((now.year())%4==0)
   {
     D[1]=29;
   }  
   //To calculate the day of the year
   int a=(int)now.day();
   int b=(int)(now.month())-1;
   int day =0;
   for(int i=0;i<b;i++)
   {
     day=day+D[i];// day of the year
   }
   day=day+a;//**DAY OF YEAR
   if(day==1 || day%30==0)
   {
     updateRTC(); // Gather data, update.
   }
   double saa_diff=0.00;// Difference in Solar Azimuth angle for every 15 min
   double sza_old=0.00;//Initializing the Solar zenith angle difference on every day
   double saa_old=180.00;//Initializing the Solar azimuth angle difference on every day
   //A[0]=0;   
//Fractional year
    fractional_year =(2*3.14/365)*(day-1+((Hour-12)/24));
//Equation of Time
    eqtime= 229.18*(0.000075+ 0.001868*cos(fractional_year)- 0.032077*sin(fractional_year)-0.014615*cos(2*fractional_year)-0.040849*sin(2*fractional_year));
//Solar declination angle
    decl = 0.006918-0.399912*cos(fractional_year)+0.070257*sin(fractional_year)-0.006758*cos(2*fractional_year)+0.000907*sin(2*fractional_year)-0.002697*cos(3*fractional_year)+0.00148*sin(3*fractional_year);
//Hour Angle
    ha=(acos((cos(90.8333*3.14/180)/(cos((latitude)*3.14/180)*cos(decl)))-(tan(latitude*3.14/180)*tan(decl))))*180/3.14;

//Sunrise time
      srtX=((720+4*(longitude-ha)- eqtime)/60)-timezone;
//Sunset time
      sstX=((720+4*(longitude+ha)- eqtime)/60)-timezone;
//Solar noon  
      sn=((720+4*(longitude) - eqtime)/60)-timezone;
//Time offset
      time_offset = eqtime - 4*(longitude) + 60*timezone;
//True solar time
      tst = Hour *60 + time_offset;
//Solar hour angle
      sha = (tst/4) - 180;
//Solar Zenith angle
      sza=acos(sin(latitude*3.14/180)*sin(decl) + cos(latitude*3.14/180)*cos(decl)*cos(sha*3.14/180))*180/3.14;
//Solar Elevation angle
  
    sea=90-sza;
    
//Solar Azimuth angle
    saa=acos(((sin(latitude*3.14/180)*cos(sza*3.14/180))-sin(decl))/(cos(latitude*3.14/180)*sin(sza*3.14/180)))*180/3.14;
    //Actuator_movement(H,srt,sst);//need to be checked for sea old with pointer
    //Solar Azimuth angle difference
    saa_diff=saa-saa_old;
    sza_old=sza;
    saa_old=saa;
    if(saa_diff<=0)
    {
      saa=180-saa;
    }
    if(saa_diff>0)
    {
      saa=180+saa;
    }
    c = sunrise(day,latitude,longitude,timezone);
  //midnight stow code
    if(Hour>sstX)
    {
// the below things will be automatically get reassigned to zero after 10 days code gets executed ,,, //we can delete this
     sza_old=0;
     saa_old=0;   
delay(500);
     }  
//Serial.println("success1\n");
//delay(500);
return sea;
}
//calculate the next day sunrise time and return d. This will be used in
double sunrise(int day1,double latitude_1,double longitude_1,double timezone_1)
 {
   double srt_next;
   int day_new=day1+1;
 //Fractional year
    fractional_year =(2*3.14/365)*(day_new-1);
//Equation of Time
    eqtime= 229.18*(0.000075+ 0.001868*cos(fractional_year)- 0.032077*sin(fractional_year)-0.014615*cos(2*fractional_year)-0.040849*sin(2*fractional_year));
//Solar declination angle
    decl = 0.006918-0.399912*cos(fractional_year)+0.070257*sin(fractional_year)-0.006758*cos(2*fractional_year)+0.000907*sin(2*fractional_year)-0.002697*cos(3*fractional_year)+0.00148*sin(3*fractional_year);
//Hour Angle
    ha=(acos((cos(90.8333*3.14/180)/(cos((latitude_1)*3.14/180)*cos(decl)))-(tan((latitude_1)*3.14/180)*tan(decl))))*180/3.14;
//Sunrise time
    srt_next=((720+4*((longitude_1)-ha)- eqtime)/60)-(timezone_1);
//    day=day-1;    
    return srt_next;
}

// ================================================================
// ===                   functions to be called when alarm trigger ===
// ================================================================
void MorningAlarm(){
  DateTime now = RTC.now();
  double H=(double)now.hour()+((double)(now.minute())/60);//Then this should show the decimal digits also no?
  double H0=H-0.25;
  double sea_Morning=astro(H);
  double sea0_Morning=astro(H0);
  //The arguments passed to Actuator_movement are global pointers
  //srtX and sstX are today's sunrise and sunset times
  Actuator_movement(H,srtX,sstX,sea_Morning,sea0_Morning);
Serial.println(sstX);
Serial.println("Morningalrm");
return;
}
// ================================================================
// ===                    FINE MOTOR CONTROL                   ===
// ================================================================
 void Actuator_movement(double hour_act,double srt_act,double sst_act,double sea_act,double sea0_act)
{
  double r = 543.500;// it is in mm
  double v = 5.000;
  double del_theta ;
  del_theta =sea_act-sea0_act;
//Print and check if sea checks out. Test every part of this function.
  double w = v/r; //angular velocity of solar panlel is rad/sec w=delta(theta)/delta(time)= l/(delta(time)*r)
  double Time_on=((del_theta*0.0174)/w)*800.0;
     mySensor.initSensitivity(2);
      xVal0 = mySensor.readAxis('x'); //Read out the 'x' Axis
      temp=xVal0;
      //Go to stow position after sst
if(hour_act>=(srt_act+1.0) && hour_act<sst_act && temp<=45) // Actuator is completely extended state when hour =srt
 {
Serial.print("Timeon\n");
Serial.println(Time_on);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
sleep(abs(Time_on));
//Relays have to go back to LOW LOW
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
return;
}
//this is the problem Dinesh
if((temp<=-45) || hour_act>=sstX || hour_act<=c)
 {
  Serial.print("SSTtouch");
  Serial.println(temp,DEC);
  Serial.println("xVal0");
  Serial.println(xVal0,DEC);
  for(int i=0;i<50;i++)
  {
   if(temp==-3 || temp==-2 || temp==-1 || temp==0 || temp==1 || temp==2 || temp==3)
   {    Serial.println("loopbroken");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        break;
   }
   if(hour_act<=c && temp<=-4)
   break;
   if(hour_act<=c && temp>=4)
   break;
   if(hour_act>=(sstX+2.0) && temp<=-4)
   break;
   if(hour_act>=(sstX+2.0) && temp>=4)
   break;
                Serial.println("temp<0");
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
      mySensor.initSensitivity(2);
      xVal0 = mySensor.readAxis('x'); //Read out the 'x' Axis
      temp=xVal0;
  Serial.println(xVal0,DEC);
  Serial.println(temp,DEC);
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  }

Serial.println("sunrise");
Serial.println(c);
Serial.println(hour_act);
}

//Get time from RTC
  void readDS3231time(byte * second,
                      byte * minute,
                      byte * hour,
                      byte * dayOfWeek,
                      byte * dayOfMonth,
                      byte * month,
                      byte * year)
  {
    //Turn RTC_power to high here?
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(0); // set DS3231 register pointer to 00h
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
    // request seven bytes of data from DS3231 starting from register 00h
    *second = bcdToDec(Wire.read() & 0x7f);
    *minute = bcdToDec(Wire.read());
    *hour = bcdToDec(Wire.read() & 0x3f);
    *dayOfWeek = bcdToDec(Wire.read());
    *dayOfMonth = bcdToDec(Wire.read());
    *month = bcdToDec(Wire.read());
    *year = bcdToDec(Wire.read());
  }
  
  void sleep(unsigned long interval_1)
  {
    unsigned long starttime=millis();
    unsigned long currenttime=starttime;
   while ((currenttime - starttime) <=interval_1) // do this loop for up to 1000mS
   {  
     currenttime=millis();
   }
  //  x second sleep
    
  }
