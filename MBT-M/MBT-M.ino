#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <string.h>
#include <stdio.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFunMPU9250-DMP.h>

MPU9250_DMP imu;
double roll, pitch, yaw;
long int pre_ts = 0;

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

TinyGPSPlus gps;
SoftwareSerial ss(13, 12);
Adafruit_BME280 bme;


File myFile;

bool fake_mode = false;
bool debug_mode = false;
unsigned long int fire_time = 0;
float millis_time;
float cur_time;
int pinCS = 53;
char tmp_str[20];
char ch_message[150];
char file_name[] = "askim_deryam.txt";
bool file_status;
float base_pres = 0;
int counter = 0;
short int deployment_status = 0;

typedef struct
{
    float bme_alt, bme_pres, bme_temp;
    double gps_alt, gps_lat, gps_lon;
    float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
    float rc_ang, rc_rot, rc_yaw, rc_speed;
    unsigned long int rc_time;
}ROCKET_STAT;

ROCKET_STAT Astra,Astra_prev;


void setup() {
    Serial.begin(9600);
    while (!Serial);
    bme.begin();
    ss.begin(9600);
    if (imu.begin() != INV_SUCCESS)
    {
        while (1) {
            Serial.println("Check connections, and try again.\n");
            delay(3000);
        }
    }
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    imu.setGyroFSR(250); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(8); // Set accel to +/-2g
    imu.setLPF(10); // Set LPF corner frequency to 5Hz
    imu.setSampleRate(10); // Set sample rate to 10Hz
    imu.setCompassSampleRate(50); // Set mag rate to 10Hz
    pre_ts = millis();

    pinMode(pinCS, OUTPUT);
    if (SD.begin()) {
        Serial.println("File system is running");
        file_status = true;
    }
    else
    {
        Serial.println("File system can not be activated");
        file_status = false;
    }
    unsigned status;
    
    Serial1.begin(9600);
    //base_pres = 1000;
    Serial.println("calibrating");
    for (cur_time = millis(); millis() - cur_time < 1000; base_pres += bme.readPressure(), counter++) {}
    base_pres /= (counter * 100.0F);
    if (debug_mode){
      float accx = 0, accy = 0, accz = 0;
      Serial.println("Starting in 3");
      delay(3000);
      for (cur_time = millis(); millis() - cur_time < 5000; add_mpu_values(), counter++) {
          accx += Astra.acc_x + 0.10;
          accy += Astra.acc_y + 0.025;
          accz += Astra.acc_z;
          if (counter > 1)
            accz /= 2;
      };
      accx /= counter;
      accy /= counter;
  
      Serial.println(accx);
      Serial.println(accy);
      Serial.println(accz);
      delay(1000);
    }
    if (fake_mode){}
    
}

char sz[32];

void loop() {
  char cheksum = 0;
    if (file_status)
      start_writing_file();
    add_hash();
    add_gps_values();
    add_bme_values();
    add_mpu_values();
    broadcast_message();
    
    for (int i = 0; i < strlen(ch_message); i++){
      cheksum += ch_message[i];
      }
      ch_message[strlen(ch_message)] = cheksum;
    /*
    if (fire_time == 0 && Astra.acc_z > 2) {
        fire_time = millis();
    }

    if (millis() - fire_time < 10000) {
        Astra.rc_speed += (Astra.acc_z - 1);
    }
    else {
        Astra.rc_speed = (Astra.bme_alt - Astra_prev.bme_alt) / ((Astra.rc_time - Astra_prev.rc_time) * 1000.0F);
    }
    if (debug_mode){
      if (Astra.bme_alt > 100) {
        Serial.print("-Yükseklik ulaşıldı-");
          if (( Astra.rc_yaw < 30 || Astra.rc_speed < -10) && deployment_status == 0) {
              deployment_status = 1;
          }
      }
      if (deployment_status == 1 && Astra.bme_alt < -100) {
          deployment_status = 3;
      }
    }
    */
    Astra_prev = Astra;
    delay(500);

}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void start_writing_file() {
    if (file_status) {
        myFile = SD.open(file_name, FILE_WRITE);
        if (!myFile) {
            Serial.println("File cannot be opened");
            return;
        }
    }
}
void close_file() {
    myFile.close();
}

void broadcast_message() {/*
    char tmp_str[] = "\n";
    strcat(ch_message, tmp_str);
    ch_message[strlen(ch_message) - 1] = '\0';*/
    millis_time = millis();
    cur_time = millis_time / 1000;
    //save_float(cur_time);
    ch_message[strlen(ch_message) - 1] = '\0';
    Serial.println(ch_message);
    Serial1.println(ch_message);
    
    if (debug_mode){
      switch (deployment_status){
      case 0:Serial.println("-Açılmadı-"); break;
      case 1:Serial.println("-Yavaşlatma paraşütü açık-"); break;
      case 2:Serial.println("-Ana paraşüt açık-"); break;
      case 3:Serial.println("-Bütün paraşütler açık-"); break;
      default:Serial.println("-Sistem arızası-"); break;
      }
    }
    if (myFile){
      save_in_file(ch_message);
      close_file();
    }
    smartDelay(0);
}

void read_file() {
    myFile = SD.open(file_name);
    if (myFile) {
        while (myFile.available()) {
            Serial.write(myFile.read());
        }
        close_file();
    }
    else {
        Serial.println("File can't be accessed");
        return;
    }
}

void save_in_file(char* save_msg) {
    if (myFile)
        myFile.print(save_msg);
    else
        return;
}

void save_float(float input) {
    char tmp_str[10];
    dtostrf(input, 1, 2, tmp_str);
    strcat(ch_message, tmp_str);
    strcat(ch_message, ",");
    free(tmp_str);
    smartDelay(0);
}

void save_gps(float input) {
    char tmp_str[10];
    dtostrf(input, 1, 6, tmp_str);    
    strcat(ch_message, tmp_str);
    strcat(ch_message, ",");
    free(tmp_str);
}

void add_hash() {
    Astra.rc_time = millis();
    //strcpy(ch_message,"$ATNTPA;");
    strcpy(ch_message, "$AT;");
    
    
    }

void add_gps_values() {
    Astra.gps_lat = (gps.location.lat());
    Astra.gps_lon = (gps.location.lng());
    Astra.gps_alt = gps.altitude.meters();
    if (fake_mode){
      save_gps(random(40992808, 40993063)/1000000.0f);
      save_gps(random(28840095, 28842520)/1000000.0f);
      save_float(random(5500, 8500)/100.0f);
    }
    else{
      if (Astra.gps_lat != 0.0f){
      save_gps(Astra.gps_lat);
      save_gps(Astra.gps_lon);
      save_float(Astra.gps_alt);
      }
    }
    Serial.println("son test");
}

void add_bme_values() {
    Astra.bme_alt = bme.readAltitude(base_pres);
    Astra.bme_pres = (bme.readPressure() / 100.0F);
    Astra.bme_temp = bme.readTemperature();
    if (fake_mode){
      save_float(random(-500, 500)/100.0f);
      save_float(random(90000, 11000)/100.0f);
      save_float(random(1000, 1500)/100.0f);
    }
    else{    
      save_float(Astra.bme_alt);
      save_float(Astra.bme_pres);
      save_float(Astra.bme_temp);    
    }
}

void add_mpu_values() {
    if (imu.dataReady())
    {
        imu.update(UPDATE_ACCEL | UPDATE_GYRO);
        Astra.acc_x = imu.calcAccel(imu.ax);
        Astra.acc_y = imu.calcAccel(imu.ay);
        Astra.acc_z = imu.calcAccel(imu.az);
        Astra.gyro_x = (imu.calcGyro(imu.gx) / 57.3);
        Astra.gyro_y = (imu.calcGyro(imu.gy) / 57.3);
        Astra.gyro_z = (imu.calcGyro(imu.gz) / 57.3);

        // Offsets
        Astra.acc_x -= 0.03;
        Astra.acc_y -= 0.005;
        Astra.acc_z += 0.05;

        Astra.rc_ang = atan2(Astra.acc_y, (sqrt((Astra.acc_x * Astra.acc_x) + (Astra.acc_z * Astra.acc_z))));
        Astra.rc_rot = atan2(-Astra.acc_x, (sqrt((Astra.acc_y * Astra.acc_y) + (Astra.acc_z * Astra.acc_z))));
        Astra.rc_yaw = atan2(-Astra.acc_z, (sqrt((Astra.acc_y * Astra.acc_y) + (Astra.acc_x * Astra.acc_x))));
        Astra.rc_rot = Astra.rc_rot * 57.3;
        Astra.rc_ang = Astra.rc_ang * 57.3;
        Astra.rc_yaw = (Astra.rc_yaw * 57.3);

        if (Astra.rc_ang < 0) {
            Astra.rc_ang = -Astra.rc_ang;
        }
        if (Astra.rc_rot < 0) {
            Astra.rc_rot = -Astra.rc_rot;
        }
        if (Astra.rc_yaw < 0) {
            Astra.rc_yaw = -Astra.rc_yaw;
        }
        if (fake_mode){
          
        }
        else{        
        save_float(Astra.acc_x);
        save_float(Astra.acc_y);
        save_float(Astra.acc_z);
        save_float(Astra.gyro_x);
        save_float(Astra.gyro_y);
        save_float(Astra.gyro_z);
        save_float(Astra.rc_ang);
        save_float(Astra.rc_rot);
        save_float(90.0f - Astra.rc_yaw);
          }
    }
}
