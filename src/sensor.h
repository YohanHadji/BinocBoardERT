#ifndef SENSOR
#define SENSOR

#include "mtiSensor/xsens_mti.h"      // main library
#include "mtiSensor/xsens_utility.h"  // needed for quaternion conversion function
#include "Arduino.h"
#include "config.h"

double getAngle(double angle1, double angle2);

struct timeCode {
  int second;
  int nanosecond;
};

struct vect3 {
  double a;
  double b;
  double c;
};

// GPS Coordinate, Latitude (° decimal), Longitude (° decimal), and Altitude (m)
struct gpsCoord {
  double lat;
  double lng;
  double alt;
};

// XYZ Coordinate
struct xyzCoord {
  double x;
  double y;
  double z;
};

// Roll Pitch Yaw Coordinate
struct rpyCoord {
  double roll;
  double pitch;
  double yaw;
};

struct gpsStatus {
  unsigned fixType;
  unsigned int hdop;
  unsigned int satNumber;
  unsigned int vAcc;
  bool isValid;
};

struct timeStruct {
  unsigned int year;
  unsigned int month;
  unsigned int day;
  unsigned int hour;
  unsigned int minute;
  unsigned int second;
  unsigned int nanosecond;
  timeCode code;
  bool isValid;
};

struct senStatus {
  timeStruct time;
  gpsCoord position;
  rpyCoord attitude;
  gpsStatus gps;
  bool valid;
  bool updated;
  bool runningNoRotation;
};

double timeDiff(timeCode time1, timeCode time2);

//void receive(XsensEventFlag_t event, XsensEventData_t *mtdata);
//void send(uint8_t *data, uint16_t length);

class senClass {
  public:
    senClass();
    bool update();
    senStatus get();
    bool isValid();
    void begin();
    void calibrate();
    void setNoRotation(int16_t timeForNoRotation);
    //void receive(XsensEventFlag_t event, XsensEventData_t *mtdata);
    //void send(uint8_t *data, uint16_t length);
    friend void receive(XsensEventFlag_t event, XsensEventData_t *mtdata);
    friend void send(uint8_t *data, uint16_t length);
    double timeDiff(timeCode time1, timeCode time2);
  private:
    void config();
    void printReceived();
    timeStruct time;
    gpsCoord position;
    rpyCoord attitude;
    gpsStatus gps;
    xsens_interface_t sen_interface;
    bool updated;
    bool valid;
    bool runningNoRotation;
};


#endif 
