#include "sensor.h"

static senStatus sen;

bool senClass::update() {
  while(SENSOR_PORT.available() > 0)  {
    xsens_mti_parse(&sen_interface, SENSOR_PORT.read());
  }
  if (sen.updated) {
    sen.updated = false;
    position = sen.position;
    attitude = sen.attitude;
    gps = sen.gps;
    time = sen.time;
    valid = sen.valid; 
    return true;
  }
  return false;
}

void receive( XsensEventFlag_t event, XsensEventData_t *mtdata) {
    switch(event) {
        case XSENS_EVT_QUATERNION:
            // We use the type field of XsensEventData_t as a doublecheck before
            // blindly copying bytes out of the union

            if( mtdata->type == XSENS_EVT_TYPE_FLOAT4 )
            {
                // Convert the quaternion to euler angles
                float attitude_mem[3];
                
                xsens_quaternion_to_euler( mtdata->data.f4x4, attitude_mem);

                sen.attitude.roll = attitude_mem[0];
                sen.attitude.pitch = attitude_mem[1];
                sen.attitude.yaw = attitude_mem[2];

                // Convert from radians to degrees
                sen.attitude.roll   *= (180.0 / PI);
                sen.attitude.pitch  *= (180.0 / PI);
                sen.attitude.yaw    *= (180.0 / PI);

                sen.attitude.roll = sen.attitude.roll;
                sen.attitude.pitch = sen.attitude.pitch;
                sen.attitude.yaw = sen.attitude.yaw;

                sen.updated = true;

            }
        break;

        case XSENS_EVT_EULER:
                sen.attitude.roll  = mtdata->data.f4x3[0];
                sen.attitude.pitch = mtdata->data.f4x3[1];
                sen.attitude.yaw   = -mtdata->data.f4x3[2];

                sen.updated = true;
        break;

        case XSENS_EVT_LAT_LON:
            if( mtdata->type == XSENS_EVT_TYPE_FLOAT2 )
            {
                sen.position.lat  = mtdata->data.f4x2[0];
                sen.position.lng = mtdata->data.f4x2[1];
                if (sen.position.lng<0) {
                    sen.position.lng = 360.0 + sen.position.lng;
                }

                if (sen.position.lat != 0 && sen.position.lng != 0 && sen.gps.fixType != 0) {
                    sen.gps.isValid = true;
                }
                else {
                    sen.gps.isValid = false;
                }
            }
        break;

        case XSENS_EVT_ALTITUDE_ELLIPSOID:
            if( mtdata->type == XSENS_EVT_TYPE_FLOAT )
            {
                sen.position.alt = mtdata->data.f4;
            }
        break;

        case XSENS_EVT_UTC_TIME:        
            if( mtdata->type == XSENS_EVT_TYPE_TIME )
            {
                sen.time.nanosecond = mtdata->data.u4;
                sen.time.year = mtdata->data.u2;
                sen.time.month = mtdata->data.u1x6[0];
                sen.time.day = mtdata->data.u1x6[1];
                sen.time.hour = mtdata->data.u1x6[2];
                sen.time.minute = mtdata->data.u1x6[3];
                sen.time.second = mtdata->data.u1x6[4];

                sen.time.code.second = (sen.time.hour*3600)+(sen.time.minute*60)+(sen.time.second);
                sen.time.code.nanosecond = sen.time.nanosecond;
            }
        break;

        case XSENS_EVT_GNSS_PVT_DATA:
            if( mtdata->type == XSENS_EVT_TYPE_GNSS )
            {
                sen.time.isValid = bitRead(mtdata->data.u1x6[0],2);
                sen.gps.fixType = bitRead(mtdata->data.u1x6[1],0);
                sen.gps.satNumber = mtdata->data.u1x6[2];
                sen.gps.vAcc = mtdata->data.u4;
            }
        break;

        case XSENS_EVT_STATUS_WORD:
          if( mtdata->type == XSENS_EVT_TYPE_U32 )
          {
            //union XDI_STATUS32_UNION status;
            //status.word = mtdata->data.u4;
          }
        break;

        default:
        break;
    }
}

// The library calls this function to send packets to the IMU
void send( uint8_t *data, uint16_t length ){
    SENSOR_PORT.write( data, length );
    if (DEBUG) {
        for (int i=0; i<length; i++) {
            USBSerial.print(data[i], HEX);
            USBSerial.print(" ");
        }
        USBSerial.println("");
    }
}

senClass::senClass() 
{
    time.isValid = false;
    gps.isValid = false;
}

void senClass::begin() {
    USBSerial.begin(115200);
    // ------------- PORT --------- DeviceRX, DeviceTX // 
    SENSOR_PORT.begin(SENSOR_BAUD, 134217756U, 39, 38);
    sen_interface = XSENS_INTERFACE_RX_TX( &receive, &send );
    config();
    //setNoRotation(10);
}

void senClass::setNoRotation(int16_t timeForNoRotation) {
    xsens_mti_set_no_rotation( &sen_interface, timeForNoRotation );
}

void senClass::calibrate() {
    //xsens_mti_reset_orientation( &sen_interface, XSENS_ORIENTATION_HEADING_RESET);
    //xsens_mti_reset_orientation( &sen_interface, XSENS_ORIENTATION_INCLINATION_RESET);
    xsens_mti_reset_orientation( &sen_interface, XSENS_ORIENTATION_ALIGNMENT_RESET);
    xsens_mti_request( &sen_interface, MT_GOTOCONFIG );
    xsens_mti_reset_orientation( &sen_interface, XSENS_ORIENTATION_STORE);
    xsens_mti_request( &sen_interface, MT_GOTOMEASUREMENT );
}

void senClass::config() {

    xsens_mti_reset_orientation( &sen_interface, XSENS_ORIENTATION_HEADING_DEFAULT);
    xsens_mti_reset_orientation( &sen_interface, XSENS_ORIENTATION_INCLINATION_DEFAULT);
    xsens_mti_reset_orientation( &sen_interface, XSENS_ORIENTATION_ALIGNMENT_DEFAULT);

    uint16_t filterProfile = 13;
    // enum XDA_TYPE_IDENTIFIER outputList[]= {XDI_LAT_LON, 
    //                                         XDI_GNSS_PVT_DATA, 
    //                                         XDI_EULER_ANGLES, 
    //                                         XDI_UTC_TIME, 
    //                                         XDI_ALTITUDE_ELLIPSOID, 
    //                                         XDI_STATUS_WORD
    //         };

    enum XDA_TYPE_IDENTIFIER outputList[]= { XDI_EULER_ANGLES, 
                                    XDI_UTC_TIME,  
                                    XDI_STATUS_WORD
    };

    uint16_t dynamicProfile = 8;
    size_t outputListSize = sizeof(outputList)/sizeof(enum XDA_TYPE_IDENTIFIER);

    xsens_mti_request( &sen_interface, MT_GOTOCONFIG );
    if (DEBUG) { printReceived(); }

    //xsens_mti_reset_factory( &sen_interface );
    //if (DEBUG) { printReceived(); }

    xsens_mti_set_filter_profile( &sen_interface, filterProfile );
    if (DEBUG) { printReceived(); }

    //xsens_mti_set_orientation( &sen_interface,  0.7071, 0, 0.7071, 0);
    //xsens_mti_set_orientation( &sen_interface,  0, 0, 1, 0);
    //if (DEBUG) { printReceived(); }

    xsens_mti_reset_orientation( &sen_interface, XSENS_ORIENTATION_STORE);
    if (DEBUG) { printReceived(); }

    //USBSerial.print("Requesting the current platform... ");
    //xsens_mti_request( &sen_interface,  MT_REQGNSSPLATFORM );
    //USBSerial.print("Sensor says it's... ");
    //if (DEBUG) { printReceived(); }

    //USBSerial.print("Setting the new platform... ");
    //xsens_mti_set_platform( &sen_interface, dynamicProfile);
    //USBSerial.print("Sensor respond... ");
    if (DEBUG) { printReceived(); }

    // Setting for the sensor: fill the outputList with desired data, the last parameter is the rate:
    // Available rate, HZA = A Hz rate. 1, 5, 10, 20, 50, 100Hz are available. Defined in xsens_constant.h
    xsens_mti_set_output( &sen_interface, outputList, outputListSize, HZ100);
    if (DEBUG) { printReceived(); }

    // Enables in run compass calibration
    xsens_mti_set_option_flag( &sen_interface, 0x10);
    xsens_mti_set_option_flag( &sen_interface, 0x80);

    //xsens_mti_clear_option_flag( &sen_interface, 0x80);
    //xsens_mti_clear_option_flag( &sen_interface, 0x10);
    if (DEBUG) { printReceived(); }
    xsens_mti_request( &sen_interface, MT_GOTOMEASUREMENT );
}

void senClass::printReceived() {
    delay(1000);
    while(SENSOR_PORT.available() > 0)  {
        USBSerial.print(SENSOR_PORT.read(), HEX); USBSerial.print(" ");
    }
    USBSerial.println("");
}

bool senClass::isValid() {
    if (sen.time.year !=1970 && sen.position.lat !=0 && sen.position.lng !=0) {
        return (gps.isValid && time.isValid);
    }
    else {
        return false;
    }
}

senStatus senClass::get() {
    senStatus senOut;
    senOut.position = position;
    senOut.attitude = attitude;
    senOut.time = time;
    senOut.valid = isValid();
    senOut.gps = gps;
    return senOut;
}


double senClass::timeDiff(timeCode time1, timeCode time2) {
    double timeDiff = (time1.second - time2.second) + ((time1.nanosecond - time2.nanosecond)/1000000000.0);
    return timeDiff;
}

double timeDiff(timeCode time1, timeCode time2) {
    double timeDiff = (time1.second - time2.second) + ((time1.nanosecond - time2.nanosecond)/1000000000.0);
    return timeDiff;
}
