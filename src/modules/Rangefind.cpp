#include "Rangefind.h"
#include <Adafruit_BNO08x.h>
#include "PositionModule.h"
#include "Default.h"
#include "GPS.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "TypeConversions.h"
#include "airtime.h"
#include "configuration.h"
#include "gps/GeoCoord.h"
#include "main.h"
#include "mesh/compression/unishox2.h"
#include "meshUtils.h"
#include "meshtastic/atak.pb.h"
#include "sleep.h"
#include "target_specific.h"
#include <Throttle.h>

#define RANGEFIND_BUTTON GPIO_NUM_48

char buff[4] = {0x80, 0x06, 0x03, 0x77};
unsigned char data[11] = {0};

#define BNO08X_RESET -1

struct euler_t
{
    float yaw;
    float pitch;
    float roll;
} ypr;

struct a
{
    float direct;
    float horizontal;
    float vertical;
} displacement;

// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

HardwareSerial rangefinderSerial(0);

bool failedIMU = false;
bool rangeButtonPressed = false;

void setReports()
{
    if (!bno08x.enableReport(reportType, reportIntervalUs))
    {
        failedIMU = true;
    }
}

Rangefind::Rangefind() : ProtobufModule("rangefind", meshtastic_PortNum_WAYPOINT_APP, &meshtastic_Position_msg), concurrency::OSThread("Rangefind")
{
    setupRangefinder();
}

Rangefind *rangefind;

DBH dbh;

void Rangefind::setupRangefinder()
{
    // Set up the software serial port
    failedIMU = false;
    rangefinderSerial.begin(9600);
    dbh.altitude = -1.0;
    dbh.bearing = -1.0;
    dbh.distance = -1.0;
    pinMode(RANGEFIND_BUTTON, INPUT_PULLUP);

    // Set up the BNO08x

    // Try to initialize!

    if (!bno08x.begin_I2C())
    {
        failedIMU = true;
    }
    else
    {
        setReports();
    }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
{

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees)
    {
        ypr->yaw *= RAD_TO_DEG;
        ypr->pitch *= RAD_TO_DEG;
        ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

float Rangefind::getDistance()
{
    rangefinderSerial.print(buff);
    int n = 0;
    return 0.0f;
    while (1)
    {
        n++;
        if (rangefinderSerial.available() > 0) // Determine whether there is data to read on the serial
        {
            delay(50);
            for (int i = 0; i < 11; i++)
            {
                data[i] = rangefinderSerial.read();
            }
            unsigned char Check = 0;
            for (int i = 0; i < 10; i++)
            {
                Check = Check + data[i];
            }
            Check = ~Check + 1;
            if (data[10] == Check)
            {
                if (data[3] == 'E' && data[4] == 'R' && data[5] == 'R')
                {
                    return -1.0;
                }
                else
                {
                    float distance = 0;
                    distance = (data[3] - 0x30) * 100 + (data[4] - 0x30) * 10 + (data[5] - 0x30) * 1 + (data[7] - 0x30) * 0.1 + (data[8] - 0x30) * 0.01 + (data[9] - 0x30) * 0.001;
                    return distance;
                }
            }
            else
            {

                return -1.0;
            }
        }
        delay(20);
        if (n > 100)
        {
            return -1.0;
        }
    }
}

bool getSensorEvent()
{
    if (bno08x.getSensorEvent(&sensorValue))
    {
        switch (sensorValue.sensorId)
        {
        case SH2_ARVR_STABILIZED_RV:
            quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, false);
        case SH2_GYRO_INTEGRATED_RV:
            quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, false);
            break;
        }
        return true;
    }
    else
    {
        return false;
    }
}

DBH Rangefind::getDBH()
{
    if (failedIMU)
    {
        dbh.distance = -1.0;
        dbh.bearing = -1.0;
        dbh.altitude = -1.0;
        return dbh;
    }

    bool failedToGetSensorEvent = true;

    for (int i = 0; i < 100; i++)
    {
        if (getSensorEvent())
        {
            break;
            failedToGetSensorEvent = false;
        }
        delay(10);
    }

    if (failedToGetSensorEvent)
    {
        dbh.distance = -1.0;
        dbh.bearing = -1.0;
        dbh.altitude = -1.0;
        return dbh;
    }

    float distance = getDistance();

    float pitch;
    float yaw;
    if (ypr.yaw < 0.0)
    {
        yaw = ypr.yaw + 2 * PI;
    }
    else
    {
        yaw = ypr.yaw;
    }

    if (ypr.roll < 0.0)
    {
        pitch = ypr.roll + PI;
    }
    else
    {
        pitch = ypr.roll - PI;
    }

    if (pitch > (PI / 2))
    {
        pitch = PI - pitch;
        if (yaw < PI)
        {
            yaw += PI;
        }
        else
        {
            yaw -= PI;
        }
    }
    else if (pitch < -(PI / 2))
    {
        pitch = -pitch - PI;
        if (yaw < PI)
        {
            yaw += PI;
        }
        else
        {
            yaw -= PI;
        }
    }

    dbh.distance = abs(cos(pitch) * distance);
    dbh.bearing = yaw;
    dbh.altitude = sin(pitch) * distance;

    return dbh;
}

void Rangefind::sendPointOfInterest()
{

    getDBH();
    if (dbh.distance < 0.0)
    {
        return;
    }

    meshtastic_NodeInfoLite *node = service->refreshLocalMeshNode();
    assert(node->has_position);

    uint32_t pos_flags = config.position.position_flags;

    // Populate a Position struct with ONLY the requested fields
    meshtastic_Position p = meshtastic_Position_init_default; //   Start with an empty structure

    if (localPosition.latitude_i == 0 && localPosition.longitude_i == 0)
    {
        nodeDB->setLocalPosition(TypeConversions::ConvertToPosition(node->position));
    }
    localPosition.seq_number++;

    if (localPosition.latitude_i == 0 && localPosition.longitude_i == 0)
    {
        return;
    }

    GeoCoord *current = new GeoCoord(localPosition.latitude_i, localPosition.longitude_i, localPosition.altitude);
    p.latitude_i = (current->pointAtDistance(dbh.distance, dbh.bearing))->getLatitude();
    p.longitude_i = (current->pointAtDistance(dbh.distance, dbh.bearing))->getLongitude();
    // TODO: Precision. Currently its just the perfect accuracy
    p.precision_bits = 32; // change to precision when implemented
    p.has_latitude_i = true;
    p.has_longitude_i = true;

    if (getValidTime(RTCQualityNTP) > 0)
    {
        p.time = getValidTime(RTCQualityNTP);
    }
    else if (rtc_found.address != ScanI2C::ADDRESS_NONE.address)
    {
        LOG_INFO("Use RTC time for position");
        p.time = getValidTime(RTCQualityDevice);
    }
    else if (getRTCQuality() < RTCQualityNTP)
    {
        LOG_INFO("Strip low RTCQuality (%d) time from position", getRTCQuality());
        p.time = 0;
    }

    if (config.position.fixed_position)
    {
        p.location_source = meshtastic_Position_LocSource_LOC_MANUAL;
    }
    else
    {
        p.location_source = localPosition.location_source;
    }

    if (pos_flags & meshtastic_Config_PositionConfig_PositionFlags_ALTITUDE)
    {
        if (pos_flags & meshtastic_Config_PositionConfig_PositionFlags_ALTITUDE_MSL)
        {
            p.altitude = localPosition.altitude + dbh.altitude;
            p.has_altitude = true;
        }
        else
        {
            p.altitude_hae = localPosition.altitude_hae;
            p.has_altitude_hae = true;
        }

        if (pos_flags & meshtastic_Config_PositionConfig_PositionFlags_GEOIDAL_SEPARATION)
        {
            p.altitude_geoidal_separation = localPosition.altitude_geoidal_separation + dbh.altitude;
            p.has_altitude_geoidal_separation = true;
        }
    }

    if (pos_flags & meshtastic_Config_PositionConfig_PositionFlags_SATINVIEW)
        p.sats_in_view = localPosition.sats_in_view;

    if (pos_flags & meshtastic_Config_PositionConfig_PositionFlags_TIMESTAMP)
        p.timestamp = localPosition.timestamp;

    if (pos_flags & meshtastic_Config_PositionConfig_PositionFlags_SEQ_NO)
        p.seq_number = localPosition.seq_number;

    if (pos_flags & meshtastic_Config_PositionConfig_PositionFlags_HEADING)
    {
        p.ground_track = localPosition.ground_track;
        p.has_ground_track = true;
    }

    if (pos_flags & meshtastic_Config_PositionConfig_PositionFlags_SPEED)
    {
        p.ground_speed = localPosition.ground_speed;
        p.has_ground_speed = true;
    }

    meshtastic_MeshPacket *packet = allocDataPacket();
    packet->to = NODENUM_BROADCAST;
    char *message = new char[60];
    sprintf(message, "New POI, %.7f, %.7f, %.2f\a", p.latitude_i * 1e-7, p.longitude_i * 1e-7, p.altitude);
    packet->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    packet->want_ack = false;

    service->sendToMesh(packet, RX_SRC_LOCAL, true);
    delete[] message;
}

int32_t Rangefind::runOnce()
{
    if (digitalRead(RANGEFIND_BUTTON) == LOW)
    {
        rangeButtonPressed = true;
        sendPointOfInterest();
        return 500;
    }
    else
    {
        rangeButtonPressed = false;
    }

    return 50;
}

bool Rangefind::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *pptr)
{
    return false;
}

float Rangefind::getBearing()
{
    return dbh.bearing;
}

float Rangefind::getAltitude()
{
    return dbh.altitude;
}

float Rangefind::getHorizontal()
{
    return dbh.distance;
}
