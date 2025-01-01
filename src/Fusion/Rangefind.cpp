#include "Rangefind.h"

char buff[4] = {0x80, 0x06, 0x03, 0x77};
unsigned char data[11] = {0};

#define RANGEFINDER_RX 2
#define RANGEFINDER_TX 3

#define BNO08X_RESET -1

Rangefind *rangefinder = nullptr;

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

SoftwareSerial rangefinderSerial(RANGEFINDER_RX, RANGEFINDER_TX);

bool failedIMU = false;

void setReports()
{
    if (!bno08x.enableReport(reportType, reportIntervalUs))
    {
        failedIMU = true;
    }
}

Rangefind *Rangefind::createRangefinder()
{
    Rangefind *rangefind = new Rangefind();
    rangefind->setupRangefinder();
    return rangefind;
}

DBH dbh;

void Rangefind::setupRangefinder()
{
    // Set up the software serial port
    failedIMU = false;
    rangefinderSerial.begin(9600);
    dbh.altitude = -1.0;
    dbh.bearing = -1.0;
    dbh.distance = -1.0;

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
