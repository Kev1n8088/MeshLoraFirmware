#include <SoftwareSerial.h>
#include "GeoCoord.h"
#include <Adafruit_BNO08x.h>

#define PI 3.1415926535897932384626433832795
#define DEG_CONVERT (180 / PI)

struct DBH
{
    float distance;
    float bearing;
    float altitude;
};

class Rangefind
{
public:
    static Rangefind *createRangefinder();
    void setupRangefinder();
    float getDistance();
    DBH getDBH();
    float getBearing();
    float getAltitude();
    float getHorizontal();
};

extern Rangefind *rangefinder;