#pragma once

#include <SoftwareSerial.h>
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

#define PI 3.1415926535897932384626433832795
#define DEG_CONVERT (180 / PI)

struct DBH
{
    float distance;
    float bearing;
    float altitude;
};

class Rangefind : public ProtobufModule<meshtastic_Position>, private concurrency::OSThread
{
protected:
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Position *p) override;

    /** Does our periodic broadcast */
    virtual int32_t runOnce() override;

public:
    Rangefind();
    void setupRangefinder();
    float getDistance();
    DBH getDBH();
    float getBearing();
    float getAltitude();
    float getHorizontal();
    // void sendPointOfInterest(NodeNum dest, bool wantReplies, uint8_t channel);

    void sendPointOfInterest();
};

extern Rangefind *rangefind;