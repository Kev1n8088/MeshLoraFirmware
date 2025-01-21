#include "SerialModule.h"
#include "GeoCoord.h"
#include "MeshService.h"
#include "NMEAWPL.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "configuration.h"
#include <Arduino.h>
#include <Throttle.h>
#include "telemetry/MSP.h"

/*
    SerialModule
        A simple interface to send messages over the mesh network by sending strings
        over a serial port.

        There are no PIN defaults, you have to enable the second serial port yourself.

    Need help with this module? Post your question on the Meshtastic Discourse:
       https://meshtastic.discourse.group

    Basic Usage:

        1) Enable the module by setting enabled to 1.
        2) Set the pins (rxd / rxd) for your preferred RX and TX GPIO pins.
           On tbeam, recommend to use:
                RXD 35
                TXD 15
        3) Set timeout to the amount of time to wait before we consider
           your packet as "done".
        4) not applicable any more
        5) Connect to your device over the serial interface at 38400 8N1.
        6) Send a packet up to 240 bytes in length. This will get relayed over the mesh network.
        7) (Optional) Set echo to 1 and any message you send out will be echoed back
           to your device.

    TODO (in this order):
        * Define a verbose RX mode to report on mesh and packet information.
            - This won't happen any time soon.

    KNOWN PROBLEMS
        * Until the module is initialized by the startup sequence, the TX pin is in a floating
          state. Device connected to that pin may see this as "noise".
        * Will not work on Linux device targets.


*/

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52) || defined(ARCH_RP2040)) && !defined(CONFIG_IDF_TARGET_ESP32S2) && \
    !defined(CONFIG_IDF_TARGET_ESP32C3)

#define RX_BUFFER 256
#define TIMEOUT 250
#define BAUD 38400
#define ACK 1

// API: Defaulting to the formerly removed phone_timeout_secs value of 15 minutes
#define SERIAL_CONNECTION_TIMEOUT (15 * 60) * 1000UL

SerialModule *serialModule;
SerialModuleRadio *serialModuleRadio;

#if defined(TTGO_T_ECHO) || defined(CANARYONE)
SerialModule::SerialModule() : StreamAPI(&Serial), concurrency::OSThread("Serial") {}
static Print *serialPrint = &Serial;
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
SerialModule::SerialModule() : StreamAPI(&Serial1), concurrency::OSThread("Serial") {}
static Print *serialPrint = &Serial1;
#else
SerialModule::SerialModule() : StreamAPI(&Serial2), concurrency::OSThread("Serial") {}
static Print *serialPrint = &Serial2;
#endif

char serialBytes[512];
size_t serialPayloadSize;

SerialModuleRadio::SerialModuleRadio() : MeshModule("SerialModuleRadio")
{
    switch (moduleConfig.serial.mode)
    {
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_TEXTMSG:
        ourPortNum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        break;
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MSP:
        ourPortNum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        break;
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA:
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO:
        ourPortNum = meshtastic_PortNum_POSITION_APP;
        break;
    default:
        ourPortNum = meshtastic_PortNum_SERIAL_APP;
        // restrict to the serial channel for rx
        boundChannel = Channels::serialChannel;
        break;
    }
}

/**
 * @brief Checks if the serial connection is established.
 *
 * @return true if the serial connection is established, false otherwise.
 *
 * For the serial2 port we can't really detect if any client is on the other side, so instead just look for recent messages
 */
bool SerialModule::checkIsConnected()
{
    return Throttle::isWithinTimespanMs(lastContactMsec, SERIAL_CONNECTION_TIMEOUT);
}

int32_t SerialModule::runOnce()
{
    /*
        Uncomment the preferences below if you want to use the module
        without having to configure it from the PythonAPI or WebUI.
    */

    // moduleConfig.serial.enabled = true;
    // moduleConfig.serial.rxd = 35;
    // moduleConfig.serial.txd = 15;
    // moduleConfig.serial.override_console_serial_port = true;
    // moduleConfig.serial.mode = meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO;
    // moduleConfig.serial.timeout = 1000;
    // moduleConfig.serial.echo = 1;

    MSP msp;

    if (!moduleConfig.serial.enabled)
        return disable();

    if (moduleConfig.serial.override_console_serial_port || (moduleConfig.serial.rxd && moduleConfig.serial.txd))
    {
        if (firstTime)
        {
            // Interface with the serial peripheral from in here.
            LOG_INFO("Init serial peripheral interface");

            uint32_t baud = getBaudRate();

            if (moduleConfig.serial.override_console_serial_port)
            {
#ifdef RP2040_SLOW_CLOCK
                Serial2.flush();
                serialPrint = &Serial2;
#else
                Serial.flush();
                serialPrint = &Serial;
#endif
                // Give it a chance to flush out 💩
                delay(10);
            }
#if defined(CONFIG_IDF_TARGET_ESP32C6)
            if (moduleConfig.serial.rxd && moduleConfig.serial.txd)
            {
                Serial1.setRxBufferSize(RX_BUFFER);
                Serial1.begin(baud, SERIAL_8N1, moduleConfig.serial.rxd, moduleConfig.serial.txd);
            }
            else
            {
                Serial.begin(baud);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            }

#elif defined(ARCH_ESP32)

            if (moduleConfig.serial.rxd && moduleConfig.serial.txd)
            {
                Serial2.setRxBufferSize(RX_BUFFER);
                Serial2.begin(baud, SERIAL_8N1, moduleConfig.serial.rxd, moduleConfig.serial.txd);
            }
            else
            {
                Serial.begin(baud);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            }
#elif !defined(TTGO_T_ECHO) && !defined(CANARYONE)
            if (moduleConfig.serial.rxd && moduleConfig.serial.txd)
            {
#ifdef ARCH_RP2040
                Serial2.setFIFOSize(RX_BUFFER);
                Serial2.setPinout(moduleConfig.serial.txd, moduleConfig.serial.rxd);
#else
                Serial2.setPins(moduleConfig.serial.rxd, moduleConfig.serial.txd);
#endif
                Serial2.begin(baud, SERIAL_8N1);
                Serial2.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            }
            else
            {
#ifdef RP2040_SLOW_CLOCK
                Serial2.begin(baud, SERIAL_8N1);
                Serial2.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#else
                Serial.begin(baud, SERIAL_8N1);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#endif
            }
#else
            Serial.begin(baud, SERIAL_8N1);
            Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#endif
            serialModuleRadio = new SerialModuleRadio();

            firstTime = 0;

            // in API mode send rebooted sequence
            if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO)
            {
                emitRebooted();
            }
            else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MSP) && HAS_GPS)
            {
                msp.begin(Serial2);
            }
        }
        else
        {
            if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO)
            {
                return runOncePart();
            }
            else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA) && HAS_GPS)
            {
                // in NMEA mode send out GGA every 2 seconds, Don't read from Port
                if (!Throttle::isWithinTimespanMs(lastNmeaTime, 2000))
                {
                    lastNmeaTime = millis();
                    printGGA(outbuf, sizeof(outbuf), localPosition);
                    serialPrint->printf("%s", outbuf);
                }
            }
            else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO) && HAS_GPS)
            {
                if (!Throttle::isWithinTimespanMs(lastNmeaTime, 10000))
                {
                    lastNmeaTime = millis();
                    uint32_t readIndex = 0;
                    const meshtastic_NodeInfoLite *tempNodeInfo = nodeDB->readNextMeshNode(readIndex);
                    while (tempNodeInfo != NULL)
                    {
                        if (tempNodeInfo->has_user && nodeDB->hasValidPosition(tempNodeInfo))
                        {
                            printWPL(outbuf, sizeof(outbuf), tempNodeInfo->position, tempNodeInfo->user.long_name, true);
                            serialPrint->printf("%s", outbuf);
                        }
                        tempNodeInfo = nodeDB->readNextMeshNode(readIndex);
                    }
                }
            }
            else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MSP) && HAS_GPS)
            {
                if (!Throttle::isWithinTimespanMs(lastNmeaTime, 5000))
                {
                    msp_altitude_t alt;
                    int32_t data[14] = {-1}; // see below
                    char headers[14][4] = {"ALT", "SPD", "PCH", "ROL", "HDG", "LON", "LAT", "NST", "CRS", "DST", "DIR", "WPN", "WPM", "WPA"};
                    // int8_t dataLengths[14] = {5, 4, 4, 4, 4, 11, 11, 2, 2, 5, 3, 3, 1, 2};
                    if (msp.request(MSP_ALTITUDE, &alt, sizeof(alt)))
                    {
                        data[0] = alt.estimatedActualPosition; // cm
                    }
                    delay(20);
                    msp_attitude_t att;
                    if (msp.request(MSP_ATTITUDE, &att, sizeof(att)))
                    {
                        data[2] = att.pitch;
                        data[3] = att.roll;
                        data[4] = att.yaw;
                    }
                    delay(20);
                    msp_raw_gps_t raw_gps;
                    if (msp.request(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps)))
                    {
                        data[1] = raw_gps.groundSpeed;
                        data[5] = raw_gps.lon;
                        data[6] = raw_gps.lat;
                        data[7] = raw_gps.numSat;
                        data[7] = raw_gps.groundCourse;
                    }
                    msp_comp_gps_t comp_gps;
                    if (msp.request(MSP_GPSSTATISTICS, &comp_gps, sizeof(comp_gps)))
                    {
                        data[9] = comp_gps.distanceToHome;
                        data[10] = comp_gps.directionToHome;
                    }
                    delay(20);
                    msp_nav_status_t nav_status;
                    if (msp.request(MSP_NAV_STATUS, &nav_status, sizeof(nav_status)))
                    {
                        data[11] = nav_status.activeWpNumber;
                        data[12] = nav_status.mode;
                        data[13] = nav_status.activeWpAction;
                    }

                    meshtastic_MeshPacket *p = router->allocForSending();
                    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
                    char *message = new char[200];
                    sprintf(message, "MSP:", headers[0], data[0], headers[1], data[1], headers[2], data[2], headers[3], data[3], headers[4], data[4], headers[5], data[5], headers[6], data[6], headers[7], data[7], headers[8], data[8], headers[9], data[9], headers[10], data[10], headers[11], data[11], headers[12], data[12], headers[13], data[13]);
                    p->decoded.payload.size = strlen(message);
                    p->to = NODENUM_BROADCAST;
                    p->decoded.want_response = false;
                    p->want_ack = false;
                    memcpy(p->decoded.payload.bytes, message, p->decoded.payload.size);
                    service->sendToMesh(p);
                }
            }

#if !defined(TTGO_T_ECHO) && !defined(CANARYONE)
            else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_WS85))
            {
                processWXSerial();
            }
            else
            {
#if defined(CONFIG_IDF_TARGET_ESP32C6)
                while (Serial1.available())
                {
                    serialPayloadSize = Serial1.readBytes(serialBytes, meshtastic_Constants_DATA_PAYLOAD_LEN);
#else
                while (Serial2.available())
                {
                    serialPayloadSize = Serial2.readBytes(serialBytes, meshtastic_Constants_DATA_PAYLOAD_LEN);
#endif
                    serialModuleRadio->sendPayload();
                }
            }
#endif
        }
        return (10);
    }
    else
    {
        return disable();
    }
}

/**
 * Sends telemetry packet over the mesh network.
 *
 * @param m The telemetry data to be sent
 *
 * @return void
 *
 * @throws None
 */
void SerialModule::sendTelemetry(meshtastic_Telemetry m)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    p->decoded.payload.size =
        pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Telemetry_msg, &m);
    p->to = NODENUM_BROADCAST;
    p->decoded.want_response = false;
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR)
    {
        p->want_ack = true;
        p->priority = meshtastic_MeshPacket_Priority_HIGH;
    }
    else
    {
        p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    }
    service->sendToMesh(p, RX_SRC_LOCAL, true);
}

/**
 * Allocates a new mesh packet for use as a reply to a received packet.
 *
 * @return A pointer to the newly allocated mesh packet.
 */
meshtastic_MeshPacket *SerialModuleRadio::allocReply()
{
    auto reply = allocDataPacket(); // Allocate a packet for sending

    return reply;
}

/**
 * Sends a payload to a specified destination node.
 *
 * @param dest The destination node number.
 * @param wantReplies Whether or not to request replies from the destination node.
 */
void SerialModuleRadio::sendPayload(NodeNum dest, bool wantReplies)
{
    const meshtastic_Channel *ch = (boundChannel != NULL) ? &channels.getByName(boundChannel) : NULL;
    meshtastic_MeshPacket *p = allocReply();
    p->to = dest;
    if (ch != NULL)
    {
        p->channel = ch->index;
    }
    p->decoded.want_response = wantReplies;

    p->want_ack = ACK;

    p->decoded.payload.size = serialPayloadSize; // You must specify how many bytes are in the reply
    memcpy(p->decoded.payload.bytes, serialBytes, p->decoded.payload.size);

    service->sendToMesh(p);
}

/**
 * Handle a received mesh packet.
 *
 * @param mp The received mesh packet.
 * @return The processed message.
 */
ProcessMessage SerialModuleRadio::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (moduleConfig.serial.enabled)
    {
        if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO)
        {
            // in API mode we don't care about stuff from radio.
            return ProcessMessage::CONTINUE;
        }

        auto &p = mp.decoded;
        // LOG_DEBUG("Received text msg self=0x%0x, from=0x%0x, to=0x%0x, id=%d, msg=%.*s",
        //          nodeDB->getNodeNum(), mp.from, mp.to, mp.id, p.payload.size, p.payload.bytes);

        if (isFromUs(&mp))
        {

            /*
             * If moduleConfig.serial.echo is true, then echo the packets that are sent out
             * back to the TX of the serial interface.
             */
            if (moduleConfig.serial.echo)
            {

                // For some reason, we get the packet back twice when we send out of the radio.
                //   TODO: need to find out why.
                if (lastRxID != mp.id)
                {
                    lastRxID = mp.id;
                    // LOG_DEBUG("* * Message came this device");
                    // serialPrint->println("* * Message came this device");
                    serialPrint->printf("%s", p.payload.bytes);
                }
            }
        }
        else
        {

            if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_DEFAULT ||
                moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_SIMPLE)
            {
                serialPrint->write(p.payload.bytes, p.payload.size);
            }
            else if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_TEXTMSG)
            {
                meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(getFrom(&mp));
                String sender = (node && node->has_user) ? node->user.short_name : "???";
                serialPrint->println();
                serialPrint->printf("%s: %s", sender, p.payload.bytes);
                serialPrint->println();
            }
            else if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MSP)
            {
                serialPrint->printf("", p.payload.bytes); // directly sends MSP commands, encoded in transmission
            }
            else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA ||
                      moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO) &&
                     HAS_GPS)
            {
                // Decode the Payload some more
                meshtastic_Position scratch;
                meshtastic_Position *decoded = NULL;
                if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag && mp.decoded.portnum == ourPortNum)
                {
                    memset(&scratch, 0, sizeof(scratch));
                    if (pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Position_msg, &scratch))
                    {
                        decoded = &scratch;
                    }
                    // send position packet as WPL to the serial port
                    printWPL(outbuf, sizeof(outbuf), *decoded, nodeDB->getMeshNode(getFrom(&mp))->user.long_name,
                             moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO);
                    serialPrint->printf("%s", outbuf);
                }
            }
            else if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MSP && HAS_GPS)
            {
            }
        }
    }
    return ProcessMessage::CONTINUE; // Let others look at this message also if they want
}

/**
 * @brief Returns the baud rate of the serial module from the module configuration.
 *
 * @return uint32_t The baud rate of the serial module.
 */
uint32_t SerialModule::getBaudRate()
{
    if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_110)
    {
        return 110;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_300)
    {
        return 300;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_600)
    {
        return 600;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_1200)
    {
        return 1200;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_2400)
    {
        return 2400;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_4800)
    {
        return 4800;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_9600)
    {
        return 9600;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_19200)
    {
        return 19200;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_38400)
    {
        return 38400;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_57600)
    {
        return 57600;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_115200)
    {
        return 115200;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_230400)
    {
        return 230400;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_460800)
    {
        return 460800;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_576000)
    {
        return 576000;
    }
    else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_921600)
    {
        return 921600;
    }
    return BAUD;
}

/**
 * Process the received weather station serial data, extract wind, voltage, and temperature information,
 * calculate averages and send telemetry data over the mesh network.
 *
 * @return void
 */
void SerialModule::processWXSerial()
{
#if !defined(TTGO_T_ECHO) && !defined(CANARYONE) && !defined(CONFIG_IDF_TARGET_ESP32C6)
    static unsigned int lastAveraged = 0;
    static unsigned int averageIntervalMillis = 300000; // 5 minutes hard coded.
    static double dir_sum_sin = 0;
    static double dir_sum_cos = 0;
    static float velSum = 0;
    static float gust = 0;
    static float lull = -1;
    static int velCount = 0;
    static int dirCount = 0;
    static char windDir[4] = "xxx";   // Assuming windDir is 3 characters long + null terminator
    static char windVel[5] = "xx.x";  // Assuming windVel is 4 characters long + null terminator
    static char windGust[5] = "xx.x"; // Assuming windGust is 4 characters long + null terminator
    static char batVoltage[5] = "0.0V";
    static char capVoltage[5] = "0.0V";
    static char temperature[5] = "00.0";
    static float batVoltageF = 0;
    static float capVoltageF = 0;
    static float temperatureF = 0;
    bool gotwind = false;

    while (Serial2.available())
    {
        // clear serialBytes buffer
        memset(serialBytes, '\0', sizeof(serialBytes));
        // memset(formattedString, '\0', sizeof(formattedString));
        serialPayloadSize = Serial2.readBytes(serialBytes, 512);
        // check for a strings we care about
        // example output of serial data fields from the WS85
        // WindDir      = 79
        // WindSpeed    = 0.5
        // WindGust     = 0.6
        // GXTS04Temp   = 24.4
        if (serialPayloadSize > 0)
        {
            // Define variables for line processing
            int lineStart = 0;
            int lineEnd = -1;

            // Process each byte in the received data
            for (size_t i = 0; i < serialPayloadSize; i++)
            {
                // go until we hit the end of line and then process the line
                if (serialBytes[i] == '\n')
                {
                    lineEnd = i;
                    // Extract the current line
                    char line[meshtastic_Constants_DATA_PAYLOAD_LEN];
                    memset(line, '\0', sizeof(line));
                    memcpy(line, &serialBytes[lineStart], lineEnd - lineStart);

                    if (strstr(line, "Wind") != NULL) // we have a wind line
                    {
                        gotwind = true;
                        // Find the positions of "=" signs in the line
                        char *windDirPos = strstr(line, "WindDir      = ");
                        char *windSpeedPos = strstr(line, "WindSpeed    = ");
                        char *windGustPos = strstr(line, "WindGust     = ");

                        if (windDirPos != NULL)
                        {
                            // Extract data after "=" for WindDir
                            strcpy(windDir, windDirPos + 15); // Add 15 to skip "WindDir = "
                            double radians = GeoCoord::toRadians(strtof(windDir, nullptr));
                            dir_sum_sin += sin(radians);
                            dir_sum_cos += cos(radians);
                            dirCount++;
                        }
                        else if (windSpeedPos != NULL)
                        {
                            // Extract data after "=" for WindSpeed
                            strcpy(windVel, windSpeedPos + 15); // Add 15 to skip "WindSpeed = "
                            float newv = strtof(windVel, nullptr);
                            velSum += newv;
                            velCount++;
                            if (newv < lull || lull == -1)
                                lull = newv;
                        }
                        else if (windGustPos != NULL)
                        {
                            strcpy(windGust, windGustPos + 15); // Add 15 to skip "WindSpeed = "
                            float newg = strtof(windGust, nullptr);
                            if (newg > gust)
                                gust = newg;
                        }

                        // these are also voltage data we care about possibly
                    }
                    else if (strstr(line, "BatVoltage") != NULL)
                    { // we have a battVoltage line
                        char *batVoltagePos = strstr(line, "BatVoltage     = ");
                        if (batVoltagePos != NULL)
                        {
                            strcpy(batVoltage, batVoltagePos + 17); // 18 for ws 80, 17 for ws85
                            batVoltageF = strtof(batVoltage, nullptr);
                            break; // last possible data we want so break
                        }
                    }
                    else if (strstr(line, "CapVoltage") != NULL)
                    { // we have a cappVoltage line
                        char *capVoltagePos = strstr(line, "CapVoltage     = ");
                        if (capVoltagePos != NULL)
                        {
                            strcpy(capVoltage, capVoltagePos + 17); // 18 for ws 80, 17 for ws85
                            capVoltageF = strtof(capVoltage, nullptr);
                        }
                        // GXTS04Temp   = 24.4
                    }
                    else if (strstr(line, "GXTS04Temp") != NULL)
                    { // we have a temperature line
                        char *tempPos = strstr(line, "GXTS04Temp   = ");
                        if (tempPos != NULL)
                        {
                            strcpy(temperature, tempPos + 15); // 15 spaces for ws85
                            temperatureF = strtof(temperature, nullptr);
                        }
                    }

                    // Update lineStart for the next line
                    lineStart = lineEnd + 1;
                }
            }
            break;
            // clear the input buffer
            while (Serial2.available() > 0)
            {
                Serial2.read(); // Read and discard the bytes in the input buffer
            }
        }
    }
    if (gotwind)
    {

        LOG_INFO("WS85 : %i %.1fg%.1f %.1fv %.1fv %.1fC", atoi(windDir), strtof(windVel, nullptr), strtof(windGust, nullptr),
                 batVoltageF, capVoltageF, temperatureF);
    }
    if (gotwind && !Throttle::isWithinTimespanMs(lastAveraged, averageIntervalMillis))
    {
        // calculate averages and send to the mesh
        float velAvg = 1.0 * velSum / velCount;

        double avgSin = dir_sum_sin / dirCount;
        double avgCos = dir_sum_cos / dirCount;

        double avgRadians = atan2(avgSin, avgCos);
        float dirAvg = GeoCoord::toDegrees(avgRadians);

        if (dirAvg < 0)
        {
            dirAvg += 360.0;
        }
        lastAveraged = millis();

        // make a telemetry packet with the data
        meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
        m.which_variant = meshtastic_Telemetry_environment_metrics_tag;

        m.variant.environment_metrics.wind_speed = velAvg;
        m.variant.environment_metrics.has_wind_speed = true;

        m.variant.environment_metrics.wind_direction = dirAvg;
        m.variant.environment_metrics.has_wind_direction = true;

        m.variant.environment_metrics.temperature = temperatureF;
        m.variant.environment_metrics.has_temperature = true;

        m.variant.environment_metrics.voltage =
            capVoltageF > batVoltageF ? capVoltageF : batVoltageF; // send the larger of the two voltage values.
        m.variant.environment_metrics.has_voltage = true;

        m.variant.environment_metrics.wind_gust = gust;
        m.variant.environment_metrics.has_wind_gust = true;

        if (lull == -1)
            lull = 0;
        m.variant.environment_metrics.wind_lull = lull;
        m.variant.environment_metrics.has_wind_lull = true;

        LOG_INFO("WS85 Transmit speed=%fm/s, direction=%d , lull=%f, gust=%f, voltage=%f temperature=%f",
                 m.variant.environment_metrics.wind_speed, m.variant.environment_metrics.wind_direction,
                 m.variant.environment_metrics.wind_lull, m.variant.environment_metrics.wind_gust,
                 m.variant.environment_metrics.voltage, m.variant.environment_metrics.temperature);

        sendTelemetry(m);

        // reset counters and gust/lull
        velSum = velCount = dirCount = 0;
        dir_sum_sin = dir_sum_cos = 0;
        gust = 0;
        lull = -1;
    }
#endif
    return;
}
#endif