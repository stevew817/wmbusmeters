/*
 Copyright (C) 2019-2020 Silicon Labs

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 This driver is made for a modified version of the Silicon Labs
 reference application example called 'wmbus collector'. This version
 can be found at <TODO>.
*/

#include "wmbus.h"
#include "wmbus_common_implementation.h"
#include "wmbus_utils.h"
#include "serial.h"

#include <assert.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <unistd.h>

using namespace std;

struct WMBusSilabs : public virtual WMBusCommonImplementation
{
    bool ping();
    string getDeviceId();
    string getDeviceUniqueId();
    LinkModeSet getLinkModes();
    void deviceReset();
    void deviceSetLinkModes(LinkModeSet lms);
    LinkModeSet supportedLinkModes() { return Any_bit; }
    int numConcurrentLinkModes() { return 0; }
    bool canSetLinkModes(LinkModeSet desired_modes) { return true; }

    void processSerialData();
    void simulate() { }

    WMBusSilabs(shared_ptr<SerialDevice> serial, shared_ptr<SerialCommunicationManager> manager);
    ~WMBusSilabs() { }

private:

    FrameStatus checkSilabsFrame(vector<uchar> &data,
                                 size_t *parsed_length);

    vector<uchar> read_buffer_;
    LinkModeSet link_modes_;
    vector<uchar> received_payload_;
};

shared_ptr<WMBus> openSilabs(string device, int baudrate, shared_ptr<SerialCommunicationManager> manager, shared_ptr<SerialDevice> serial_override)
{
    assert(device != "");

    if (serial_override)
    {
        WMBusSilabs *imp = new WMBusSilabs(serial_override, manager);
        return shared_ptr<WMBus>(imp);
    }
    auto serial = manager->createSerialDeviceTTY(device.c_str(), baudrate, "rawtty");
    WMBusSilabs *imp = new WMBusSilabs(serial, manager);
    return shared_ptr<WMBus>(imp);
}

WMBusSilabs::WMBusSilabs(shared_ptr<SerialDevice> serial, shared_ptr<SerialCommunicationManager> manager) :
    WMBusCommonImplementation(DEVICE_SILABS, manager, serial)
{
    reset();
}

bool WMBusSilabs::ping()
{
    return true;
}

string WMBusSilabs::getDeviceId()
{
    return "?";
}

string WMBusSilabs::getDeviceUniqueId()
{
    return "?";
}

LinkModeSet WMBusSilabs::getLinkModes() {
    return link_modes_;
}

void WMBusSilabs::deviceReset()
{
}

void WMBusSilabs::deviceSetLinkModes(LinkModeSet lms)
{
}

void WMBusSilabs::processSerialData()
{
    vector<uchar> data;

    // Receive and accumulated serial data until a full frame has been received.
    serial()->receive(&data);
    read_buffer_.insert(read_buffer_.end(), data.begin(), data.end());

    size_t parsed_length;

    for (;;)
    {
        FrameStatus status = checkSilabsFrame(read_buffer_, &parsed_length);

        if (status == PartialFrame)
        {
            break;
        }
        if (status == TextAndNotFrame)
        {
            read_buffer_.clear();
            break;
        }
        if (status == ErrorInFrame)
        {
            debug("(silabs) error in received message.\n");
            string msg = bin2hex(read_buffer_);
            read_buffer_.clear();
            break;
        }
        if (status == FullFrame)
        {
            read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin()+parsed_length);

            // Telegram has been passed to the core in checkSilabsFrame
        }
    }
}

FrameStatus WMBusSilabs::checkSilabsFrame( vector<uchar> &data,
                                           size_t *parsed_length )
{
    vector<uchar> payload;
    *parsed_length = 0;
    if (data.size() == 0) return PartialFrame;

    if (isDebugEnabled())
    {
        string s  = safeString(data);
        debug("(silabs) checkSilabsFrame \"%s\"\n", s.c_str());
    }

    size_t eolp = 0;
    // Look for end of line
    for (; eolp < data.size(); ++eolp) {
        if (data[eolp] == '\n') break; // Expect CRLF, look for LF ('\n')
    }
    if (eolp >= data.size())
    {
        debug("(silabs) no eol found yet, partial frame\n");
        return PartialFrame;
    }
    eolp++; // Point to byte after CRLF.
    *parsed_length = eolp; // If we found the newline marker, we'll parse everything on the line.
    // Normally it is CRLF, but enable code to handle single LF as well.
    int eof_len = data[eolp-2] == '\r' ? 2 : 1;
    // If it was a CRLF then eof_len == 2, else it is 1.
    if (data[0] != 'R' || data[1] != 'X' || data[2] != ':')
    {
        // wmbus telegrams should start with 'RX'
        debug("(silabs) no leading 'RX:' so it is text and no frame\n");
        return TextAndNotFrame;
    }

    // Eat RX
    auto start = data.begin();
    auto stop = std::find(start, data.end(), ':');
    if (stop == data.end()) {
        debug("(silabs) not enough delimiters\n");
        return TextAndNotFrame;
    }

    debug("(silabs) RX:\n");
    start = stop + 1;

    // Eat timestamp
    stop = std::find(start, data.end(), ':');
    if (stop == data.end()) {
        debug("(silabs) not enough delimiters\n");
        return TextAndNotFrame;
    }

    string ts = std::string(start, stop);
    debug("(silabs) timestamp %s\n", ts.c_str());
    start = stop + 1;

    // Eat RSSI
    stop = std::find(start, data.end(), ':');
    if (stop == data.end()) {
        debug("(silabs) not enough delimiters\n");
        return TextAndNotFrame;
    }

    string rssi = std::string(start, stop);
    debug("(silabs) rssi %s\n", rssi.c_str());
    start = stop + 1;

    // Eat mode
    stop = std::find(start, data.end(), ':');
    if (stop == data.end()) {
        debug("(silabs) not enough delimiters\n");
        return TextAndNotFrame;
    }

    string mode = std::string(start, stop);
    debug("(silabs) mode %s\n", mode.c_str());
    start = stop + 1;

    // Eat frame type
    stop = std::find(start, data.end(), ':');
    if (stop == data.end()) {
        debug("(silabs) not enough delimiters\n");
        return TextAndNotFrame;
    }

    string type = std::string(start, stop);
    debug("(silabs) type %s\n", type.c_str());
    start = stop + 1;

    // Convert ASCII hex packet to binary
    vector<uchar> hex;
    hex.insert(hex.end(), start, data.begin()+eolp-eof_len); // Remove CRLF
    payload.clear();
    bool ok = hex2bin(hex, &payload);
    if (!ok)
    {
        string s = safeString(hex);
        debug("(silabs) bad hex \"%s\"\n", s.c_str());
        warning("(silabs) warning: malformatted ASCII hex, ignoring telegram.\n");
        return ErrorInFrame;
    }

    if (type.compare("A") == 0) {
        ok = trimCRCsFrameFormatA(payload);
    } else if (type.compare("B") == 0) {
        ok = trimCRCsFrameFormatB(payload);
    } else {
        warning("(silabs) unrecognized frame type %s\n", type.c_str());
        return ErrorInFrame;
    }

    if (!ok)
    {
        warning("(silabs) CRC check fail on frame type %s mode %s! Ignoring telegram!\n", type.c_str(), mode.c_str());
        return ErrorInFrame;
    }
    debug("(silabs) received full frame\n");

    AboutTelegram about("silabs", atoi(rssi.c_str()));
    handleTelegram(about, payload);

    return FullFrame;
}

AccessCheck detectSILABS(Detected *detected, shared_ptr<SerialCommunicationManager> manager)
{
    string tty = detected->specified_device.file;
    int bps = atoi(detected->specified_device.bps.c_str());

    // Since we do not know how to talk to the other end, it might not
    // even respond. The only thing we can do is to try to open the serial device.
    auto serial = manager->createSerialDeviceTTY(tty.c_str(), bps, "detect silabs");
    AccessCheck rc = serial->open(false);
    if (rc != AccessCheck::AccessOK) return AccessCheck::NotThere;

    serial->close();

    detected->setAsFound("silabs", WMBusDeviceType::DEVICE_SILABS, bps, false, false);

    return AccessCheck::AccessOK;
}
