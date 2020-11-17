/*
 Copyright (C) 2019-2020 Fredrik Öhrström

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

#include"wmbus.h"
#include"wmbus_common_implementation.h"
#include"wmbus_utils.h"
#include"wmbus_cul.h"
#include"serial.h"

#include<assert.h>
#include<fcntl.h>
#include<grp.h>
#include<pthread.h>
#include<semaphore.h>
#include<string.h>
#include<errno.h>
#include<sys/stat.h>
#include<sys/types.h>
#include<unistd.h>

using namespace std;

#define SET_LINK_MODE 1
#define SET_X01_MODE 2

struct WMBusCUL : public virtual WMBusCommonImplementation
{
    bool ping();
    string getDeviceId();
    string getDeviceUniqueId();
    LinkModeSet getLinkModes();
    void deviceReset();
    void deviceSetLinkModes(LinkModeSet lms);
    LinkModeSet supportedLinkModes() {
        return
            C1_bit |
            S1_bit |
            T1_bit;
    }
    int numConcurrentLinkModes() { return 1; }
    bool canSetLinkModes(LinkModeSet lms)
    {
        if (lms.empty()) return false;
        if (!supportedLinkModes().supports(lms)) return false;
        // Ok, the supplied link modes are compatible,
        // but im871a can only listen to one at a time.
        return 1 == countSetBits(lms.asBits());
    }
    void processSerialData();
    void simulate();

    WMBusCUL(shared_ptr<SerialDevice> serial, shared_ptr<SerialCommunicationManager> manager);
    ~WMBusCUL() { }

private:

    LinkModeSet link_modes_ {};
    vector<uchar> read_buffer_;
    vector<uchar> received_payload_;
    string sent_command_;
    string received_response_;

    FrameStatus checkCULFrame(vector<uchar> &data,
                              size_t *hex_frame_length,
                              vector<uchar> &payload);

    string setup_;
};

shared_ptr<WMBus> openCUL(string device, shared_ptr<SerialCommunicationManager> manager, shared_ptr<SerialDevice> serial_override)
{
    if (serial_override)
    {
        WMBusCUL *imp = new WMBusCUL(serial_override, manager);
        imp->markAsNoLongerSerial();
        return shared_ptr<WMBus>(imp);
    }

    auto serial = manager->createSerialDeviceTTY(device.c_str(), 38400, "cul");
    WMBusCUL *imp = new WMBusCUL(serial, manager);
    return shared_ptr<WMBus>(imp);
}

WMBusCUL::WMBusCUL(shared_ptr<SerialDevice> serial, shared_ptr<SerialCommunicationManager> manager) :
    WMBusCommonImplementation(DEVICE_CUL, manager, serial, true)
{
    reset();
}

bool WMBusCUL::ping()
{
    verbose("(cul) ping\n");
    return true;
}

string WMBusCUL::getDeviceId()
{
    verbose("(cul) getDeviceId\n");
    return "";
}

string WMBusCUL::getDeviceUniqueId()
{
    verbose("(cul) getDeviceUniqueId\n");
    return "";
}

LinkModeSet WMBusCUL::getLinkModes()
{
    return link_modes_;
}

void WMBusCUL::deviceReset()
{
    // No device specific settings needed right now.
    // The common code in wmbus.cc reset()
    // will open the serial device and potentially
    // set the link modes properly.
}

void WMBusCUL::deviceSetLinkModes(LinkModeSet lms)
{
    if (serial()->readonly()) return; // Feeding from stdin or file.

    if (!canSetLinkModes(lms))
    {
        string modes = lms.hr();
        error("(cul) setting link mode(s) %s is not supported\n", modes.c_str());
    }
    // 'brc' command: b - wmbus, r - receive, c - c mode (with t)
    vector<uchar> msg(5);
    msg[0] = 'b';
    msg[1] = 'r';
    if (lms.has(LinkMode::C1)) {
        msg[2] = 'c';
    } else if (lms.has(LinkMode::S1)) {
        msg[2] = 's';
    } else if (lms.has(LinkMode::T1)) {
        msg[2] = 't';
    }
    msg[3] = 0xa;
    msg[4] = 0xd;

    verbose("(cul) set link mode %c\n", msg[2]);
    sent_command_ = string(&msg[0], &msg[3]);
    received_response_ = "";
    bool sent = serial()->send(msg);

    if (sent) waitForResponse(1);

    sent_command_ = "";
    debug("(cul) received \"%s\"", received_response_.c_str());

    bool ok = true;
    if (lms.has(LinkMode::C1)) {
        if (received_response_ != "CMODE") ok = false;
    } else if (lms.has(LinkMode::S1)) {
        if (received_response_ != "SMODE") ok = false;
    } else if (lms.has(LinkMode::T1)) {
        if (received_response_ != "TMODE") ok = false;
    }

    if (!ok)
    {
        string modes = lms.hr();
        error("(cul) setting link mode(s) %s is not supported for this cul device!\n", modes.c_str());
    }

    // X01 - start the receiver
    msg[0] = 'X';
    msg[1] = '0'; // 6
    msg[2] = '1'; // 7
    msg[3] = 0xa;
    msg[4] = 0xd;

    sent = serial()->send(msg);

    // Any response here, or does it silently move into listening mode?
}

void WMBusCUL::simulate()
{
}

string expectedResponses(vector<uchar> &data)
{
    string safe = safeString(data);
    if (safe.find("CMODE") != string::npos) return "CMODE";
    if (safe.find("TMODE") != string::npos) return "TMODE";
    if (safe.find("SMODE") != string::npos) return "SMODE";
    return "";
}

void WMBusCUL::processSerialData()
{
    vector<uchar> data;

    // Receive and accumulated serial data until a full frame has been received.
    serial()->receive(&data);
    read_buffer_.insert(read_buffer_.end(), data.begin(), data.end());

    size_t frame_length;
    vector<uchar> payload;

    for (;;)
    {
        FrameStatus status = checkCULFrame(read_buffer_, &frame_length, payload);

        if (status == PartialFrame)
        {
            break;
        }
        if (status == TextAndNotFrame)
        {
            // The buffer has already been printed by serial cmd.
            if (sent_command_ != "")
            {
                string r = expectedResponses(read_buffer_);
                if (r != "")
                {
                    received_response_ = r;
                    notifyResponseIsHere(1);
                }
            }
            read_buffer_.clear();
            break;
        }
        if (status == ErrorInFrame)
        {
            debug("(cul) error in received message.\n");
            string msg = bin2hex(read_buffer_);
            read_buffer_.clear();
            break;
        }
        if (status == FullFrame)
        {
            read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin()+frame_length);

            // We do not currently know how to get the rssi out of the cul dongle.
            AboutTelegram about("cul", 0);
            handleTelegram(about, payload);
        }
    }
}

FrameStatus WMBusCUL::checkCULFrame(vector<uchar> &data,
                                    size_t *hex_frame_length,
                                    vector<uchar> &payload)
{
    if (data.size() == 0) return PartialFrame;

    if (isDebugEnabled())
    {
        string s  = safeString(data);
        debug("(cul) checkCULFrame \"%s\"\n", s.c_str());
    }

    size_t eolp = 0;
    // Look for end of line
    for (; eolp < data.size(); ++eolp) {
        if (data[eolp] == '\n') break; // Expect CRLF, look for LF ('\n')
    }
    if (eolp >= data.size())
    {
        debug("(cul) no eol found yet, partial frame\n");
        return PartialFrame;
    }
    eolp++; // Point to byte after CRLF.
    // Normally it is CRLF, but enable code to handle single LF as well.
    int eof_len = data[eolp-2] == '\r' ? 2 : 1;
    // If it was a CRLF then eof_len == 2, else it is 1.
    if (data[0] != 'b')
    {
        // C1 and T1 telegrams should start with a 'b'
        debug("(cul) no leading 'b' so it is text and no frame\n");
        return TextAndNotFrame;
    }

    if (data[1] == 'Y')
    {
        // C1 telegram in frame format B
        // bY..44............<CR><LF>
        *hex_frame_length = eolp;
        vector<uchar> hex;
        // If reception is started with X01, then there are no RSSI bytes.
        // If started with X21, then there are two RSSI bytes (4 hex digits at the end).
        // Now we always start with X01.
        hex.insert(hex.end(), data.begin()+2, data.begin()+eolp-eof_len); // Remove CRLF
        payload.clear();
        bool ok = hex2bin(hex, &payload);
        if (!ok)
        {
            string s = safeString(hex);
            debug("(cul) bad hex \"%s\"\n", s.c_str());
            warning("(cul) warning: the hex string is not proper! Ignoring telegram!\n");
            return ErrorInFrame;
        }
        ok = trimCRCsFrameFormatB(payload);
        if (!ok)
        {
            warning("(cul) dll C1 (frame b) crcs failed check! Ignoring telegram!\n");
            return ErrorInFrame;
        }
        debug("(cul) received full C1 frame\n");
        return FullFrame;
    }
    else
    {
        // T1 telegram in frame format A
        // b..44..............<CR><LF>
        *hex_frame_length = eolp;
        vector<uchar> hex;
        // If reception is started with X01, then there are no RSSI bytes.
        // If started with X21, then there are two RSSI bytes (4 hex digits at the end).
        // Now we always start with X01.
        hex.insert(hex.end(), data.begin()+1, data.begin()+eolp-eof_len); // Remove CRLF
        payload.clear();
        bool ok = hex2bin(hex, &payload);
        if (!ok)
        {
            string s = safeString(hex);
            debug("(cul) bad hex \"%s\"\n", s.c_str());
            warning("(cul) warning: the hex string is not proper! Ignoring telegram!\n");
            return ErrorInFrame;
        }
        ok = trimCRCsFrameFormatA(payload);
        if (!ok)
        {
            warning("(cul) dll T1 (frame a) crcs failed check! Ignoring telegram!\n");
            return ErrorInFrame;
        }
        debug("(cul) received full T1 frame\n");
        return FullFrame;
    }
}

AccessCheck detectCUL(Detected *detected, shared_ptr<SerialCommunicationManager> manager)
{
    // Talk to the device and expect a very specific answer.
    auto serial = manager->createSerialDeviceTTY(detected->found_file.c_str(), 38400, "detect cul");
    serial->disableCallbacks();
    AccessCheck rc = serial->open(false);
    if (rc != AccessCheck::AccessOK) return AccessCheck::NotThere;

    bool found = false;
    for (int i=0; i<3; ++i)
    {
        verbose("(cul) get version\n");
        // Try three times, it seems slow sometimes.
        vector<uchar> data;

        // get the version string: "V 1.67 nanoCUL868" or similar
        vector<uchar> msg(3);
        msg[0] = CMD_GET_VERSION; // V
        msg[1] = 0x0a;
        msg[2] = 0x0d;

        bool ok = serial->send(msg);
        if (!ok) return AccessCheck::NotThere;

        // Wait for 200ms so that the USB stick have time to prepare a response.
        usleep(1000*200);
        serial->receive(&data);
        string resp = safeString(data);
        debug("(cul) probe response \"%s\"\n", resp.c_str());
        if (resp.find("CUL") != string::npos)
        {
            found = true;
            break;
        }
        usleep(1000*500);
    }

    serial->close();

    if (!found)
    {
        return AccessCheck::NotThere;
    }

    detected->setAsFound("", WMBusDeviceType::DEVICE_CUL, 38400, false, false, detected->specified_device.linkmodes);

    return AccessCheck::AccessOK;
}
