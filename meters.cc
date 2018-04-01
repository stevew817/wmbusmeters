/*
 Copyright (C) 2017-2018 Fredrik Öhrström

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

#include"meters.h"
#include"meters_common_implementation.h"

#include<memory.h>

MeterCommonImplementation::MeterCommonImplementation(WMBus *bus, const char *name, const char *id, const char *key,
                                                     MeterType type, int manufacturer, int media) :
    type_(type), manufacturer_(manufacturer), media_(media), name_(name), bus_(bus)
{
    use_aes_ = true;
    hex2bin(id, &id_);
    if (strlen(key) == 0) {
	use_aes_ = false;
    } else {
	hex2bin(key, &key_);
    }
}

MeterType MeterCommonImplementation::type()
{
    return type_;
}

int MeterCommonImplementation::manufacturer()
{
    return manufacturer_;
}

int MeterCommonImplementation::media()
{
    return media_;
}

string MeterCommonImplementation::id()
{
    return bin2hex(id_);
}

string MeterCommonImplementation::name()
{
    return name_;
}

WMBus *MeterCommonImplementation::bus()
{
    return bus_;
}

void MeterCommonImplementation::onUpdate(function<void(Meter*)> cb)
{
    on_update_.push_back(cb);
}

int MeterCommonImplementation::numUpdates()
{
    return num_updates_;
}

string MeterCommonImplementation::datetimeOfUpdateHumanReadable()
{
    char datetime[40];
    memset(datetime, 0, sizeof(datetime));
    strftime(datetime, 20, "%Y-%m-%d %H:%M.%S", localtime(&datetime_of_update_));
    return string(datetime);
}

string MeterCommonImplementation::datetimeOfUpdateRobot()
{
    char datetime[40];
    memset(datetime, 0, sizeof(datetime));
    // This is the date time in the Greenwich timezone (Zulu time), dont get surprised!
    strftime(datetime, sizeof(datetime), "%FT%TZ", gmtime(&datetime_of_update_));
    return string(datetime);
}

MeterType toMeterType(const char *type)
{
    if (!strcmp(type, "multical21")) return MULTICAL21_METER;
    if (!strcmp(type, "multical302")) return MULTICAL302_METER;
    if (!strcmp(type, "omnipower")) return OMNIPOWER_METER;
    if (!strcmp(type, "water")) return MULTICAL21_METER;
    if (!strcmp(type, "heat")) return MULTICAL302_METER;
    if (!strcmp(type, "electricity")) return OMNIPOWER_METER;
    return UNKNOWN_METER;
}


bool MeterCommonImplementation::isTelegramForMe(Telegram *t)
{
    return t->m_field == manufacturer_ &&
        t->a_field_address[3] == id_[3] &&
        t->a_field_address[2] == id_[2] &&
        t->a_field_address[1] == id_[1] &&
        t->a_field_address[0] == id_[0];
}

bool MeterCommonImplementation::useAes()
{
    return use_aes_;
}

vector<uchar> MeterCommonImplementation::key()
{
    return key_;
}

void MeterCommonImplementation::triggerUpdate(Telegram *t)
{
    datetime_of_update_ = time(NULL);
    num_updates_++;
    for (auto &cb : on_update_) if (cb) cb(this);
    t->handled = true;
}
