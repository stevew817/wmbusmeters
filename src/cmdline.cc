/*
 Copyright (C) 2017-2020 Fredrik Öhrström

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

#include"cmdline.h"
#include"meters.h"
#include"util.h"

#include<string>

using namespace std;

shared_ptr<Configuration> parseCommandLine(int argc, char **argv) {

    Configuration * c = new Configuration;

    int i=1;
    const char *filename = strrchr(argv[0], '/');
    if (filename) {
        filename++;
    } else {
        filename = argv[0];
    }
    if (!strcmp(filename, "wmbusmetersd")) {
        c->daemon = true;
        if (argc < 2) {
            error("Usage error: wmbusmetersd must have at least a single argument to the pid file.\n"
                  "But you can also supply --device= and --listento= to override the config files.\n");
        }
        int i = 1;
        for (;;)
        {
            if (argv[i] == NULL) break;
            if (!strncmp(argv[i], "--device=", 9))
            {
                c->device_override = string(argv[i]+9);
                debug("(daemon) device override \"%s\"\n", c->device_override.c_str());
                i++;
                continue;
            }
            if (!strncmp(argv[i], "--listento=", 11))
            {
                c->listento_override = string(argv[i]+11);
                debug("(daemon) listento override \"%s\"\n", c->listento_override.c_str());
                i++;
                continue;
            }
            break;
        }
        c->pid_file = argv[i];
        return shared_ptr<Configuration>(c);
    }
    if (argc < 2) {
        c->need_help = true;
        return shared_ptr<Configuration>(c);
    }
    while (argv[i] && argv[i][0] == '-')
    {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "-help") || !strcmp(argv[i], "--help")) {
            c->need_help = true;
            return shared_ptr<Configuration>(c);
        }
        if (!strcmp(argv[i], "--silent")) {
            c->silent = true;
            i++;
            continue;
        }
        if (!strcmp(argv[i], "--verbose")) {
            c->verbose = true;
            i++;
            continue;
        }
        if (!strcmp(argv[i], "--version")) {
            c->version = true;
            return shared_ptr<Configuration>(c);
        }
        if (!strcmp(argv[i], "--license")) {
            c->license = true;
            return shared_ptr<Configuration>(c);
        }
        if (!strcmp(argv[i], "--debug")) {
            c->debug = true;
            i++;
            continue;
        }
        if (!strcmp(argv[i], "--trace")) {
            c->debug = true;
            c->trace = true;
            i++;
            continue;
        }
        if (!strcmp(argv[i], "--internaltesting")) {
            c->internaltesting = true;
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--donotprobe=", 13))
        {
            string df = string(argv[i]+13);
            debug("(cmdline) do not probe \"%s\"\n", df.c_str());
            c->do_not_probe_ttys.insert(df);
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--listento=", 11))
        {
            LinkModeSet lms = parseLinkModes(argv[i]+11);
            if (lms.empty())
            {
                error("Unknown default link mode \"%s\"!\n", argv[i]+11);
            }
            if (!c->default_device_linkmodes.empty()) {
                error("You have already specified a default link mode!\n");
            }
            c->default_device_linkmodes = lms;
            i++;
            continue;
        }

        LinkMode lm = isLinkModeOption(argv[i]);
        if (lm != LinkMode::UNKNOWN) {
            if (!c->default_device_linkmodes.empty())
            {
                error("You have already specified a default link mode!\n");
            }
            // Add to the empty set.
            c->default_device_linkmodes.addLinkMode(lm);
            i++;
            continue;
        }
        if (!strcmp(argv[i], "--logtelegrams")) {
            c->logtelegrams = true;
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--useconfig", 11)) {
            if (strlen(argv[i]) == 11)
            {
                c->useconfig = true;
                c->config_root = "";
                return shared_ptr<Configuration>(c);
            }
            else if (strlen(argv[i]) > 12 && argv[i][11] == '=')
            {
                size_t len = strlen(argv[i]) - 12;
                c->useconfig = true;
                c->config_root = string(argv[i]+12, len);
                if (c->config_root == "/") {
                    c->config_root = "";
                }
            }
            else
            {
                error("You must supply a directory to --useconfig=dir\n");
            }
            i++;
            for (;;)
            {
                if (argv[i] == NULL) break;
                if (!strncmp(argv[i], "--device=", 9))
                {
                    c->device_override = string(argv[i]+9);
                    debug("(useconfig) device override \"%s\"\n", c->device_override.c_str());
                    i++;
                    continue;
                }
                if (!strncmp(argv[i], "--listento=", 11))
                {
                    c->listento_override = string(argv[i]+11);
                    debug("(useconfig) listento override \"%s\"\n", c->listento_override.c_str());
                    i++;
                    continue;
                }
                break;
            }
            if (i+1 < argc) {
                error("Usage error: --useconfig can only be followed by --device= and --listento=\n");
            }
            return shared_ptr<Configuration>(c);
            continue;
        }
        if (!strcmp(argv[i], "--reload")) {
            c->reload = true;
            if (i > 1 || argc > 2) {
                error("Usage error: --reload implies no other arguments on the command line.\n");
            }
            return shared_ptr<Configuration>(c);
        }
        if (!strncmp(argv[i], "--format=", 9))
        {
            if (!strcmp(argv[i]+9, "json"))
            {
                c->json = true;
                c->fields = false;
            }
            else
            if (!strcmp(argv[i]+9, "fields"))
            {
                c->json = false;
                c->fields = true;
                c->separator = ';';
            }
            else
            if (!strcmp(argv[i]+9, "hr"))
            {
                c->json = false;
                c->fields = false;
                c->separator = '\t';
            }
            else
            {
                error("Unknown output format: \"%s\"\n", argv[i]+9);
            }
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--addconversions=", 17)) {
            if (strlen(argv[i]) > 16)
            {
                string s = string(argv[i]+17);
                handleConversions(c, s);
            } else {
                error("You must supply conversion units.\n");
            }
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--selectfields=", 15)) {
            if (strlen(argv[i]) > 15)
            {
                string s = string(argv[i]+15);
                handleSelectedFields(c, s);
            } else {
                error("You must supply fields to be selected.\n");
            }
            i++;
            continue;
        }

        if (!strncmp(argv[i], "--separator=", 12)) {
            if (!c->fields) {
                error("You must specify --format=fields before --separator=X\n");
            }
            if (strlen(argv[i]) != 13) {
                error("You must supply a single character as the field separator.\n");
            }
            c->separator = argv[i][12];
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--meterfilesaction", 18)) {
            if (strlen(argv[i]) > 18 && argv[i][18] == '=') {
                if (!strncmp(argv[i]+19, "overwrite", 9)) {
                    c->meterfiles_action = MeterFileType::Overwrite;
                } else if (!strncmp(argv[i]+19, "append", 6)) {
                    c->meterfiles_action = MeterFileType::Append;
                } else {
                    error("No such meter file action %s\n", argv[i]+19);
                }
            } else {
                error("Incorrect option %s\n", argv[i]);
            }
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--meterfilesnaming", 18)) {
            if (strlen(argv[i]) > 18 && argv[i][18] == '=') {
                if (!strncmp(argv[i]+19, "name-id", 7))
                {
                    c->meterfiles_naming = MeterFileNaming::NameId;
                }
                else if (!strncmp(argv[i]+19, "name", 4))
                {
                    c->meterfiles_naming = MeterFileNaming::Name;
                }
                else if (!strncmp(argv[i]+19, "id", 2))
                {
                    c->meterfiles_naming = MeterFileNaming::Id;
                } else
                {
                    error("No such meter file naming %s\n", argv[i]+19);
                }
            } else {
                error("Incorrect option %s\n", argv[i]);
            }
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--meterfilestimestamp", 21)) {
            if (strlen(argv[i]) > 22 && argv[i][21] == '=') {
                if (!strncmp(argv[i]+22, "day", 3))
                {
                    c->meterfiles_timestamp = MeterFileTimestamp::Day;
                }
                else if (!strncmp(argv[i]+22, "hour", 4))
                {
                    c->meterfiles_timestamp = MeterFileTimestamp::Hour;
                }
                else if (!strncmp(argv[i]+22, "minute", 6))
                {
                    c->meterfiles_timestamp = MeterFileTimestamp::Minute;
                }
                else if (!strncmp(argv[i]+22, "micros", 5))
                {
                    c->meterfiles_timestamp = MeterFileTimestamp::Micros;
                }
                else if (!strncmp(argv[i]+22, "never", 5))
                {
                    c->meterfiles_timestamp = MeterFileTimestamp::Never;
                } else
                {
                    error("No such meter file timestamp \"%s\"\n", argv[i]+22);
                }
            } else {
                error("Incorrect option %s\n", argv[i]);
            }
            i++;
            continue;
        }
        if (!strcmp(argv[i], "--meterfiles") ||
            (!strncmp(argv[i], "--meterfiles", 12) &&
             strlen(argv[i]) > 12 &&
             argv[i][12] == '='))
        {
            c->meterfiles = true;
            size_t len = strlen(argv[i]);
            if (len > 13) {
                c->meterfiles_dir = string(argv[i]+13, len-13);
            } else {
                c->meterfiles_dir = "/tmp";
            }
            if (!checkIfDirExists(c->meterfiles_dir.c_str())) {
                error("Cannot write meter files into dir \"%s\"\n", c->meterfiles_dir.c_str());
            }
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--logfile=", 10)) {
            c->use_logfile = true;
            if (strlen(argv[i]) > 10) {
                size_t len = strlen(argv[i])-10;
                if (len > 0) {
                    c->logfile = string(argv[i]+10, len);
                } else {
                    error("Not a valid log file name.");
                }
            }
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--usestderr", 11)) {
            c->use_stderr_for_log = true;
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--ignoreduplicates", 18)) {
            c->ignore_duplicate_telegrams = true;
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--usestdoutforlogging", 13)) {
            c->use_stderr_for_log = false;
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--shell=", 8)) {
            string cmd = string(argv[i]+8);
            if (cmd == "") {
                error("The telegram shell command cannot be empty.\n");
            }
            c->telegram_shells.push_back(cmd);
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--alarmshell=", 13)) {
            string cmd = string(argv[i]+13);
            if (cmd == "") {
                error("The alarm shell command cannot be empty.\n");
            }
            c->alarm_shells.push_back(cmd);
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--json_", 7))
        {
            // For example: --json_floor=42
            string json = string(argv[i]+7);
            if (json == "") {
                error("The json command cannot be empty.\n");
            }
            // The extra "floor"="42" will be pushed to the json.
            debug("Added json %s\n", json.c_str());
            c->jsons.push_back(json);
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--listenvs=", 11)) {
            c->list_shell_envs = true;
            c->list_meter = string(argv[i]+11);
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--listfields=", 13)) {
            c->list_fields = true;
            c->list_meter = string(argv[i]+13);
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--listmeters=", 13)) {
            c->list_meters = true;
            c->list_meters_search = string(argv[i]+13);
            i++;
            continue;
        }
        else if (!strncmp(argv[i], "--listmeters", 12)) {
            c->list_meters = true;
            c->list_meters_search = "";
            i++;
            continue;
        }
        if (!strcmp(argv[i], "--oneshot")) {
            c->oneshot = true;
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--exitafter=", 12) && strlen(argv[i]) > 12) {
            c->exitafter = parseTime(argv[i]+12);
            if (c->exitafter <= 0) {
                error("Not a valid time to exit after. \"%s\"\n", argv[i]+12);
            }
            i++;
            continue;
        }
        if (!strcmp(argv[i], "--nodeviceexit")) {
            c->nodeviceexit = true;
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--resetafter=", 13) && strlen(argv[i]) > 13) {
            c->resetafter = parseTime(argv[i]+13);
            if (c->resetafter <= 0) {
                error("Not a valid time to regularly reset after. \"%s\"\n", argv[i]+13);
            }
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--alarmtimeout=", 15)) {
            c->alarm_timeout = parseTime(argv[i]+15);
            if (c->alarm_timeout <= 0) {
                error("Not a valid alarm timeout. \"%s\"\n", argv[i]+15);
            }
            i++;
            continue;
        }
        if (!strncmp(argv[i], "--alarmexpectedactivity=", 24)) {
            string ea = string(argv[i]+24);
            if (!isValidTimePeriod(ea))
            {
                error("Not a valid time period string. \"%s\"\n", ea.c_str());
            }
            c->alarm_expected_activity = ea;
            i++;
            continue;
        }

        if (!strcmp(argv[i], "--")) {
            i++;
            break;
        }
        error("Unknown option \"%s\"\n", argv[i]);
    }

    while (argv[i])
    {
        bool ok = handleDevice(c, argv[i]);
        if (!ok)
        {
            if (!argv[i+1])
            {
                // This was the last argument on the commandline.
                // It should have been a device or a file.
                error("Not a valid device \"%s\"\n", argv[i]);
            }
            // There are more arguments...
            break;
        }
        i++;
    }


    if (c->supplied_wmbus_devices.size() == 0 &&
        c->use_auto_device_detect == false &&
        !c->list_shell_envs &&
        !c->list_fields &&
        !c->list_meters)
    {
        error("You must supply at least one device (eg auto:c1) to receive wmbus telegrams.\n");
    }

    if ((argc-i) % 4 != 0) {
        error("For each meter you must supply a: name,type,id and key.\n");
    }
    int num_meters = (argc-i)/4;

    for (int m=0; m<num_meters; ++m) {
        string name = argv[m*4+i+0];
        string type = argv[m*4+i+1];
        string id = argv[m*4+i+2];
        string key = argv[m*4+i+3];
        LinkModeSet modes;
        size_t colon = type.find(':');
        MeterType mt = toMeterType(type);
        if (colon != string::npos)
        {
            // The config can be supplied after the type, like this:
            // apator162:c1
            string modess = type.substr(colon+1);
            type = type.substr(0, colon);
            mt = toMeterType(type);
            if (mt == MeterType::UNKNOWN) error("Not a valid meter type \"%s\"\n", type.c_str());
            modes = parseLinkModes(modess);
            LinkModeSet default_modes = toMeterLinkModeSet(type);
            if (!default_modes.hasAll(modes))
            {
                string want = modes.hr();
                string has = default_modes.hr();
                error("(cmdline) cannot set link modes to: %s because meter %s only transmits on: %s\n",
                      want.c_str(), type.c_str(), has.c_str());
            }
            string modeshr = modes.hr();
            debug("(cmdline) setting link modes to %s for meter %s\n",
                  modeshr.c_str(), name.c_str());
        }
        else {
            modes = toMeterLinkModeSet(type);
        }

        mt = toMeterType(type);

        if (mt == MeterType::UNKNOWN) error("Not a valid meter type \"%s\"\n", type.c_str());
        if (!isValidMatchExpressions(id, true)) error("Not a valid id nor a valid meter match expression \"%s\"\n", id.c_str());
        if (!isValidKey(key, mt)) error("Not a valid meter key \"%s\"\n", key.c_str());
        vector<string> no_meter_shells, no_meter_jsons;
        c->meters.push_back(MeterInfo(name, type, id, key, modes, no_meter_shells, no_meter_jsons));
    }

    return shared_ptr<Configuration>(c);
}
