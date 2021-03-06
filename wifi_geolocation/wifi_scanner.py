#!/usr/bin/env python3

import json
import subprocess
import sys
import os
import time
import requests
import stat
import traceback

DEVNULL = open(os.devnull, 'w')

class WifiScanner(object):
    def __init__(self, log_error = None):
        if not log_error:
            def stderr_write(text):
                sys.stderr.write(text + "\n")
                sys.stderr.flush()
            self.log_error = stderr_write

        self.error_count = {
            "nmcli": 0,
            "iwlist": 0,
        }

        self.iwlist_path = None
        self.iwlist_suid = False

        if os.path.exists("/sbin/iwlist"):
            self.iwlist_path = "/sbin/iwlist"
            if os.stat("/sbin/iwlist").st_mode & stat.S_ISUID:
                self.iwlist_suid = True

        self.nmcli_path = None

        if os.path.exists("/usr/bin/nmcli"):
            self.nmcli_path = "/usr/bin/nmcli"

        if os.path.exists("/usr/local/bin/nmcli"):
            self.nmcli_path = "/usr/local/bin/nmcli"

        self.is_root = (os.getuid() == 0)

        if (not self.nmcli_path) and self.iwlist_path and (not self.iwlist_suid) and (not self.is_root):
            self.log_error("I didn't find nmcli on your system; I found iwlist, so I can use that to scan, " + \
                "but I can't trigger a full Wi-Fi scan with iwlist without root permissions. Either run this " + \
                "code as root, OR run `sudo chmod 4755 %s` to enable full Wi-Fi scanning." % self.iwlist_path)

        if not self.nmcli_path and not self.iwlist_path:
            raise Exception("I found neither nmcli nor iwlist found on your system. I don't know how to scan. Bye!")

    def scan(self):
        scan_results = None

        if self.error_count["nmcli"] <= self.error_count["iwlist"]:
            if self.nmcli_path:
                try:
                    scan_results = self._nmcli_wifi_scan()
                except (TypeError, ValueError):
                    self.error_count["nmcli"] += 1
                    self.log_error("nmcli error: %s" % traceback.format_exc())

            if not scan_results and self.iwlist_path:
                self.error_count["nmcli"] += 1
                try:
                    scan_results = self._iwlist_wifi_scan()
                except (TypeError, ValueError):
                    self.error_count["iwlist"] += 1
                    self.log_error("iwlist error: %s" % traceback.format_exc())

        else:
            if self.iwlist_path:
                try:
                    scan_results = self._iwlist_wifi_scan()
                except (TypeError, ValueError):
                    self.error_count["iwlist"] += 1
                    self.log_error("iwlist error: %s" % traceback.format_exc())

            if not scan_results and self.nmcli_path:
                self.error_count["iwlist"] += 1
                try:
                    scan_results = self._nmcli_wifi_scan()
                except (TypeError, ValueError):
                    self.error_count["nmcli"] += 1
                    self.log_error("nmcli error: %s" % traceback.format_exc())

        
        return scan_results

    def _nmcli_wifi_status(self):
        result = subprocess.check_output(["nmcli", "radio", "wifi"]).decode('utf-8').strip()
        return result

    def _nmcli_wifi_on(self):
        result = subprocess.check_output(["nmcli", "radio", "wifi", "on"]).decode('utf-8').strip()
        return result

    def _nmcli_wifi_scan(self):
        if self._nmcli_wifi_status() != "enabled":
            self._nmcli_wifi_on()
        try:
            subprocess.check_output(["nmcli", "dev", "wifi", "rescan"], stderr = subprocess.STDOUT)
        except subprocess.CalledProcessError as e:
            if "not authorized" in e.output.decode('utf-8'):
                log_msg = "nmcli found but not authorized to perform a rescan operation with nmcli.\n" + \
                "I will still run, but results may be severely limited in accuracy. To fix this:\n\n" + \
                    "Create /etc/polkit-1/localauthority/50-local.d/10-nmcli-wifi-scan.pkla and put in it:\n\n" + \
                    "[Allow wi-fi scans for all users]\n" + \
                    "Identity=unix-user:*\n" + \
                    "Action=org.freedesktop.NetworkManager.wifi.scan\n" + \
                    "ResultAny=yes\n" + \
                    "ResultInactive=yes\n" + \
                    "ResultActive=yes\n\n" + \
                    "Then run:\n\n  sudo service polkit restart && sudo service network-manager restart\n\n\n"
                self.log_error(log_msg)

        lines = subprocess.check_output(["nmcli", "-t", "dev", "wifi", "list"]).decode('utf-8').strip().split("\n")

        if len(lines) == 0:
            return None

        scan_results = []
        for line in lines:
            fields = line.replace("\\:", "").split(":")

            if len(fields) < 7:
                continue

            macAddress = fields[1].strip().lower()
            if len(macAddress) != 12:
                continue
            macAddress = ":".join([macAddress[0:2], macAddress[2:4], macAddress[4:6], macAddress[6:8], macAddress[8:10], macAddress[10:12]])

            scan_results.append({
                "macAddress": macAddress,
                "ssid": fields[2].strip(),
                "channel": int(fields[4].strip()),
                "signalLevel": 100 - int(fields[6].strip()),
                "age": 0,
            })

        return scan_results

    def _iwlist_wifi_scan(self):
        response = subprocess.check_output(['/sbin/iwlist', 'scanning'], stderr = DEVNULL)
        lines = list(filter(lambda x: len(x) > 0, response.decode('utf-8').strip().split('\n')))
        if len(lines) == 0:
            return None

        scan_results = []
        result = {}
        try:
            for line in response.decode('utf-8').split('\n'):
                line = line.strip()
                if line.startswith('Cell'):
                    if result != {}:
                        scan_results.append(result)
                        result = {}
                    result['macAddress'] = line.split('Address:')[1].strip()
                    result['age'] = 0
                if 'Signal level' in line:
                    result['signalLevel'] = int(line.split('Signal level=')[1].split(' ')[0])
                if 'Channel:' in line:
                    result['channel'] = int(line.split('Channel:')[1].strip())
        except (ValueError, IndexError, TypeError):
            self.log_error("iwlist returned strange data")

        return scan_results

def main(args=None):
    print("Testing mode")
    from pprint import pprint
    import time
    w = WifiScanner()
    while True:
        time.sleep(1)
        print(w.scan())

if __name__ == '__main__':
    main()
