#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
import subprocess
from sensor_msgs.msg import NavSatFix
import sys
import os
import time
import requests
import stat

DEVNULL = open(os.devnull, 'w')

class WifiGeolocationNode(Node):
    def __init__(self, node_name = "wifi_geolocation_node"):
        super().__init__(node_name = node_name)
        self.log = self.get_logger()
        self.clock = self.get_clock()

        if not os.path.exists("/sbin/iwlist"):
            self.log.fatal("Could not find /sbin/iwlist")
            exit(1)

        if not os.path.exists("/usr/bin/nmcli"):
            if not (os.stat("/sbin/iwlist").st_mode & stat.S_ISUID):
                self.log.warn("nmcli appears to not be installed. That's okay, but I cannot trigger a full Wi-Fi scan as a non-root user. Run `sudo chmod 4755 /sbin/iwlist` to resolve this.")
            self.has_nmcli = False
        else:
            self.has_nmcli = True

        self.declare_parameter('provider', 'mozilla')
        self.declare_parameter('api_key', 'test')
        self.declare_parameter('interval', 10.0)
        self.provider = self.get_parameter('provider')._value

        if self.provider == "mozilla":
            self.url = 'https://location.services.mozilla.com/v1/geolocate?key=%s' % self.get_parameter('api_key')._value
        elif self.provider == "google":
            self.url = 'https://www.googleapis.com/geolocation/v1/geolocate?key=%s' % self.get_parameter('api_key')._value
        else:
            self.log.fatal("invalid provider %s" % self.provider)
            exit(1)

        self.pub_fix = self.create_publisher(NavSatFix, "fix", 10)

        self.timer = self.create_timer(self.get_parameter('interval')._value, self.on_timer)

    def on_timer(self):
        scan_data = self.scan()
        result = self.geolocate(scan_data)
        if result is None:
            self.log.warn("could not geolocate")
            return

        msg = NavSatFix()
        msg.latitude = result.get("location", {}).get("lat", 0.0)
        msg.longitude = result.get("location", {}).get("lng", 0.0)
        msg.header.stamp = self.clock.now().to_msg()
        accuracy = float(result.get("accuracy", 0.0))
        msg.position_covariance = [accuracy, 0.0, 0.0, 0.0, accuracy, 0.0, 0.0, 0.0, accuracy ** 2]
        msg.position_covariance_type = 1 # COVARANCE_TYPE_APPROXIMATED
        self.pub_fix.publish(msg)

    def geolocate(self, scan_data):
        data = {
            "radioType": "gsm",
            "considerIp": "true",
            "wifiAccessPoints": scan_data,
        }
        try:
            r = requests.post(self.url, json = data, timeout = 3)
            return json.loads(r.content.decode("utf-8"))
        except:
            self.log.warn(traceback.format_exc())
            return None
        
    def scan(self):
        """
        Scans for Wi-Fi access points and returns signal data in a list 
        consistent with the format required for the wifiAccessPoints parameter of
        the Google geolocation API.
        """
        if self.has_nmcli:
            try:
                response = subprocess.check_output(['/usr/bin/nmcli', 'dev', 'wifi', 'rescan'], stderr = DEVNULL)
            except subprocess.CalledProcessError:
                pass
            time.sleep(1)

        accesspoints = []
        accesspoint = {}
        response = subprocess.check_output(['/sbin/iwlist', 'scanning'], stderr = DEVNULL)
        try:
            for line in response.decode('utf-8').split('\n'):
                line = line.strip()
                if line.startswith('Cell'):
                    if accesspoint != {}:
                        accesspoints.append(accesspoint)
                        accesspoint = {}
                    accesspoint['macAddress'] = line.split('Address:')[1].strip()
                    accesspoint['age'] = 0
                if 'Signal level' in line:
                    accesspoint['signalLevel'] = int(line.split('Signal level=')[1].split(' ')[0])
                if 'Channel:' in line:
                    accesspoint['channel'] = int(line.split('Channel:')[1].strip())
        except (ValueError, IndexError, TypeError):
            self.log.warn("iwlist returned strange data")
        return accesspoints

def main(args=None):
    rclpy.init(args=args)
    node = WifiGeolocationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
