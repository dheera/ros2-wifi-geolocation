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

from .wifi_scanner import WifiScanner

status = "nmcli radio wifi"
enable = "nmcli radio wifi on"
iist = "nmcli dev wifi list"

DEVNULL = open(os.devnull, 'w')

class WifiGeolocationNode(Node):
    def __init__(self, node_name = "wifi_geolocation_node"):
        super().__init__(node_name = node_name)
        self.log = self.get_logger()
        self.clock = self.get_clock()
        self.scanner = WifiScanner()

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

        self.errors_scan = 0
        self.errors_geolocation = 0

    def on_timer(self):
        scan_data = self.scanner.scan()
        if not scan_data:
            self.errors_scan += 1
            if self.errors_scan > 0:
                self.log.warn("could not scan after %d tries" % self.errors_scan)
            return
        self.errors_scan = 0

        result = self.geolocate(scan_data)
        if result is None:
            self.errors_geolocation += 1
            if self.errors_geolocation > 5:
                self.log.warn("could not geolocate after %d tries" % self.errors_geolocation)
            return

        self.errors_geolocation = 0

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
        
def main(args=None):
    rclpy.init(args=args)
    node = WifiGeolocationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
