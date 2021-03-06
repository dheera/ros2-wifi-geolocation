# ros2-wifi-geolocation

This is a ROS2 package that performs geolocation using a Wi-Fi scan and publishes the result as a sensor_msgs/NavSatFix.
It can use either the Mozilla API (free) or the Google API (more accurate; paid; requires specifying an API key) and an
perform the scan using either `nmcli` (preferred) or `iwlist` to do the scan.

This can be useful for if you operate robots indoors and need to fetch a lat/long, or if you need an initial geolocation guess
for mapping algorithms in places where GPS does not work well, e.g. in dense cities or under artificial structures, or maybe
simply want to map/rosbag your roadtrip with a laptop that doesn't have GPS.

This is NOT intended to provide fast 1 Hz updates of geolocation. Please do NOT abuse free APIs. If you need fast updates please
use a paid API like Google and mind the charges that are associated with that.

# Quick start

## 0. Give permissions for your user to perform Wi-Fi scans

### If your system has nmcli

Put this in `/etc/polkit-1/localauthority/50-local.d/10-nmcli-wifi-scan.pkla`:

```
[Allow wi-fi scans for all users]
Identity=unix-user:*
Action=org.freedesktop.NetworkManager.wifi.scan
ResultAny=yes
ResultInactive=yes
ResultActive=yes
```

And then run:
```
sudo service polkit restart && sudo service network-manager restart
```

### If your system has only iwlist

```sudo chmod 4755 /sbin/iwlist```

## 1. Install and run the node

```
mkdir -p some_ws/src
cd some_ws/src
git clone https://github.com/dheera/ros2-wifi-geolocation
cd ..
colcon build
ros2 run wifi_geolocation wifi_geolocation_node
```

Then in another window
```
ros2 topic echo /fix
```

You might need to wait 20-40 seconds to see the first results since the default value of "interval" is set conservatively.

# Documentation

## Subscribers

None.

## Publishers

- **fix** : sensor_msgs/NavSatFix : The geolocation result.

## Parameters

- **provider** : string : "mozilla" or "google". Other values are invalid. Default "mozilla".
- **api_key** : string : API key for the provider you choose. Use your Google API key ($5 for 1000 requests) if you choose to use Google's service. Default "test" (for Mozilla's free API).
- **interval** : double : Interval in seconds between geolocation requests. Default 20.
- **consider_ip** : bool : Should IP-based geolocation be performed? Default false.

# FAQ

### This is freaky, how can you geolocate my laptop without a GPS antenna?

Thank the folks at [Mozilla](https://mozilla.org) and [Google](https://google.com) for [wardriving](https://en.wikipedia.org/wiki/Wardriving)
the planet and indexing Wi-Fi networks by their GPS location.

# Disclaimer

I designed this mainly as a fallback geolocation option for robots that I own and control, in the event of GPS failure, and for approximate indoor localization in extremely large indoor venues.

If you use a paid API I am not responsible for any charges. Set "interval" appropriately and edit the code to put hard limits on API usage if you wish.

I am not liable for any abuse of this code. Do not use it to abuse privacy or geolocate anyone without their consent.
