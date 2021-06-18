# ros2-wifi-geolocation

This is a ROS2 package that performs geolocation using a Wi-Fi scan and publishes the result as a sensor_msgs/NavSatFix.
It can use either the Mozilla API (free) or the Google API (more accurate; paid; requires specifying an API key) and an
perform the scan using either `nmcli` (preferred) or `iwlist` to do the scan.

If your system does NOT have `nmcli`, root permissions are needed to do a full scan with `iwlist`, so if that is the case,
you will either need to run the node as root or run `sudo chmod 4755 /sbin/iwlist` and then it should work fine. Root access
is not necessary if you have `nmcli`, which is most full Ubuntu distributions.

This can be useful for if you operate robots indoors and need to fetch a lat/long, or if you need an initial geolocation guess
for mapping algorithms in places where GPS does not work well, e.g. in dense cities or under artificial structures, or maybe
simply want to map/rosbag your roadtrip with a laptop that doesn't have GPS.

## Subscribers

None.

## Publishers

- **fix** : sensor_msgs/NavSatFix : The geolocation result.

## Parameters

- **provider** : string : "mozilla" (default) or "google". Other values are invalid.
- **api_key** : string : API key for the provider you choose. Default is "test" for Mozilla's free API. Use your Google API key ($5 for 1000 requests) if you choose to use Google's service.
- **interval** : double : Interval in seconds between geolocation requests.

# FAQ

### This is freaky, how can you geolocate my laptop without a GPS antenna?

Thank the folks at [Mozilla](https://mozilla.org) and [Google](https://google.com) for [wardriving](https://en.wikipedia.org/wiki/Wardriving)
the planet and indexing Wi-Fi networks by their GPS location.

# Disclaimer

If you use a paid API I am not responsible for any charges. Set "interval" appropriately and edit the code to put hard limits on API usage if you wish.
