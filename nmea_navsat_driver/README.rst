nmea_navsat_driver
===============
ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver

GPS REACH RS2
============

official web: https://docs.emlid.com/reachrs2/

Operation instructions:
-----------------------
1. Emlid Reach2 is configured to connect to a hotsopt named "mapir" with the standar passwor XD to get RTK data. Notice that if no hotspot is found at startup time, the GPS will automatically set up a wifi network to be use with Emlid flow software to configure the sensor.
2. To get GPS data just connect the USB-C port to your computer, it will create a subnetwork in the 192.168.2.X domain.
3. Configure the IP of your computer to get a valid ip in this subnetwork, like for example: 192.168.2.1
4. By default the GPS uses the IP 192.168.2.15, that you can browse to see the satelites and accuracy on real time. Also to configure the device.
