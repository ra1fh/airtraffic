# Airtraffic: NMEA/FLARM and GDL90 Simulator

<div style="text-align:center"><img src="skydemon.png" /></div>

This is a command line tool to simulate a simple air traffic scenario
and send traffic information via NMEA/FLARM or GDL90 protocol to an
EFB application.

## Getting Started

### Prerequisites

 * Ruby 2.5, 2.6, 2.7 or 3.0
 * ![Ruby EventMachine](https://github.com/eventmachine/eventmachine)
 * EFB App or other NMEA/FLARM/GDL90 receiver
   * ![ForeFlight](https://www.foreflight.com),
   * ![SkyDemon](https://www.skydemon.aero/),
   * ![Enroute](https://akaflieg-freiburg.github.io/enroute/)

### Installation

    git clone https://github.com/ra1fh/airtraffic.git

## Usage

The tool simulates the aircraft position flying east, 5 planes with
full position/distance information flying south and one bearingless
aircraft flying north, such that the EFB has a lot of traffic to
announce.

### GDL90

Send GDL90 UDP packets to 10.1.1.1 port 4000:

	ruby ./airtraffic.rb --gdl 10.1.1.1:4000

GDL90 does not support bearingless traffic, so the bearingless
aircraft is shown as normal traffic.

### NMEA/FLARM
	
Open NMEA listener on port 2000:

	ruby ./airtraffic.rb --nmea 0.0.0.0:2000

The target IP is not configurable within the EFB app, so the network
has to be set up such that the EFB can connect 192.168.1.1:2000. The
network has to route to the listener via port forwarding, dedicated
WLAN or other means.

The bearingless aircraft is presented in the NMEA PFLAA record with
distance in "RelativeNorth" field and empty "RelativeEast". The only
EFB supporting bearingless aircraft is SkyDemon in NMEA/FLARM
mode.

Multiple connections are possible.

## EFB configuration notes

### SkyDemon

 * NMEA/FLARM
   * Enable the device: Setup -> Third-Party Devices -> FLARM Devices -> Air Avionics AT-1
   * Start with: Fly -> Use Air Avionics AT-1
   * SkyDemon will connect to TCP 192.168.1.1:2000
 * GDL90
   * Enable the device: Setup -> GDL90 Compatible Device
   * Start with: Fly -> Use GDL90 Compatible Device
   * Send GDL90 traffic to the IP of your mobile device, destination port 4000
 
### ForeFlight

 * NMEA/FLARM
   * ForeFlight automatically connects TCP 192.168.1.1:2000 on startup
 * GDL90
   * ForeFlight automatically opens an UDP listener on port 4000 on startup
   * Send GDL90 traffic to the IP of your mobile device, destination port 4000

### Enroute

 * Open Menu -> Information -> Traffic Receiver
 * The page explains how to connect and allows to trigger connection attempts
