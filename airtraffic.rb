#!/usr/bin/env ruby
# coding: utf-8

# Copyright (c) 2023-2025 Ralf Horstmann <ralf@ackstorm.de>
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

require 'optparse'
require 'socket'
require 'yaml'

#
# Built-in example simulation scene that defines some example aircraft and their movements
#
DEFAULT_SCENE = "---
ownship:
  { id: 'D-EZZZ', lat: 50.00, lon: 8.0, alt: 3000_ft, speed: 80_kt, vspeed: 0_fpm, direction: 180, turnrate: -1 }

traffic:
  - { id: 'D-EAAA', address: 0xaa5501, lat: 50.005, lon: 8.04, alt: 880, speed: 80, direction: 180, turnrate: 1 }
  - { id: 'D-EBBB', address: 0xaa5502, lat: 50.005, lon: 8.06, alt: 914.4_m, speed: 80.0, direction: 180, turnrate: 1 }
  - { id: 'D-ECCC', address: 0xaa5503, lat: 50.005, lon: 8.08, alt: 3130_ft, speed: 80_kt, direction: 180, turnrate: 1 }
  - { id: 'D-EDDD', address: 0xaa5504, lat: 50.005, lon: 8.10, alt: 3200.0_ft, speed: 41.146_ms, direction: 180, turnrate: 1 }
  - { id: 'D-EEEE', address: 0xaa5505, lat: 50.005, lon: 8.12, alt: 3400.0_ft, speed: 92.000_mph, direction: 180, vspeed: 0_fpm, turnrate: 1 }
  - { id: 'D-EFFF', address: 0xaa5506, lat: 49.955, lon: 8.08, alt: 3400.0_ft, speed: 148.16_kmh, direction: 000, bearingless: true, turnrate: -1 }
"

#
# Unit conversion
#
KT_TO_MS = 1852.0 / 3600.0
MPH_TO_MS = 0.4470
MS_TO_KT = 1 / KT_TO_MS
KMH_TO_MS = 1000.0 / 3600.0
FT_TO_M = 0.3048
M_TO_FT = 1 / FT_TO_M
FPM_TO_MS = 0.3048 / 60.0
MS_TO_FPM = 1 / FPM_TO_MS

def hexdump(data, indent = 0, start = 0, finish = nil, width = 16)
    ascii = ''
    counter = 0
    dump = []
    line =  ' ' * indent
    line += '%06x ' % start
    data.each_byte do |c|
        if counter >= start
            line += ' ' if (counter % 8) == 0
            line += '%02x ' % c
            ascii << (c.between?(32, 126) ? c : ?.)
            if ascii.length >= width
                line += '|' + ascii + "|"
                dump << line
                ascii = ''
                line = ' ' * indent
                line += '%06x ' % (counter + 1)
            end
        end
        throw :done if finish && finish <= counter
        counter += 1
    end rescue :done
    line += ' ' * (57 - line.length + indent) + '|' + ascii + '|'
    dump << line
    dump.join("\n")
end

#
# Aircraft model class that does distance calculation as well as
# movement when update() is called frequently.
#
class Aircraft
    attr_accessor :lat, :lon, :alt, :dist, :total, :id
    attr_accessor :speed, :direction, :bearingless
    attr_accessor :address, :nacp, :addrtype
    attr_accessor :vspeed

    EARTH_RADIUS = 6378137.0

    def initialize(lat:, lon:, alt:, speed:, direction:, id:,
                   bearingless: false, address: 0,
                   addrtype: 0, nacp: 8, vspeed: 0.0, turnrate: 0
                  )
        @lat = lat.to_f
        @lon = lon.to_f
        if alt.to_s =~ /^([0-9]+(?:\.[0-9]+)?)(?:_(m|ft))?$/
            value = $1
            unit = $2
            case unit
            when "ft"
                @alt = value.to_f * FT_TO_M
            when "m"
                @alt = value.to_f
            else
                @alt = value.to_f # default unit meters
            end
        else
            raise "invalid altitude for \"#{id}\": #{alt}"
        end
        if speed.to_s =~ /^([0-9]+(?:\.[0-9]+)?)(?:_(ms|kmh|mph|kt))?$/
            value = $1
            unit = $2
            case unit
            when "ms"
                    @speed = value.to_f
            when "kmh"
                    @speed = value.to_f * KMH_TO_MS
            when "mph"
                    @speed = value.to_f * MPH_TO_MS
            when "kt"
                    @speed = value.to_f * KT_TO_MS
            else
                    @speed = value.to_f * KT_TO_MS # default unit knots
            end
        else
            raise "invalid speed for \"#{id}\": #{speed}"
        end
        if vspeed.to_s =~ /^((?:-)?[0-9]+(?:\.[0-9]+)?)(?:_(fpm|ms))?$/
            value = $1
            unit = $2
            case unit
            when "fpm"
                    @vspeed = value.to_f * FPM_TO_MS
            when "ms"
                    @vspeed = value.to_f
            else
                    @vspeed = value.to_f # default unit m/s
            end
        else
            raise "invalid vertical speed for \"#{id}\": #{vspeed}"
        end
        @direction = direction.to_s.to_f
        @id = id
        @bearingless = bearingless
        @address = address
        @addrtype = addrtype
        @nacp = nacp
        @turnrate = turnrate
    end

    def self.from_yaml(yaml)
        new(lat: yaml['lat'],
            lon: yaml['lon'],
            alt: yaml['alt'],
            speed:  yaml['speed'],
            direction: yaml['direction'],
            id: yaml['id'],
            bearingless: yaml['bearingless'] || false,
            address: yaml['address'] || 0,
            addrtype: yaml['addrtype'] || 0,
            nacp: yaml['nacp'] || 0,
            vspeed: yaml['vspeed'] || 0.0,
            turnrate: yaml['turnrate'] || 0
           )
    end

    def to_s
        "id: #{@id}, lat: #{'%2.4f' % @lat}, lon: #{'%2.4f' % @lon}, direction: #{'%03d' % @direction}, speed: #{'%2.1f' % @speed}, vspeed: #{'%2.1f' % @vspeed} turn: #{@turnrate}"
    end

    # see https://en.wikipedia.org/wiki/Great-circle_distance
    # for calculating the distance between two geographical coordinates
    def great_circle_distance(other)
        sdlat = Math.sin(to_rad(@lat - other.lat)) / 2.0
        sdlon = Math.sin(to_rad(@lon - other.lon)) / 2.0
        squared = Math.cos(to_rad(@lat)) *
                  Math.cos(to_rad(other.lat)) *
                  sdlon**2 + sdlat**2
        central_angle = 2.0 * Math.asin(Math.sqrt(squared))
        return central_angle * EARTH_RADIUS
    end

    def distance(other)
        return great_circle_distance(other)
    end

    def move(distance, bearing)
        lat1 = to_rad(@lat) # phi
        lon1 = to_rad(@lon) # lambda
        dir  = to_rad(bearing)
        central = distance / EARTH_RADIUS
        lat2 = Math.asin(Math.sin(lat1) * Math.cos(central) + Math.cos(lat1) * Math.sin(central) * Math.cos(dir))
        lon2 = lon1 + Math.atan2(Math.sin(dir) * Math.sin(central) * Math.cos(lat1), Math.cos(central) - Math.sin(lat1) * Math.sin(lat2))
        lon2_harmonized = (lon2 + 3 * Math::PI) % (2 * Math::PI) - Math::PI;
        @lat = lat2 * 180 / Math::PI
        @lon = lon2_harmonized * 180 / Math::PI
    end

    def bearing(other)
        lat1 = to_rad(@lat)
        lon1 = to_rad(@lon)
        lat2 = to_rad(other.lat)
        lon2 = to_rad(other.lon)
        dlon = lon2 - lon1
        y = Math.sin(dlon) * Math.cos(lat2)
        x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dlon)
        return ((to_deg(Math.atan2(y, x)) + 360.0) % 360.0)
    end

    def relative_north(other)
        d = distance(other)
        b = bearing(other)
        return (Math.cos(to_rad(b)) * d)
    end

    def relative_east(other)
        d = distance(other)
        b = bearing(other)
        return (Math.sin(to_rad(b)) * d)
    end

    def relative_vertical(other)
        other.alt - @alt
    end

    def update(elapsed)
        distance_m = @speed * elapsed
        move(distance_m, @direction)
        @alt += @vspeed * elapsed
        @direction = @direction + elapsed * @turnrate
    end

    private

    def to_rad(deg)
        return deg * Math::PI / 180.0
    end

    def to_deg(rad)
        return rad * 180 / Math::PI
    end

end

#
# Collection class of own aircraft and a list of simulated
# traffic items
#
class Scene

    def initialize(ownship, traffic)
        @ownship = ownship
        @traffic = traffic
        @last = Time.now
        @mutex = Mutex.new
    end

    def update
        @mutex.synchronize do
            now = Time.now
            elapsed = now - @last
            @last = now
            @ownship.update(elapsed)
            @traffic.each do |o|
                o.update(elapsed)
            end
        end
    end

    def ownship
        @mutex.synchronize do
            return @ownship.dup
        end
    end

    def traffic
        @mutex.synchronize do
            return @traffic.map {|t|t.dup}
        end
    end
end

#
# NMEA/FLARM protocol implementation
#
class FlarmProtocol

    attr_accessor :verbose
    attr_accessor :stratux

    def initialize(scene, verbose = 0)
        @scene = scene
        @verbose = verbose
        @stratux = false
    end

    def request(req)
        case req
        when /^\$PAAVC,R,([^,]+),([^,]+)\*/
            return paavc($1, $2)
        end
        return nil
    end

    def message()
        data = []
        data << gprmc()  # essential for SkyDemon
        data << pgrmz()  # altitude
        data << gpgga()  # essential for ForeFlight
        data << gpgsa()  # essential for ForeFlight
        data << pflaa()
        data << pflau()
        return data
    end

    private

    def gprmc()
        utc = Time.now.getutc
        utctime = utc.strftime('%H%M%S.%L')[0..8]
        utcdate = utc.strftime('%d%m%y')
        latdeg = @scene.ownship.lat.floor
        latmin = (@scene.ownship.lat - latdeg) * 60
        latmod = latdeg * 100 + latmin
        londeg = @scene.ownship.lon.floor
        lonmin = (@scene.ownship.lon - londeg) * 60
        lonmod = londeg * 100 + lonmin
        ns = @scene.ownship.lat < 0 ? 'S' : 'N'
        ew = @scene.ownship.lon < 0 ? 'W' : 'E'
        # note that this record has one field more than required to match AT-01
        message = "$GPRMC,#{utctime},A," +
                  "#{'%010.5f' % latmod.abs},#{ns}," +
                  "#{'%011.5f' % lonmod.abs},#{ew}," +
                  "#{'%.1f' % (@scene.ownship.speed * MS_TO_KT)}," +
                  "#{'%.1f' % @scene.ownship.direction}," +
                  "#{utcdate},,,"
        return checksum(message)
    end

    def gpgga()
        utc = Time.now.getutc
        utctime = utc.strftime('%H%M%S.%L')[0..8]
        latdeg = @scene.ownship.lat.floor
        latmin = (@scene.ownship.lat - latdeg) * 60
        latmod = latdeg * 100 + latmin
        londeg = @scene.ownship.lon.floor
        lonmin = (@scene.ownship.lon - londeg) * 60
        lonmod = londeg * 100 + lonmin
        ns = @scene.ownship.lat < 0 ? 'S' : 'N'
        ew = @scene.ownship.lon < 0 ? 'W' : 'E'
        alt = @scene.ownship.alt
        message = "$GPGGA,#{utctime}," +
                  "#{'%010.5f' % latmod.abs},#{ns}," +
                  "#{'%011.5f' % lonmod.abs},#{ew}," +
                  "1,07,1.0,#{'%.1f' % alt},M,,,,"
        return checksum(message)
    end

    def pflau()
        rx  = "#{@scene.traffic.size}"
        tx  = 1 # transmission status, 0 = no transmission, 1 = OK
        gps = 2 # gps status, 0 = no GPS, 1 = 3d-fix on ground, 2 = 3d-fix when moving
        power = 1 # power status, 0 = under/over voltage, 1 = OK
        alarmlevel = 0 # alarm level, 0 = no alarm, 1 = low-level alarm, 2 = important alarm, 3 = urgen alarm
        relative_bearing = ''
        alarmtype = 0 # alarm type, 0 = aircraft trafic , 1 = silent aircraft alarm, 2 = aircraft alarm, 3 = obstacle alarm
        relative_vertical = '' # meters above/below
        relative_distance = '' # horizontal distance in meters
        id = ''
        message = "$PFLAU,#{rx},#{tx},#{gps},#{power},#{alarmlevel},#{relative_bearing},#{alarmtype},#{relative_vertical},#{relative_distance},#{id}"
        return checksum(message)
    end

    def pflaa()
        data = []
        @scene.traffic.each do |t|
            alarmlevel = 0
            if t.bearingless
                relative_north = @scene.ownship.distance(t).round.to_i
                relative_east = ''
            else
                relative_north = @scene.ownship.relative_north(t).round.to_i
                relative_east = @scene.ownship.relative_east(t).round.to_i
            end
            relative_vertical = @scene.ownship.relative_vertical(t)
            relative_vertical = relative_vertical < 0 ? relative_vertical.floor : relative_vertical.ceil
            idtype = 2 # 0=random, 1=ICAO, 2=FLARM
            id = t.address.to_s(16).upcase
            if @stratux and t.id
                reg = '!' + t.id.gsub('-','')
                idtype = 1
            else
                reg = ''
            end
            track = t.direction.round.to_i
            turnrate = ''
            groundspeed = t.speed.round.to_i # !m/s
            climbrate = '%.1f' % t.vspeed
            aircraft_type = 8 # aircraft
            unknown = ''
            if t.bearingless
                message = "$PFLAA," +
                          "#{alarmlevel},#{relative_north},,#{relative_vertical}," +
                          "#{idtype},#{id},,,,#{climbrate}," +
                          "#{aircraft_type},#{unknown}"
            else
                message = "$PFLAA," +
                          "#{alarmlevel},#{relative_north},#{relative_east}," +
                          "#{relative_vertical}," +
                          "#{idtype},#{id}#{reg},#{track}," +
                          "#{turnrate},#{groundspeed},#{climbrate}," +
                          "#{aircraft_type},#{unknown}"
            end

            data << checksum(message)
        end
        return data
    end

    def gpgsa
        return "$GPGSA,A,3,,,,,,,,,,,,,1.0,1.0,1.0*33\r\n"
    end

    def pgrmz
        message = "$PGRMZ,#{(@scene.ownship.alt * M_TO_FT ).round.to_i},F,2"
        return checksum(message)
    end

    def paavc(device, item)
        message = "$PAAVC,A,#{device},#{item},"
        case device
        when "AT"
            case item
            when "HWVER"
                return checksum(message + "1.0")
            when "IOBLVER"
                return checksum(message + "1.0")
            when "IOSWVER"
                return checksum(message + "1.0")
            when "SERIAL"
                return checksum(message + "23422342")
            when "SWVER"
                return checksum(message + "1.0")
            else
                return checksum(message + "NN")
            end
        end

        return nil
    end

    def checksum(nmea)
        start = 0
        if nmea[0] == '$'
            start = 1
        end
        checksum = 0
        nmea[start..-1].each_char do |c|
            oldcheck = checksum
            checksum = checksum ^ c.ord
        end
        return "#{nmea}*#{checksum.to_s(16).upcase.rjust(2, '0')}\r\n"
    end
end

#
# Threaded connection handler for NMEA/FLARM
#
class FlarmThread
    attr_accessor :protocol
    attr_accessor :verbose

    def initialize(nmea_ip, nmea_port)
        @nmea_ip = nmea_ip
        @nmea_port = nmea_port
        @thread = nil
    end

    def run
        puts "-- listening for NMEA on #{@nmea_ip}:#{@nmea_port}"
        @threads = Thread.start(@nmea_ip, @nmea_port) do | nmea_ip, nmea_port |
            listener = TCPServer.new(nmea_ip, nmea_port)
            loop do
                connection(listener.accept)
            end
        end
    end

    def join
        @thread.join if @thread
    end

    private

    def connection(socket)
        port, ip = Socket.unpack_sockaddr_in(socket.getpeername)
        peer = "#{ip}:#{port}"
        puts "-- new NMEA connection from #{peer}"
        first = true
        last = Time.now
        thread = Thread.start do
            begin
                loop do
                    if socket.wait_readable(1)
                        message = socket.read_nonblock(256)
                        response = protocol_request(peer, message)
                        if response
                            puts "-- received NMEA query from #{peer} with response" if @verbose == 1
                            socket.write(response)
                        else
                            puts "-- received NMEA query from #{peer} not understood" if @verbose == 1
                        end
                    end
                    now = Time.now
                    elapsed = now - last
                    if elapsed >= 1
                        last = now
                        message = protocol_message(peer)
                        socket.write(message.flatten.join)
                    end
                end
            rescue => e
                puts "-- closed NMEA connection to #{peer} (#{e.message})"
                if @verbose > 1
                    puts "-- details: #{e.inspect}"
                    e.backtrace.each {|line| puts "   #{line}"}
                end
            ensure
                Thread.exit
                socket.close
            end
        end
    end

    def protocol_message(peer)
        puts("-- sending NMEA  message to #{peer}") if @verbose > 0
        data = @protocol.message()
        data.flatten.each { |d| puts("   " + d) } if @verbose > 1
        return data
    end

    def protocol_request(peer, data)
        if @verbose > 1
            puts "-- received FLARM request from #{peer}"
            puts hexdump(data, 3)
        end
        response = @protocol.request(data)
        if @verbose > 1
            if response
                puts "-- sent FLARM response to #{peer}"
                puts "   \"#{response.strip}\""
            else
                puts "-- no FLARM response to #{peer}"
            end
        end
        return response
    end

end

#
# GDL90 protocol implementation
#
class Gdl90Protocol

    attr_accessor :verbose

    CRC16Table = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
    ].freeze

    def initialize(scene, verbose = 0)
        @scene = scene
        @verbose = verbose
    end

    def msg_heartbeat(mc=0x0000)
        st1 = 0x81
        st2 = 0x00
        time = Time.now.getutc
        ts = (time.hour * 3600) + (time.min * 60) + time.sec
        if (ts & 0x10000) != 0
            ts = ts & 0x0ffff
            st2 = st2 | 0x80
        end
        msg = ''
        msg << 0x00.chr
        msg << [st1, st2, ts, mc].pack("CCvv")
        packet(msg)
    end

    def msg_heartbeat_foreflight()
        msg = ''
        msg << 0x65.chr
        msg << 0x00.chr # sub ID
        msg << 0x01.chr # version
        msg << [0x0000000000000001].pack('Q>')
        msg << 'ADS-RUBY'.ljust(8, ' ')
        msg << 'ADS-RUBY'.ljust(16, ' ')
        msg << [0x00000001].pack('L>')
        packet(msg)
    end

    def msg_ownship(status: 0, addrType: 0, address: 0,
                latitude: 0.0, longitude: 0.0,
                altitude: 0, misc: 9,
                navIntegrityCat: 8, navAccuracyCat: 9,
                hVelocity: nil, vVelocity: nil, trackHeading: 0,
                emitterCat: 1, callSign: '', code: 0)
        return msg_10_20(10, status, addrType, address,
                         latitude, longitude, altitude,
                         misc, navIntegrityCat, navAccuracyCat,
                         hVelocity, vVelocity, trackHeading,
                         emitterCat, callSign, code)
    end

    def msg_ownship_altitude(altitude: 0, merit: 50, warning: false)
        msg = ''
        msg << 0x0b.chr
        altitude = (altitude * M_TO_FT / 5).round.to_i
        if altitude < 0
            altitude = (0x10000 + altitude) & 0xffff
        end
        msg << [altitude].pack('s>')

        if merit == nil
            merit = 0x7fff
        elsif merit > 32766
            merit = 0x7ffe
        end
        b = (merit & 0x7f00) >> 8
        if warning
            b = b | 0x80
        end
        msg << b.chr
        msg << (merit & 0xff).chr
        packet(msg)
    end

    def msg_traffic(status: 0, addrType: 0, address: 0,
                    latitude: 0.0, longitude: 0.0,
                    altitude: 0, misc: 9,
                    navIntegrityCat: 8, navAccuracyCat: 8,
                    hVelocity: nil, vVelocity: nil, trackHeading: 0,
                    emitterCat: 1, callSign: '', code: 0)
        return msg_10_20(20, status, addrType, address,
                         latitude, longitude, altitude,
                         misc, navIntegrityCat, navAccuracyCat,
                         hVelocity, vVelocity, trackHeading,
                         emitterCat, callSign, code)
    end


    def msg_10_20(msgid, status, addrType, address,
                  latitude, longitude, altitude,
                  misc, navIntegrityCat, navAccuracyCat,
                  hVelocity, vVelocity, trackHeading,
                  emitterCat, callSign, code)
        puts "-- gdl90 #{msgid} #{callSign.ljust(6)} #{'%2.4f' % latitude} #{'%2.4f' % longitude} #{('%4.1f' % altitude).rjust(6)} #{'%2.1f' % hVelocity} #{'%2.1f' % vVelocity} #{('%.0f' % trackHeading).rjust(3,'0')}" if @verbose > 2
        msg = ''
        msg << msgid.chr
        msg << (((status & 0xf) << 4) | addrType & 0xf).chr
        msg << pack_3(address)
        msg << pack_latlon(latitude)
        msg << pack_latlon(longitude)
        alt = ((altitude * M_TO_FT).round.to_i + 1000) / 25
        alt = 0     if alt < 0
        alt = 0xffe if alt > 0xffe
        msg <<  ((alt & 0xff0) >> 4).chr
        msg << (((alt & 0x00f) << 4) | (misc & 0x00f)).chr
        msg << (((navIntegrityCat & 0xf) << 4) | (navAccuracyCat & 0xf)).chr
        if not hVelocity
            hVelocity = 0xfff
        elsif hVelocity < 0
            hVelocity = 0
        elsif hVelocity > 0xffe
            hVelocity = 0xffe
        end
        vVelocity = vVelocity * MS_TO_FPM
        if not vVelocity
            vVelocity = 0x800
        elsif vVelocity > 32576
            vVelocity = 0x1fe
        elsif vVelocity < -32576
            vVelocity = 0xe02
        else
            vVelocity = (vVelocity / 64).round.to_i
            if vVelocity < 0
                vVelocity = (0x1000000 + vVelocity) & 0xffffff
            end
        end
        msg << (((hVelocity * MS_TO_KT).round.to_i & 0xff0) >> 4).chr
        msg << ((((hVelocity * MS_TO_KT).round.to_i & 0x00f) << 4) | ((vVelocity & 0xf00) >> 8)).chr
        msg << (vVelocity & 0x0ff).chr
        trackHeading = (trackHeading / (360.0 / 256)).round.to_i
        msg << (trackHeading & 0xff).chr
        msg << (emitterCat & 0xff).chr
        msg << callSign.ljust(8).slice(0,8)
        msg << ((code & 0xf) << 4).chr
        packet(msg)
    end

    def message()
        data = []
        data << msg_heartbeat()
        data << msg_heartbeat_foreflight()
        data << msg_ownship(latitude: @scene.ownship.lat,
                            longitude: @scene.ownship.lon,
                            altitude: @scene.ownship.alt,
                            hVelocity: @scene.ownship.speed,
                            vVelocity: @scene.ownship.vspeed,
                            trackHeading: @scene.ownship.direction,
                            callSign: @scene.ownship.id)
        data << msg_ownship_altitude(altitude: @scene.ownship.alt)
        @scene.traffic.each do |t|
            data << msg_traffic(latitude: t.lat,
                                longitude: t.lon,
                                altitude: t.alt,
                                hVelocity: t.speed,
                                vVelocity: t.vspeed,
                                trackHeading: t.direction,
                                address: t.address,
                                callSign: t.id)
        end
        data
    end

    def selftest()
        test_vectors = [
            [[0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02], [0xb3, 0x8b]],
            [[0x00, 0x81, 0x00, 0x28, 0xc9, 0x01, 0x00], [0xa6, 0x6d]],
            [[0x0b, 0x00, 0x69, 0x00, 0x32], [0x4c, 0x0d]],
            [[0x0a, 0x00, 0x00, 0x00, 0x00, 0x15, 0x76, 0x78, 0xba, 0x8d,
              0x1f, 0x03, 0xb9, 0x88, 0x00, 0x00, 0x00, 0xa8, 0x01, 0x20,
              0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00], [0x97, 0x33]],
        ]
        errors = 0
        test_vectors.each do |t|
            data = t.first.pack("C*")
            cref = t.last.pack("C*")
            csum = crc(data)
            status = (cref == csum) ? "OK  " : "FAIL"
            errors += 1 if cref != csum
        end
        if errors > 0
            puts "-- self-test GDL90 FAILED"
        end
    end

    private

    def pack_3(data)
        res = ''
        res << [(data & 0xff0000) >> 16,
                (data & 0x00ff00) >> 8,
                data & 0xff].pack('c*')
        res
    end

    def pack_latlon(l)
        l = (l * (0x800000 / 180.0)).round.to_i
        if l < 0
            l = (0x1000000 + l) & 0xffffff
        end
        pack_3(l)
    end

    def escape(msg)
        res = ''
        msg.each_byte do |b|
            if b == 0x7d or b == 0x7e
                res << 0x7d.chr
                res << (b ^ 0x20).chr
            else
                res << b.chr
            end
        end
        res
    end

    def packet(msg)
        msg = add_crc(msg)
        msg = escape(msg)
        msg.insert(0, 0x7e.chr)
        msg << 0x7e.chr
        if @verbose > 3
            puts "-- gdl message:"
            puts hexdump(msg, 3)
        end
        msg
    end

    def add_crc(msg)
        msg = msg.dup
        msg << crc(msg)
        msg
    end

    def crc(data)
        result = ''
        crc = 0
        data.each_byte do |b|
            crc = ((CRC16Table[(crc >> 8)] ^ (crc << 8) ^ b) & 0xffff)
        end
        result << (crc & 0xff).chr
        result << ((crc & 0xff00) >> 8).chr
    end
end

#
# Threaded connection handler for GDL90
#
class GdlThread
    attr_accessor :protocol
    attr_accessor :verbose

    def initialize(gdl_ip, gdl_port)
        @gdl_ip = gdl_ip
        @gdl_port = gdl_port
    end

    def run
        puts "-- sending GDL90 to #{@gdl_ip}:#{@gdl_port}"
        @thread = Thread.start(@gdl_ip, @gdl_port) do | gdl_ip, gdl_port |
            socket = UDPSocket.new
            loop do
                sleep(1)
                puts "-- sending GDL90 message to #{gdl_ip}:#{gdl_port}" if @verbose > 0
                @protocol.message().each do |m|
                    socket.send(m, 0, gdl_ip, gdl_port)
                end
            end
            socket.close
        end
    end

    def join
        @thread.join if @thread
    end
end

#
# Load simulation scene from yaml configuration
#
def setup_scene(data)
    config = YAML.safe_load(data)
    raise "missing ownship in scene" unless config['ownship']
    raise "missing traffic in scene" unless config['traffic']
    ownship = Aircraft.from_yaml(config['ownship'])
    traffic = config['traffic'].map do |aircraft|
        Aircraft.from_yaml(aircraft)
    end
    return Scene.new(ownship, traffic)
end

#
# Run simulation and handle network connections
#
def run_simulation(options)
    if options[:config]
        scene = setup_scene(File.read(options[:config]))
    else
        scene = setup_scene(DEFAULT_SCENE)
    end
    puts "-- ownship: #{scene.ownship.to_s}" if options[:verbose] > 1
    scene.traffic.each do |t|
        puts "-- traffic: #{t.to_s}" if options[:verbose] > 1
    end
    flarm_protocol = FlarmProtocol.new(scene, options[:verbose])
    flarm_protocol.stratux = options[:stratux]
    gdl90_protocol = Gdl90Protocol.new(scene, options[:verbose])
    gdl90_protocol.selftest()

    Thread.abort_on_exception = true

    threads = []
    threads << Thread.start(scene) do |scene|
        loop do
            sleep(0.1)
            scene.update
        end
    end

    if options[:gdl_ip] and options[:gdl_port]
        gdl = GdlThread.new(options[:gdl_ip], options[:gdl_port])
        gdl.protocol = gdl90_protocol
        gdl.verbose = options[:verbose]
        gdl.run
        threads << gdl
    end

    if options[:nmea_ip] and options[:nmea_port]
        flarm = FlarmThread.new(options[:nmea_ip], options[:nmea_port])
        flarm.protocol = flarm_protocol
        flarm.verbose = options[:verbose]
        flarm.run
        threads << flarm
    end

    threads.each(&:join)
end

#
# Entry point handles command line options
#
def main
    trap(:INT) do
        puts
        exit
    end

    options = {:verbose => 0, :stratux => false}
    OptionParser.new do |opts|
        opts.banner = "Usage: airtraffic.rb [options]"

        opts.on_head("")
        opts.on_head("Simulated air traffic scenario with NMEA/FLARM and GDL90 output")
        opts.on_head("")

        opts.on("-v", "--verbose", "Increase verbosity level") do |v|
            options[:verbose] += 1
        end

        opts.on("-n", "--nmea IP:PORT", "enable NMEA/FLARM listener") do |n|
            options[:nmea] = n
        end

        opts.on("-g", "--gdl IP:PORT", "enable GDL90 sender") do |g|
            options[:gdl] = g
        end

        opts.on("-c", "--config FILE",
                "load scene configuration file instead of",
                "using built-in example scene") do |c|
            options[:config] = c
        end

        opts.on("-s", "--stratux", "enable stratux mode for NMEA/FLARM") do |g|
            options[:stratux] = true
        end
    end.parse!

    if not options[:nmea] and not options[:gdl]
        puts "either --nmea or --gdl required"
        exit 1
    end

    options[:nmea_ip] = nil
    options[:nmea_port] = nil
    if options[:nmea]
        if options[:nmea] =~ /([^:]+):([^:]+)/
            if $1
                options[:nmea_ip] = $1
            else
                options[:nmea_ip] = "127.0.0.1"
            end
            if $2
                options[:nmea_port] = $2.to_i
            else
                options[:nmea_port] = 2000
            end
        else
            puts "error: invalid nmea IP/PORT"
            exit 1
        end
    end

    options[:gdl_ip] = nil
    options[:gdl_port] = nil
    if options[:gdl]
        if options[:gdl] =~ /([^:]+):([^:]+)/
            if $1
                options[:gdl_ip] = $1
            else
                options[:gdl_ip] = "127.0.0.1"
            end
            if $2
                options[:gdl_port] = $2.to_i
            else
                options[:gdl_port] = 4000
            end
        else
            puts "error: invalid GDL90 IP/PORT"
            exit 1
        end
    end

    begin
        run_simulation(options)
    rescue => e
        if options[:verbose] < 2
            puts "error: #{e.message}"
            exit(1)
        else
            raise
        end
    end
end

main()
