# This is a snapshot of the GPSInfo Msg that should become a standard 
# GPS Msg in the future versions of ROS, it can be found here:
# http://www.ros.org/doc/api/gps_common/html/msg/GPSInfo.html


Header header

# Latitude (degrees). Positive is north of equator; negative is south.
float64 latitude

# Longitude (degrees). Positive is east of prime meridian, negative south.
float64 longitude

# Altitude (meters). Positive is above reference (e.g., sea level).
float64 altitude

# UTM WGS84 coordinates: easting (meters)
float64 utm_e

# UTM WGS84 coordinates: northing (meters)
float64 utm_n

# UTM zone
string utm_zone

# Direction (degrees from north)
float64 track

# Ground speed (meters/second)
float64 speed

# Vertical speed (meters/second)
float64 climb

# Device orientation (units in degrees)
float64 pitch
float64 roll
float64 dip

# GPS time
float64 time

# Total (positional-temporal) dilution of precision
float64 gdop

# Positional (3D) dilution of precision
float64 pdop

# Horizontal dilution of precision
float64 hdop

# Vertical dilution of precision
float64 vdop

# Temporal dilution of precision
float64 tdop

# Spherical position uncertainty (meters) [epe]
float64 err

# Horizontal position uncertainty (meters) [eph]
float64 err_horz

# Vertical position uncertainty (meters) [epv]
float64 err_vert

# Track uncertainty (degrees) [epd]
float64 err_track

# Ground speed uncertainty (meters/second) [eps]
float64 err_speed

# Vertical speed uncertainty (meters/second) [epc]
float64 err_climb

# Temporal uncertainty [ept]
float64 err_time

# Satellites used in solution
uint16 satellites # Number of satellites
int32[] satellite_prn # PRN identifiers
int32[] satellite_z # Elevation of satellites
int32[] satellite_azimuth # Azimuth of satellites
int32[] satellite_snr # Signal-to-noise ratios (dB)

# Measurement status
uint16 STATUS_NO_FIX=0   # Unable to fix position
uint16 STATUS_FIX=1      # Normal fix
uint16 STATUS_DGPS_FIX=3 # Fixed with DGPS
uint16 STATUS_WAAS_FIX=5 # Fixed with WAAS
uint16 status
