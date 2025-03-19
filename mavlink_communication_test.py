from pymavlink import mavutil

def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"  # Same telemetry module
    baud_rate = 57600  # Same baud rate

    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established, waiting for data...")

    return telem_link

telem_link = setup_telem_connection()

while True:
    # Wait for the next GPS_RAW_INT message
    msg = telem_link.recv_match(type="GPS_RAW_INT", blocking=True)

    if msg:
        lat = msg.lat / 1e7  # Convert back to decimal degrees
        lon = msg.lon / 1e7
        alt = msg.alt / 1000  # Convert back to meters
        hdop = msg.eph  # Horizontal accuracy in cm
        vdop = msg.epv  # Vertical accuracy in cm
        vel = msg.vel / 100  # Convert to m/s
        cog = msg.cog / 100  # Convert to degrees
        satellites = msg.satellites_visible  # Number of satellites

        print(f"Received GPS Data:")
        print(f"  Latitude: {lat}")
        print(f"  Longitude: {lon}")
        print(f"  Altitude: {alt} m")
        print(f"  Accuracy: HDOP {hdop} cm, VDOP {vdop} cm")
        print(f"  Velocity: {vel} m/s")
        print(f"  Heading: {cog}°")
        print(f"  Satellites Visible: {satellites}")