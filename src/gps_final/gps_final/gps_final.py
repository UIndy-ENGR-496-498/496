import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import pynmea2  # Import the pynmea2 library for parsing NMEA sentences

class GPSListener(Node):

    def __init__(self):
        super().__init__('gps_listener')
        
        # Adjust baudrate if your GPS uses something other than 9600
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)

        # Create a publisher to publish GPS data
        self.publisher_ = self.create_publisher(String, 'gps_data', 10)

        # Set a timer to read from the GPS sensor every second
        self.timer = self.create_timer(1.0, self.read_gps_data)

        self.get_logger().info('GPSListener node started. Listening on /dev/ttyACM0...')

    def read_gps_data(self):
        # Check if there is data waiting in the buffer
        if self.serial_port.in_waiting > 0:
            # Read a line (NMEA sentence) from the GPS
            raw_data = self.serial_port.readline().decode('utf-8', errors='replace').strip()

            try:
                # Parse the NMEA sentence with pynmea2
                msg = String()

                # Attempt to parse different types of NMEA sentences
                if raw_data.startswith('$GPGGA'):
                    # Parse GPGGA sentence for latitude, longitude, and altitude
                    msg.data = self.parse_gpgga(raw_data)
                elif raw_data.startswith('$GPRMC'):
                    # Parse GPRMC sentence for time, status, latitude, longitude, speed
                    msg.data = self.parse_gprmc(raw_data)
                elif raw_data.startswith('$GPGLL'):
                    # Parse GPGLL sentence for latitude, longitude
                    msg.data = self.parse_gpgll(raw_data)
                elif raw_data.startswith('$GPVTG'):
                    # Parse GPVTG sentence for speed and course
                    msg.data = self.parse_gpvtg(raw_data)
                elif raw_data.startswith('$GPGSV'):
                    # Parse GPGSV sentence for satellite information
                    msg.data = self.parse_gpgsv(raw_data)
                else:
                    # If the sentence is not one we specifically handle, just return the raw data
                    msg.data = f"Raw NMEA Data: {raw_data}"

                # Publish the parsed data or raw data if parsing was unsuccessful
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published GPS Data: {msg.data}')
            except Exception as e:
                self.get_logger().warn(f"Failed to parse NMEA sentence: {raw_data}. Error: {e}")

    def parse_gpgga(self, raw_data):
        # Parse the GPGGA sentence
        try:
            nmea_obj = pynmea2.parse(raw_data)
            latitude = nmea_obj.latitude
            longitude = nmea_obj.longitude
            altitude = nmea_obj.altitude
            return f"GPGGA - Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude} meters"
        except pynmea2.nmea.ParseError as e:
            return f"Failed to parse GPGGA sentence: {raw_data}"

    def parse_gprmc(self, raw_data):
        # Parse the GPRMC sentence
        try:
            nmea_obj = pynmea2.parse(raw_data)
            time = nmea_obj.timestamp
            status = nmea_obj.status
            latitude = nmea_obj.latitude
            longitude = nmea_obj.longitude
            speed = nmea_obj.spd_over_grnd
            return f"GPRMC - Time: {time}, Status: {status}, Latitude: {latitude}, Longitude: {longitude}, Speed: {speed} knots"
        except pynmea2.nmea.ParseError as e:
            return f"Failed to parse GPRMC sentence: {raw_data}"

    def parse_gpgll(self, raw_data):
        # Parse the GPGLL sentence for latitude and longitude
        try:
            nmea_obj = pynmea2.parse(raw_data)
            latitude = nmea_obj.latitude
            longitude = nmea_obj.longitude
            return f"GPGLL - Latitude: {latitude}, Longitude: {longitude}"
        except pynmea2.nmea.ParseError as e:
            return f"Failed to parse GPGLL sentence: {raw_data}"

    def parse_gpvtg(self, raw_data):
        # Parse the GPVTG sentence for speed and course
        try:
            nmea_obj = pynmea2.parse(raw_data)
            course = nmea_obj.true_course
            speed_knots = nmea_obj.spd_over_grnd
            return f"GPVTG - Course: {course} degrees, Speed: {speed_knots} knots"
        except pynmea2.nmea.ParseError as e:
            return f"Failed to parse GPVTG sentence: {raw_data}"

    def parse_gpgsv(self, raw_data):
        # Parse the GPGSV sentence for satellite information
        try:
            nmea_obj = pynmea2.parse(raw_data)
            num_of_sv = nmea_obj.num_sv_in_view
            satellite_info = f"Number of satellites in view: {num_of_sv}"
            return f"GPGSV - {satellite_info}"
        except pynmea2.nmea.ParseError as e:
            return f"Failed to parse GPGSV sentence: {raw_data}"

def main(args=None):
    rclpy.init(args=args)
    gps_listener = GPSListener()
    rclpy.spin(gps_listener)
    gps_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
