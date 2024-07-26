import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# URI of the Crazyflie to connect to
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

# Initialize the low-level drivers (don't list the debug drivers)
cflib.crtp.init_drivers(enable_debug_driver=False)

def log_position(cf):
    # Setup a log configuration for position
    position_log = LogConfig(name="Position", period_in_ms=100)
    position_log.add_variable('lps.x', 'float')
    position_log.add_variable('lps.y', 'float')
    position_log.add_variable('lps.z', 'float')

    # Callback to process the received position data
    def position_callback(timestamp, data, logconf):
        x = data['lps.x']
        y = data['lps.y']
        z = data['lps.z']
        print(f"Position: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")

    position_log.data_received_cb.add_callback(position_callback)
    cf.log.add_config(position_log)
    position_log.start()

def connection_setup(link_uri):
    print(f"Connecting to {link_uri}")
    cf = Crazyflie(rw_cache='./cache')

    # Connecting to the Crazyflie
    cf.open_link(link_uri)

    try:
        # Log position data from the Loco Positioning System
        log_position(cf)

        # Keep the program alive to continuously receive and display the position data
        print("Press Ctrl-C to stop logging...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Disconnecting...")
    finally:
        # Stop the logging before disconnecting to clean up properly
        cf.log.delete_all_configs()
        cf.close_link()

# Main program
if __name__ == '__main__':
    # Connect to the Crazyflie and start logging its position
    connection_setup(URI)
