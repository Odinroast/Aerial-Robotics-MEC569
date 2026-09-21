"""
Example script that allows a user to "push" the Crazyflie 2.x around
using your hands while it's hovering.

This examples uses the Flow and Multi-ranger decks to measure distances
in all directions and tries to keep away from anything that comes closer
than 0.2m by setting a velocity in the opposite direction.

The demo is ended by either pressing Ctrl-C or by holding your hand above the
Crazyflie.
"""
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from collections import deque

URI = uri_helper.uri_from_env(default='radio://0/98/2M/E7E7E7E7E8')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Initialize log_data with a maximum capacity of 30 items
log_data = deque(maxlen=30)

def log_stab_callback(timestamp, data, logconf):

    # Unpack the values
    F_d = data['range.front'] / 1000
    B_d = data['range.back'] / 1000
    L_d = data['range.left'] / 1000
    R_d = data['range.right'] / 1000
    U_d = data['range.up'] / 1000
    D_d = data['range.zrange'] / 1000

    # Update the global variable
    data = [F_d, B_d, L_d, R_d, U_d, D_d]
    log_data.append(data)

    # Unpack Variables and call 
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def check_box():
    # Function to measure change in Z to detect presensce of a box
    if len(log_data) < 2:
        return False

    # Extract all Z readings (index 5) from the current buffer
    z_readings = [reading[4] for reading in log_data]
    
    # Check the total delta inside the 30-reading window
    z_change = max(z_readings) - min(z_readings)
    
    if z_change >= 0.050:
        print("box detected")
        return True
        
    return False

def is_close(range):
    MIN_DISTANCE = 0.2  # m
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Define logging params
    lg_stab = LogConfig(name='Range', period_in_ms=10)
    lg_stab.add_variable('range.front', 'float')
    lg_stab.add_variable('range.back', 'float')
    lg_stab.add_variable('range.left', 'float')
    lg_stab.add_variable('range.right', 'float')
    lg_stab.add_variable('range.zrange', 'float')
    lg_stab.add_variable('range.up', 'float')
    
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # Start Logging 
        cf = scf.cf
        cf.log.add_config(lg_stab)
        lg_stab.data_received_cb.add_callback(log_stab_callback)
        lg_stab.start()

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)
        
        with MotionCommander(scf, default_height=0.5) as motion_commander:
            with Multiranger(scf) as multi_ranger:
                # Fly Forward and Yaw Around 180
                motion_commander.forward(0.5)
                time.sleep(1)
                motion_commander.turn_left(180)
                time.sleep(1)
                # Box Finding Algorithm (Box Height = 0.165)
                check_box()

                # Obstacle avoidance
                keep_flying = True
                while keep_flying:
                    VELOCITY = 0.5
                    velocity_x = 0.0
                    velocity_y = 0.0
                    if is_close(multi_ranger.front):
                        velocity_x -= VELOCITY
                    if is_close(multi_ranger.back):
                        velocity_x += VELOCITY

                    if is_close(multi_ranger.left):
                        velocity_y -= VELOCITY
                    if is_close(multi_ranger.right):
                        velocity_y += VELOCITY

                    if is_close(multi_ranger.up):
                        keep_flying = False

                    motion_commander.start_linear_motion(
                        velocity_x, velocity_y, 0)
                    time.sleep(0.1)
            # Stop Logging
            lg_stab.stop()
            print('Demo terminated!')