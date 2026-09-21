import logging
import time

# Connecting Imports 
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

# Logging Imports
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/98/2M/E7E7E7E7E8')
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Simple Function (Asynchoronous) to set parameters
def simple_param_async(scf, groupstr, namestr):
    cf = scf.cf
    full_name = groupstr+ "." +namestr
    cf.param.add_update_callback(group=groupstr, name=namestr,
                                           cb=param_stab_est_callback)
    time.sleep(1)
    cf.param.set_value(full_name,2)
    time.sleep(1)
    
# Call back function for 
def param_stab_est_callback(name, value):
    print('The crazyflie has parameter ' + name + ' set at number: ' + value)

# Simple Function (Asynchronous) to log drone data
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()

# Call back function on log recieve
def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

# Simple Function to connect to the drone
def simple_connect():
    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Logging
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    # Parameters
    group = "stabilizer"
    name = "estimator"

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # simple_connect()
        simple_log_async(scf, lg_stab)
