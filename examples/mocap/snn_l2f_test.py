import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from pprint import pprint
from cf_utils import *
from mocap_wrapper import MocapWrapper

SNN_CONTROL = True
DO_SQUARE = False
DO_FORWARD = False
AXIS_EXPLORATION = False
GATHER_DATA = False
DO_EIGHT_FIGURE = False
# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E0')

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'cf'

# battery variables
batt_level = 0
batt_state = 0

# time variables
t_start = 0

# Log config
# logconfig = [{"name": "rltm.m1", "type": "float"},
#             #  {"name": "rltm.m2", "type": "float"},
#             #  {"name": "rltm.m3", "type": "float"},
#             #  {"name": "rltm.m4", "type": "float"},]
#             {"name": "snn_control.motor1", "type": "float"},]
logconfig = [{"name": "pm.vbat", "type": "float"}, 
              {"name": "snn_control.motor1", "type": "int8_t"}, 
              {"name": "posCtl.targetX", "type": "float"}, 
              {"name": "locSrv.x", "type": "float"}, 
              {"name": "ctrltarget.x", "type": "float"}]

def connection_failed_link_error(link_uri, msg):
    print(f"Connection to {link_uri} failed: {msg}, trying to reconnect and land")
    reconnect_and_land(link_uri)

def log_callback(timestamp, data, logconf):
    global batt_level, batt_state, t_start
    pprint(data)
    # print(f"[{time.time() - t_start:.2f}s] Batt. level: {data['pm.vbat']:0.2f}V, " + \
    #       f"state: {data['pm.state']}, " + \
    #       f"target: {data['posCtl.targetX']:0.3f}, " + \
    #       f"locSrv: {data['locSrv.x']:0.3f}, " + \
    #       f"ctrltarget: {data['ctrltarget.x']:0.3f}")
    # batt_level = data["pm.vbat"]
    # batt_state = data["pm.state"]

def stop_logconfig(logconfig):
    logconfig.stop()
def do_square(x,y,z, yaw, commander):
    commander.go_to(x+0.4, y+0.4, z, yaw, 1)
    time.sleep(2)
    # commander.land(0.0, 4.0)
    commander.go_to(x-0.4, y+0.4, z, yaw, 1)
    time.sleep(2.0)
    commander.go_to(x-0.4, y-0.4, z, yaw, 1)
    time.sleep(2.0)
    commander.go_to(x+0.4, y-0.4, z, yaw, 1)
    time.sleep(2.0)
    commander.go_to(x, y, z, yaw, 1)
    time.sleep(2.0)

def gather_data(x,y,z, yaw, commander):
    do_square(x, y, z, yaw, commander)
    time.sleep(1)
    do_square(x, y, z, yaw, commander)
    time.sleep(1)
    do_square(x, y, z, yaw, commander)
    time.sleep(1)
    do_square(x, y, z, yaw, commander)
    time.sleep(1)
    # do_square(x, y, z, yaw, commander)
    # time.sleep(1)
    # do_square(x, y, z, yaw, commander)
    # time.sleep(1)
    # do_square(x, y, z, yaw, commander)
    # time.sleep(1)

import numpy as np
def do_eight_figure(x,y,z, yaw, commander):
    # x = a*np.sin(t)
    # y = b*np.sin(t)*np.cos(t)
    # z = 1
    a = 1
    b = 1
    T = 10 # period in sec
    dt = 0.02/T
    t = np.arange(0,np.pi,step=dt)
    length = len(t)
    inter = 0.1
    for i in range(int(len(t))):
        x = a*np.sin(t[i])
        y = b*np.sin(t[i])*np.cos(t[i])
        print(x,y)
        if t>inter:
            inter += .1
            commander.go_to(x, y, z, yaw, 0.1)
            
            time.sleep(0.1)
        
    deactivate_snn_controller(cf)
    commander.go_to(x, y, 1.5, yaw, .2)
    



def do_axis_exploration(x,y,z, yaw, commander):
    commander.go_to(x+1, y, z, yaw, 1)
    time.sleep(5)
    commander.go_to(x, y, z, yaw, 1)
    # commander.land(0.0, 4.0)
    time.sleep(4.0)
    commander.go_to(x, y+1, z, yaw, 1)
    time.sleep(4.0)
    commander.go_to(x, y, z, yaw, 1)
    time.sleep(4.0)
    commander.go_to(x, y, z+1, yaw, 1)
    time.sleep(4.0)
    commander.go_to(x, y, z, yaw, 1)
    time.sleep(4.0)
    commander.go_to(x, y, z, yaw+90, 1)
    time.sleep(4.0)
    commander.go_to(x, y, z, yaw, 1)

def do_forward(x,y,z, yaw, commander):
    commander.go_to(x+.5, y, z, yaw, .5)
    
    time.sleep(2)
    # commander.land(0.0, 4.0)
    commander.go_to(x, y, z, yaw, .5)
    time.sleep(.5)
def run_sequence(cf):
    global batt_level, batt_state, t_start

    # Starting position
    x = 0
    y = 0
    z = 1.
    
    yaw = 0

    commander = cf.high_level_commander
    setpoint = cf.commander.send_setpoint
    deactivate_snn_controller(cf)
    start_onboard_logging(cf)
    t_start = time.time()
    print("Takeoff")
    commander.takeoff(1., 2)
    time.sleep(5)
    
    print("Hovering...")
    # commander.go_to(x, y, z, 90, 3)
    # commander.send_zdistance_setpoint(20,0,0,1)
    # time.sleep(1)
    # commander.send_zdistance_setpoint(0,0,0,1)
    # commander.go_to(x, y, z, -90, 3)
    if not SNN_CONTROL:
        if DO_SQUARE:
            do_square(x, y, z, yaw, commander)
        elif DO_FORWARD:
            do_forward(x, y, z, yaw, commander)
        elif AXIS_EXPLORATION:
            do_axis_exploration(x, y, z, yaw, commander)
        elif GATHER_DATA:
            gather_data(x, y, z, yaw, commander)
            time.sleep(1)
            # gather_data(x, y, z, yaw, commander)
            time.sleep(1)
        elif DO_EIGHT_FIGURE:
            do_eight_figure(x, y, z, yaw, commander)
            time.sleep(0.1)
            commander.go_to(x, y, z, yaw, .5)
            time.sleep(0.2)
            do_eight_figure(x, y, z, yaw, commander)
            time.sleep(0.1)
            commander.go_to(x, y, z, yaw, .5)
            time.sleep(0.2)
            do_eight_figure(x, y, z, yaw, commander)
            time.sleep(0.1)
            commander.go_to(x, y, z, yaw, .5)
            time.sleep(0.2)
            do_eight_figure(x, y, z, yaw, commander)
            time.sleep(0.1)
            commander.go_to(x, y, z, yaw, .5)
            time.sleep(0.2)
            do_eight_figure(x, y, z, yaw, commander)
            time.sleep(0.1)
            commander.go_to(x, y, z, yaw, .5)
            time.sleep(0.2)
            do_eight_figure(x, y, z, yaw, commander)
            time.sleep(0.1)
            commander.go_to(x, y, z, yaw, .5)
            time.sleep(0.2)

    # time.sleep(
    # commander.go_to(x, y, z, 90, 3)   
    
    t_start = time.time()
    
    # input("Press Enter to continue...")

    if SNN_CONTROL:
        
        print("Preparing to activate SNN controller")
        # Ensure we're at the hover position before switching
        commander.go_to(x, y, z, yaw, 0.5)
        time.sleep(0.5)
        
        # Set I-gain before activation to prevent integration windup
        set_snn_I_gain(cf, 0.0)
        time.sleep(0.1)
        
        print("Activating SNN controller")
        activate_snn_controller(cf)
        time.sleep(0.5)  # Give it time to stabilize
        if DO_SQUARE:
            do_square(x, y, z, yaw, commander)
        elif DO_FORWARD:
            do_forward(x, y, z, yaw, commander)
            time.sleep(.1)
            deactivate_snn_controller(cf)
        elif AXIS_EXPLORATION:
            do_axis_exploration(x, y, z, yaw, commander)
        elif GATHER_DATA:
            gather_data(x, y, z, yaw, commander)
            time.sleep(1)
            # gather_data(x, y, z, yaw, commander)
            time.sleep(1)
        elif DO_EIGHT_FIGURE:
            do_eight_figure(x, y, z, yaw, commander)
            time.sleep(0.1)
            deactivate_snn_controller(cf)
            commander.go_to(x, y, z, yaw, .5)
            
            
        else:
            time.sleep(5)
        # if KeyboardInterrupt:
        #     deactivate_snn_controller(cf)
        #     commander.land(0.0, 4.0)
        #     stop_onboard_logging(cf)

        # time.sleep(5)
        
        deactivate_snn_controller(cf)
        print("You didn't crash! :)")
        stop_onboard_logging(cf)



    print("Landing...")
    commander.land(0.0, 4.0)
    time.sleep(2.5)
    stop_logconfig(log_config)
    time.sleep(1.0)
    print("Stopping motors")
    stop_onboard_logging(cf)

    commander.stop()

    print("Disconnecting...")
    cf.close()


if __name__ == '__main__':
    print("initializing drivers")
    cflib.crtp.init_drivers()

    print("Initializing MocapWrapper")
    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)

    print("Connect to the Crazyflie")
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            cf.connection_lost.add_callback(connection_failed_link_error)
            print("Adding log config")
            log_config = add_logconfig(cf, logconfig, log_callback, freq=2000)
            # log_config = add_logconfig(cf)
#
            # Set up a callback to handle data from the mocap system
            mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

            print("Activating the kalman estimator")
            activate_kalman_estimator(cf)
            reset_estimator(cf)
            start_onboard_logging(cf)
            # time.sleep(5)
            # get list of parameters of group
            # get_group_list(cf)
            # get_param_list(cf, "rlt")
            # input("Press Enter to continue...")
            run_sequence(cf)
            
            # time.sleep(1.0)
            stop_onboard_logging(cf)

            stop_logconfig(log_config)
            input("Press Enter to continue...")
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            cf.high_level_commander.land(0,3)
            cf.high_level_commander.stop()

    mocap_wrapper.close()