import time

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
ORIENTATION_STD_DEV = 4.5e-3

SEND_FULL_POSE = True


def add_logconfig(cf, variables, callback, freq=1000):
    log_config = LogConfig(name='Logging', period_in_ms=freq)
    for v in variables:
        log_config.add_variable(v['name'], v['type'])
    log_config.data_received_cb.add_callback(callback)
    cf.log.add_config(log_config)
    log_config.start()
    return log_config

def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', ORIENTATION_STD_DEV)

def activate_snn_controller(cf):
    # first warm up
    # cf.param.set_value('snn_ct.warmUp', '1')
    cf.param.set_value('snn_mc.use_snn', '1')
    cf.param.set_value('snn_ct.snnType', '0')
    time.sleep(0.5)
    # cf.param.set_value('snn_ct.warmUp', '0')

    time.sleep(0.1)

def deactivate_snn_controller(cf):
    cf.param.set_value('snn_mc.use_snn', '0')
    
    time.sleep(0.1)

def set_hover_mode_learned(cf):
    cf.param.set_value(cf, "rlt.trigger", 0) # setting the trigger mode to the custom command (cf. https://github.com/arplaboratory/learning_to_fly_controller/blob/0a7680de591d85813f1cd27834b240aeac962fdd/rl_tools_controller.c#L80)
    cf.param.set_value(cf, "rlt.wn", 1)
    cf.param.set_value(cf, "rlt.motor_warmup", 1)
    cf.param.set_value(cf, "rlt.target_z", 2)
    input("Press enter to start hovering")
    prev = time.time()
    acc = 0
    cnt = 0
    while True:
        i = input("Hold enter to fly")
        if i == "q":
            break
        current = time.time()
        acc += current - prev
        cnt += 1
        if cnt % 100 == 0:
            print(f"Average rate: {1/(acc / cnt):.3f}Hz")
            acc = 0
            cnt = 0
        prev = current
        send_learned_policy_packet(cf)

def set_snn_I_gain(cf, gain):
    cf.param.set_value('pid_rate.snnIGain', str(gain))
    time.sleep(0.1)

def start_onboard_logging(cf):
    cf.param.set_value("usd.logging", "1")
    time.sleep(0.05)

def stop_onboard_logging(cf):
    cf.param.set_value("usd.logging", "0")
    time.sleep(0.05)

def get_group_list(cf):
    """
    Get the list of groups that are available for the current firmware.
    """
    print("Available groups:")
    for group in cf.param.values.keys():
        print(group)
    time.sleep(1)
    
def get_param_list(cf, group):
    """
    Read a value for the supplied parameter. This can block for a period
    of time if the parameter values have not been fetched yet.
    """
    params_group = cf.param.values[group]
    for key in params_group.keys():
        print(f"{key}: {params_group[key]}")
    time.sleep(1) 

def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    trajectory_mem.write_data_sync()
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                print("Kalman filter converged.")
                break

def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(0.1)

    wait_for_position_estimator(cf)



def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if SEND_FULL_POSE:
        cf.extpos.send_extpose(z, x, y, quat.z, quat.x, quat.y, quat.w)
        # cf.extpos.send_extpose(-y, x, z, -quat.y, quat.x, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(z, x, y)
        # print(-y, x, z)
        # cf.extpos.send_extpos(-y, x, z)

def reconnect_and_land(uri):
    start_time = time.time()
    duration = 10
    while time.time() - start_time < duration:
        print("Try to reconnect")
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print("Recovered connection and stopping propellors")
                cf = scf.cf
                cf.high_level_commander.stop()
        except Exception as e:
            print("Connection failed: ", e)
            time.sleep(1)