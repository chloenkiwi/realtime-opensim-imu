#!/usr/bin/python3
import os
import time
import opensim as osim
from opensim import Vec3
import numpy as np
from helper import (
    quat2sto_single,
    convert_csv_to_list_of_packets,
    transform_data, 
)
from data_stream_client import DataStreamClient
from config_manager import Config
import argparse
from multiprocessing import Process, Queue
from scipy.spatial.transform import Rotation
import queue
import ntplib
HOSTS = [
        #  'cn.pool.ntp.org',
        #  'asia.pool.ntp.org',
        #  'cn.ntp.org.cn',
         'ntp.aliyun.com',
        #  'time.asia.apple.com',
        #  'time.cloudflare.com',
        #  'ntp.tencent.com',
        #  'time.cloudflare.com',
        #  'ntp.nict.jp',
        #  'time.nist.gov',
        #  'ntp.tuna.tsinghua.edu.cn',
        # 'ntp.neu.edu.cn',
        #  '202.120.2.101', 
        #  'ntp.sjtu.edu.cn',
        #  'ntp.fudan.edu.cn',
        #  'ntp.shu.edu.cn',
         ]


def main(args):
    config = Config(args)
    
    osim.Logger.setLevelString(config.logging_level)
    q = Queue()  # queue for quaternion data

    if not config.offline:
        online_init(args, config, q)
    else:
        offline_init(config, q)

    time_sample, data = q.get()
    for _ in range(args.start):
        time_sample, data = q.get() # calibrate myself
    if len(data["raw_data"]) != len(config.sensors):
        raise ValueError("Not the right number of sensors in the data.")
    
    print("Start calibration.")
    if config.ave_quat:
        data = average_quat(config, q)  
    data = transform_data(data)
    # head_err = compute_quat(data)
    # data = trans_quats(data, relative_quats)
    
    quat2sto_single(data, config.sensors, config.sto_filename, 0., config.rate)

    model, ikReporter = initalize_model_and_reporter(config)

    s0, ikSolver = initialize_ik(config, model)
    print("End calibration.")

    # IK solver loop
    time_s = 0
    time_stack = []
    prev_time_stamp = None
    keep_running = True
    last_val_data = None
    data_list = []
    temp_q = queue.Queue()
    if not config.offline:
        while q.qsize() > 0:
            q.get()
    while keep_running:
        try:
            keep_running, time_s, last_val_data, prev_time_stamp = \
                update(config, q, model, s0, ikSolver, time_s, time_stack, last_val_data, temp_q, prev_time_stamp, data_list)
        except KeyboardInterrupt:
            break
    result_date = time.strftime('%y%m%d_%H%M')
    if config.offline:
        savedir = os.path.join(DATA_PATH, 'offline_results', result_date)
        # savedir = offline_export_path
    else:
        savedir = os.path.join(DATA_PATH, 'results', result_date)
    os.makedirs(savedir, exist_ok=True)
    save_time(time_stack, config, savedir)
    if config.log_angle:
        save_kinematics(ikReporter, config, savedir)

def offline_init(config, q):
    packets = convert_csv_to_list_of_packets(
            str(config.offline_data_folder / config.offline_data_name)
            # offline_data_path
        )
    for idx, packet in enumerate(packets):
        # if idx < 100: continue
        q.put([0, packet])
    q.put("done")

def online_init(args, config, q):
    fields = ["Quat1", "Quat2", "Quat3", "Quat4"]
    requested_data = [[i, sensor] for sensor in fields for i in range(len(config.sensors))]
    if config.log_time:
        requested_data.append([None, "universal_time"])
    client = DataStreamClient(args.address, q, requested_data=requested_data)
    process = Process(target=client.run_forever)
    process.start()

def update(config, q, model, s0, ikSolver, time_s, time_stack, last_val_data, temp_q, prev_time_stamp,  data_list):
    dt = 1 / config.rate
    # dt_ms = int(1000 / config.rate)
    time_stamp = 0
    if not config.datadrop_comp:
        print(f'Elements in Queue: {q.qsize():05}', end="\r")
        for _ in range(100//config.rate): 
            queue_values = q.get()
            if queue_values == "done":
                return False, time_s, last_val_data, prev_time_stamp
        time_sample, data = queue_values
        data = transform_data(data)
        if config.offline:
            if 'Package' in data['raw_data'][0].keys():
                time_stamp = int(data['raw_data'][0]['Package'])
                if prev_time_stamp is not None: 
                    if time_stamp < prev_time_stamp:
                        time_stamp += 65536
                    dt = (time_stamp - prev_time_stamp)/100
        else:
            time_stamp = int(data['time_stamp_in_milliseconds'])
            if prev_time_stamp is not None: 
                dt = (time_stamp - prev_time_stamp)/1000
        if round(dt, 2) > 1 / config.rate:
            print('Data drop')
    else:
        if config.offline:
            if config.offline_padded:
                print(f'Elements in Queue: {q.qsize():05}', end="\r")
                for _ in range(100//config.rate):
                    queue_values = q.get()
                    if queue_values == "done":
                        return False, time_s, last_val_data, prev_time_stamp
                time_sample, data = queue_values
                data = transform_data(data)
            else:
                for _ in range(100//config.rate):                    
                    if not temp_q.empty():
                        print(f'Temp queue:\t {temp_q.qsize():05}', end="\r")
                        data = temp_q.get()
                    else:
                        print(f'Elements in Queue:\t {q.qsize():05}', end="\r")
                        queue_values = q.get()
                        if queue_values == "done":
                            return False, time_s, last_val_data, prev_time_stamp
                        time_sample, data = queue_values

                    time_stamp = int(data['raw_data'][0]['Package'])
                    if prev_time_stamp is not None: 
                        if time_stamp < prev_time_stamp:
                            time_stamp += 65536
                        expect_stamp = prev_time_stamp + i+1
                        # if time_stamp < expect_stamp - 2:
                        #     continue                    
                        if time_stamp > expect_stamp:
                            temp_q.put(data)
                            time_stamp = expect_stamp
                            data = last_val_data
                            print("\nData drop")
                        else: 
                            data = transform_data(data)
                    else: 
                        data = transform_data(data)
        else:
            for i in range(100//config.rate):
                if not temp_q.empty():
                    print(f'Temp queue:\t {temp_q.qsize():05}', end="\r")
                    data = temp_q.get()
                else:
                    print(f'Elements in Queue:\t {q.qsize():05}', end="\r")
                    queue_values = q.get()
                    if queue_values == "done":
                        return False, time_s, last_val_data, prev_time_stamp
                    time_sample, data = queue_values
                time_stamp = int(data['time_stamp_in_milliseconds']) # online
                
                if prev_time_stamp is not None: 
                    expect_stamp = prev_time_stamp + 10*(i+1)
                    if time_stamp > expect_stamp + 2:
                        temp_q.put(data)
                        time_stamp = expect_stamp
                        data = last_val_data
                        print("\nData drop")
                    else: 
                        data = transform_data(data)
                else: 
                    data = transform_data(data)
            
        last_val_data = data

    quat2sto_single(data, config.sensors, config.sto_filename, time_s, config.rate)
    quatTable = osim.TimeSeriesTableQuaternion(config.sto_filename)
    orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
    rowVecView = orientationsData.getNearestRow(time_s)
    ikSolver.updateOrientationData(time_s + dt, rowVecView)
    s0.setTime(time_s + dt)

    add_time = time.time()
    ikSolver.track(s0)
    time_IK = time.time() - add_time

    if config.visualize:
        try:
            model.getVisualizer().show(s0)
        except RuntimeError:
            print("It seemed that you closed the visualizer window. Quitting now.")
            return False, time_s, last_val_data, time_stamp
    try:
        model.realizeReport(s0)
    except Exception as e:
        print(e)
    # rowind = ikReporter.getTable().getRowIndexBeforeTime((t+1)*dt) # most recent index in kinematics table
    # kin_step = ikReporter.getTable().getRowAtIndex(rowind).to_numpy() # joint angles for current time step as numpy array

    time_delay = 0.
    if config.log_time:
        collect_time = float(data['custom_data']['universal_time'][0])
        
        # Delay is valid only when the acquisition clock and host clock share
        # the same time reference.
        time_delay = time.time() + time_offset - collect_time
    time_stack.append([time_s + dt, time_stamp, time_IK, time_delay])
    time_s += dt

    return True, time_s, last_val_data, time_stamp


def save_kinematics(ikReporter, config, dir):
    columns = list(ikReporter.getTable().getColumnLabels())
    columns.insert(0, 'time')
    time_col = np.array(ikReporter.getTable().getIndependentColumn())
    angle_data = ikReporter.getTable().getMatrix().to_numpy()
    data = np.column_stack((time_col, angle_data))
    header = ','.join(columns)
    np.savetxt(os.path.join(dir, config.output_filename), data, delimiter=',', header=header, comments='')

def save_time(time_stack, config, dir):
    data = time_stack
    header = 'sample time,time stamp,IK time,delay time'
    np.savetxt(os.path.join(dir, config.time_filename), data, delimiter=',', header=header, comments='')


def initialize_ik(config, model):
    quatTable = osim.TimeSeriesTableQuaternion(config.sto_filename)
    orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(
        quatTable
    )
    oRefs = osim.BufferedOrientationsReference(orientationsData)
    init_state = model.initSystem()
    mRefs = osim.MarkersReference()
    coordinateReferences = osim.SimTKArrayCoordinateReference()
    if config.visualize:
        model.setUseVisualizer(True)
    model.initSystem()
    s0 = init_state
    ikSolver = osim.InverseKinematicsSolverRT(
        model, mRefs, oRefs, coordinateReferences, config.constraint_var
    )
    ikSolver.setAccuracy = config.accuracy
    s0.setTime(0.0)
    ikSolver.assemble(s0)
    if config.visualize:  # initialize visualization
        model.getVisualizer().show(s0)
        model.getVisualizer().getSimbodyVisualizer().setShowSimTime(False) ## False?
    return s0,ikSolver

def initalize_model_and_reporter(config, head_err=0):
    visualize_init = False
    # sensor_to_opensim_rotations = Vec3(0, 0, 0)
    sensor_to_opensim_rotations = Vec3(0,head_err,0) 
    imuPlacer = osim.IMUPlacer()
    # imuPlacer.set_model_file(uncal_model_file)
    imuPlacer.set_model_file(str(config.uncal_model_filename))
    imuPlacer.set_orientation_file_for_calibration(config.sto_filename)
    imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuPlacer.set_base_imu_label(config.base_imu)
    imuPlacer.set_base_heading_axis(config.base_imu_axis)
    imuPlacer.run(visualize_init)
    model = imuPlacer.getCalibratedModel()
    # model.printToXML(model_file)
    model.printToXML(str(config.model_filename))

    # Initialize model
    coordinates = model.getCoordinateSet()
    ikReporter = osim.TableReporter()
    ikReporter.setName("ik_reporter")
    if config.log_angle:
        for coord in coordinates:
            ikReporter.addToReport(coord.getOutput("value"), coord.getName())
    model.addComponent(ikReporter)
    model.finalizeConnections()
    return model,ikReporter

def average_quat(config, q, average_time=2):
    """Average the first calibration window of quaternion samples.

    The input stream is expected to provide 100 samples per second, so the
    default two-second calibration window requires at least 200 samples after
    the samples skipped by ``--start``. Shorter offline recordings cannot be
    used with quaternion averaging enabled.
    """
    data_lists = [[] for _ in range(len(config.sensors))]
    data_list = []
    relative_quats = []

    for _ in range(100*average_time):
        time_sample, data = q.get()
        # print(data)
        for sensor in range(len(config.sensors)):
            data_lists[sensor].append([data["raw_data"][sensor][f"Quat{i+1}"] for i in range(4)])

    for sensor in range(len(config.sensors)):
        # Convert to Rotation objects
        rotations = Rotation.from_quat(data_lists[sensor])
        # Compute the mean rotation
        mean_rotation = rotations.mean()
        # Convert mean rotation to quaternion
        mean_quaternion = mean_rotation.as_quat()
        data_list.append(mean_quaternion)
        for i in range(4):
            data["raw_data"][sensor][f"Quat{i+1}"] = mean_quaternion[i]
    return data

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Estimates kinematics from IMU data using a musculoskeletal model."
    )
    parser.add_argument("--address", type=str, help="IP address (e.g., 192.168.137.1)")
    parser.add_argument(
        "--start",
        type=int,
        help="Number of incoming samples to skip before calibration (default: 100)",
        default=100,
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Full path of config file. If not supplied, it will use the default config file",
        default="config.toml"
    )
    args = parser.parse_args()

    print("Starting real-time OpenSim inverse kinematics.\n")
    time_offset = 0.
    c = ntplib.NTPClient()
    for host in HOSTS:
        try:
            response = c.request(host)                
            if response:
                print('NTP from: ' + host + '\n')
                break
        except Exception:
            pass
    if not response: 
        print('Get NTP failed. Please retry')
    ntp_time = response.tx_time
    init_time = time.time()
    print("universal time: " + time.ctime(ntp_time))
    time_offset = ntp_time - init_time

    main(args)
