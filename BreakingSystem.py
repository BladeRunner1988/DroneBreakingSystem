import argparse
import socket
import exceptions
import time
import pymavlink.mavutil as mavutil
from dronekit import connect, VehicleMode, APIException
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import matplotlib.pyplot as plt

# New Antecedent/Consequent objects hold universe variables and membership
# functions
obstacle_distance = ctrl.Antecedent(np.arange(1, 16, 1), 'obstacle_distance')
drone_velocity = ctrl.Consequent(np.arange(0, 6, 1), 'drone_velocity')

vehicle = None


# fuzzy logic related functions starts here
def get_drone_break_control_system_simulator():
    # obstacle distance custom membership functions
    obstacle_distance['closest'] = fuzz.trimf(obstacle_distance.universe, [0, 0, 0])
    obstacle_distance['too_close'] = fuzz.trimf(obstacle_distance.universe, [0, 1, 1])
    obstacle_distance['closer'] = fuzz.trimf(obstacle_distance.universe, [1, 2, 5])
    obstacle_distance['close'] = fuzz.trimf(obstacle_distance.universe, [2, 5, 10])
    obstacle_distance['far'] = fuzz.trimf(obstacle_distance.universe, [5, 10, 15])
    obstacle_distance['too_far'] = fuzz.trimf(obstacle_distance.universe, [10, 15, 15])
    # obstacleDistance.automf(7)

    # drone velocity custom membership functions
    drone_velocity['stop'] = fuzz.trimf(drone_velocity.universe, [0, 0, 0])
    drone_velocity['slowest'] = fuzz.trimf(drone_velocity.universe, [0, 1, 2])
    drone_velocity['slow'] = fuzz.trimf(drone_velocity.universe, [1, 2, 3])
    drone_velocity['fast'] = fuzz.trimf(drone_velocity.universe, [2, 3, 4])
    drone_velocity['faster'] = fuzz.trimf(drone_velocity.universe, [3, 4, 5])
    drone_velocity['fastest'] = fuzz.trimf(drone_velocity.universe, [4, 5, 5])

    # prepare to show obstacle distance antecedent membership graph mapping
    obstacle_distance.view()
    # prepare to show drone velocity consequent membership graph mapping
    drone_velocity.view()

    # fuzzy rules
    rule1 = ctrl.Rule(obstacle_distance['too_far'], drone_velocity['fastest'])
    rule2 = ctrl.Rule(obstacle_distance['far'], drone_velocity['faster'])
    rule3 = ctrl.Rule(obstacle_distance['close'], drone_velocity['fast'])
    rule4 = ctrl.Rule(obstacle_distance['closer'], drone_velocity['slow'])
    rule5 = ctrl.Rule(obstacle_distance['too_close'], drone_velocity['slowest'])
    rule6 = ctrl.Rule(obstacle_distance['closest'], drone_velocity['stop'])

    drone_velocity_control = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
    drone_break = ctrl.ControlSystemSimulation(drone_velocity_control)

    return drone_break


# drone related functions starts here
def show_drone_attributes(vehicle):
    print "Autopilot Firmware version: %s" % vehicle.version
    print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
    print "Global Location: %s" % vehicle.location.global_frame
    print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
    print "Local Location: %s" % vehicle.location.local_frame  # NED
    print "Attitude: %s" % vehicle.attitude
    print "Velocity: %s" % vehicle.velocity
    print "GPS: %s" % vehicle.gps_0
    print "Groundspeed: %s" % vehicle.groundspeed
    print "Airspeed: %s" % vehicle.airspeed
    print "Gimbal status: %s" % vehicle.gimbal
    print "Battery: %s" % vehicle.battery
    print "EKF OK?: %s" % vehicle.ekf_ok
    print "Last Heartbeat: %s" % vehicle.last_heartbeat
    print "Rangefinder: %s" % vehicle.rangefinder
    print "Rangefinder distance: %s" % vehicle.rangefinder.distance
    print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    print "Heading: %s" % vehicle.heading
    print "Is Armable?: %s" % vehicle.is_armable
    print "System status: %s" % vehicle.system_status.state
    print "Mode: %s" % vehicle.mode.name  # settable
    print "Armed: %s" % vehicle.armed  # settable


def connect_my_copter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect

    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    try:
        # connect('REPLACE_connection_string_for_your_vehicle', heartbeat_timeout=15)
        global vehicle
        vehicle = connect(connection_string, wait_ready=True)
        while True:
            if vehicle is not None:
                break

        # print "Vehicle System status: %s" % vehicle.system_status
        # print "Vehicle version: %s" % vehicle.version
        # print "Vehicle mode: %s" % vehicle.mode
        # print "Vehicle mount status: %s" % vehicle.mount_status
        show_drone_attributes(vehicle)
        # vehicle.close()
    # Bad TCP connection
    except socket.error:
        print 'No server exists!'

    # Bad TTY connection
    # except OSError as e:
    #     print 'No serial exists!'

    # API Error
    except APIException:
        print 'Timeout!'

    # Other error
    except:
        print 'Some other error!'

    return vehicle


def change_polling_rate(vehicle):
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # Including IMU
        10,  # Polling Rate (Hz)
        1)  # Turn On
    vehicle.send_mavlink(msg)
    time.sleep(1)


def arm_and_takeoff(vehicle, target_altitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # while vehicle.gps_0.fix_type < 2:
    #     print "Waiting for GPS...:", vehicle.gps_0.fix_type
    #     time.sleep(1)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print "Reached target altitude"
            break
        time.sleep(1)


def send_global_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s, +x = north, +y = east, +z = down and vice versa
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def send_local_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # -- BITMASK -> Consider only the velocities
        0, 0, 0,  # POSITION
        velocity_x, velocity_y, velocity_z,  # VELOCITY
        0, 0, 0,  # ACCELERATIONS
        0, 0)
    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


# ****** fuzzy logic codes ******
drone_break_control_system_simulator = get_drone_break_control_system_simulator()
# Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
drone_break_control_system_simulator.input['obstacle_distance'] = 10.0

# Crunch the numbers
drone_break_control_system_simulator.compute()

print drone_break_control_system_simulator.output['drone_velocity']
drone_velocity.view(sim=drone_break_control_system_simulator)
plt.show()


# ****** drone kit codes ******
# establish connection with drone
vehicle = connect_my_copter()
# change drone polling rate
change_polling_rate(vehicle)
# arm and initiate drone takeoff
arm_and_takeoff(vehicle, 20)
# TODO: implement breaking mechanism using fuzzy logic
# TODO: fly to intended direction/place
send_local_ned_velocity(2, 0, 0, 5)
# hover drone for 5 seconds
time.sleep(5)
# TODO: land and shutdown drone
vehicle.mode = VehicleMode("LAND")
vehicle.close()