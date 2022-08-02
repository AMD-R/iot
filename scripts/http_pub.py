#!/usr/bin/env python3
import requests
import yaml
import datetime
import rospy
import os
import sys
from typing import NoReturn
from cryptography_scripts import import_private, sign_message
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Float32, String


def battery_callback(data: BatteryState):
    """Callback Function for battery_percentage topic."""
    global battery
    battery = data.percentage


def gps_callback(data: NavSatFix):
    """Callback Function for iot/gps topic."""
    global gps
    gps['lat'] = data.latitude
    gps['lon'] = data.longitude


def velocity_callback(data: Float32):
    """Callback Function for iot/velocity topic."""
    global velocity
    velocity = data.data


def mission_callback(data: String):
    """Callback Function for iot/mission topic."""
    global mission
    mission = data.data


def start_server(host: str, port: int, private_key: str, key_password: str,
                 verbose: bool = True) -> NoReturn:
    """Starts the http client.
    Prameters
    ---------
    host: str
        The host name of the http server
    port: int
        The port number of the server
    private_key: str
        The private key file name
    key_password: str
        The password used to encrypt the private key
    verbose: bool
        If True node will log the message sent to the server
    """
    rate = rospy.Rate(1)

    # Setting subscriber
    rospy.Subscriber("battery_percentage", BatteryState, battery_callback)
    rospy.Subscriber("iot/gps", NavSatFix, gps_callback)
    rospy.Subscriber("iot/velocity", Float32, velocity_callback)
    rospy.Subscriber("iot/mission", String, mission_callback)

    if key_password:
        key_password = bytes(key_password, 'utf-8')
    url = host + ":" + str(port) + '/api/amd-r/subscriber'

    # Setting intitial dummy values
    global battery
    global gps
    global velocity
    global mission
    battery = 0
    gps = {
        'lat': 0,
        'lon': 0
    }
    velocity = 0
    mission = "None"

    while not rospy.is_shutdown():
        data = {
            'gps': gps,
            'battery': battery,
            'speed': velocity,
            'mission': mission,
            'Time': datetime.datetime.now().isoformat()
        }
        message = bytes(str(data).replace(" ", "").replace("'", '"'), 'utf-8')

        key = import_private(private_key, key_password)
        signature = sign_message(key, message)
        data['signature'] = signature.hex()
        data['name'] = 'Test AMD-R'

        # Logging message
        if verbose:
            rospy.loginfo("Sending: " + str(data))

        # Sending Data
        try:
            post = requests.post(url, json=data)
        except requests.ConnectionError:
            rospy.logerr("Unable to connect to server. Please Ensure the host "
                         "and port is correct or the server is up."
                         "Retrying...")
        rate.sleep()


if __name__ == '__main__':
    # https://answers.ros.org/question/109761/where-to-download-extra-files-needed-at-runtime/
    # https://answers.ros.org/question/143281/where-should-i-put-configuration-files/
    # Getting Parameters
    rospy.init_node("http_pub")
    config = rospy.get_param('/httppub/config')
    verbose = rospy.get_param('/httppub/verbose', 1)

    # Getting configuration
    try:
        with open(config, 'r') as f:
            data = yaml.safe_load(f)
            key_dir = os.path.dirname(config)
            key = os.path.join(key_dir, data['private_key'])
    except FileNotFoundError:
        rospy.logerr("Could Not Find Config File")
        sys.exit(1)
    except KeyError:
        rospy.logerr("Could not find private_key. Please make sure the config "
                     "file is configure correctly")
        sys.exit(1)

    # Starting Loop
    try:
        start_server(data['host'], data['port'], key, data['password'],
                     int(verbose))
    except KeyError:
        rospy.logerr("Invalid config file, please ensure it is configure "
                     "correctly")
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
    except ValueError:
        start_server(data['host'], data['port'], key, data['password'],
                     int(1))
