#!/usr/bin/env python3
import requests
import yaml
import random
import datetime
import rospy
import os
from typing import NoReturn
from cryptography_scripts import import_private, sign_message


def start_server(host: str, port: int, private_key: str, key_password) -> (
        NoReturn
):
    """Starts the http client.
    Prameters
    ---------
    host: str
    The host name of the http server
    port: int
    The port number of the server
    """
    if key_password:
        key_password = bytes(key_password, 'utf-8')
    url = host + ":" + str(port) + '/api/amd-r/subscriber'
    data = {
        'gps': {
            'lat': random.random() * (2.95 - 2.94) + 2.94,
            'lon': random.random() * (101.875 - 101.874) + 101.874,
        },
        'battery': random.random() * 100,
        'speed': random.random() * (2 - 0) + 0,
        'mission': random.random(),
        'Time': datetime.datetime.now().isoformat()
    }
    message = bytes(str(data).replace(" ", "").replace("'", '"'), 'utf-8')

    key = import_private(private_key, key_password)
    signature = sign_message(key, message)
    data['signature'] = signature.hex()
    data['name'] = 'Test AMD-R'

    post = requests.post(url, json=data)


if __name__ == '__main__':
    # https://answers.ros.org/question/109761/where-to-download-extra-files-needed-at-runtime/
    # https://answers.ros.org/question/143281/where-should-i-put-configuration-files/
    rospy.init_node("http_pub")
    config = rospy.get_param('/httppub/config')
    with open(config, 'r') as f:
        data = yaml.safe_load(f)
        key_dir = os.path.dirname(config)
        key = os.path.join(key_dir, data['private_key'])

    try:
        start_server(data['host'], data['port'], key, data['password'])
    except KeyboardInterrupt:
        pass
