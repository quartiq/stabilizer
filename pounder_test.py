#!/usr/bin/python3
"""
Description: Test Stabilizer communication and DDS configuration.

Author: Ryan Summers
"""

import socket
import json

HOST = '10.0.16.99'
PORT = 1235

def do_request(s, request):
    """ Perform a request with the Stabilizer.

    Args:
        s: The socket to the stabilizer.
        request: The request to transmit.

    Returns:
        The received response object.
    """
    # Transform the value field.
    request['value'] = json.dumps(request['value'], separators=[',', ':']).replace('"', "'")
    data = (json.dumps(request, separators=[',', ':']) + '\n').encode('ascii')
    s.send(data)

    response = b''
    while not response.endswith(b'\n'):
        response += s.recv(1024)

    # Decode the value array
    response = json.loads(response.decode('ascii'))
    response['value'] = response['value'].replace("'", '"')
    response['value'] = json.loads(response['value'])

    return response


def read_attribute(s, attribute_name):
    """ Read an attribute on the Stabilizer device.

    Args:
        s: The socket to the stabilizer.
        attribute_name: The name of the endpoint to write to (the attribute name).

    Returns:
        The value of the attribute. May be a string or a dictionary.
    """
    request = {
        "req": "Read",
        "attribute": attribute_name,
        "value": "",
    }

    response = do_request(s, request)

    if 'code' not in response or response['code'] != 200:
        raise Exception(f'Failed to read {attribute_name}: {response}')

    return response['value']


def write_attribute(s, attribute_name, attribute_value):
    """ Write an attribute on the Stabilizer device.

    Args:
        s: The socket to the stabilizer.
        attribute_name: The name of the endpoint to write to (the attribute name).
        attribute_value: The value to write to the attribute. May be a string or a dictionary.
    """
    request = {
        "req": "Write",
        "attribute": attribute_name,
        "value": attribute_value,
    }

    response = do_request(s, request)

    if 'code' not in response or response['code'] != 200:
        raise Exception(f'Failed to write {attribute_name}: {response}')


def main():
    """ Main program entry point. """
    with socket.socket() as s:

        # Connect to the stabilizer.
        s.connect((HOST, PORT))

        # A sample configuration for an output channel.
        channel_config = {
            'attenuation': 31.5,
            'parameters': {
                'phase_offset': 0.5,
                'frequency': 100.0e6,
                'amplitude': 0.2,
                'enabled': True,
            }
        }

        # Configure OUT0 and read it back.
        write_attribute(s, "pounder/out0", channel_config)
        print('Pounder OUT0: ', read_attribute(s, "pounder/out0"))

        print('Pounder IN1: ', read_attribute(s, "pounder/in1"))
        print('Pounder OUT1: ', read_attribute(s, "pounder/out1"))

if __name__ == '__main__':
    main()
