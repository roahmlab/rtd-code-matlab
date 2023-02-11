import socket
import json
import subprocess
import sys

HOST = "0.0.0.0"
PORT = 65535
ACK_MSG = bytes('OK!', 'utf-8')
RESEND_MSG = bytes('BAD', 'utf-8')

def readPacket(connection):
    buffer = connection.recv(1024)
    if not buffer:
        return None
    msg_size = int.from_bytes(buffer[0:4], byteorder='big', signed=False)
    message = buffer[4:]
    while len(message) < msg_size+1:
        buffer = connection.recv(1024)
        if not buffer:
            return None
        message = message + buffer
    checksum = message[-1]
    message = message[:-1]

    if checksum == sum(message) % 256:
        connection.send(ACK_MSG)
    else:
        connection.send(RESEND_MSG)
        return None
    return json.loads(message)

uuid = ''
pid = None

def list2str(data):
    return ' '.join('{}'.format(k) for k in data)

def returnResult(connection):
    with open("armour.out", "r") as f:
        data = list(f)
    data = [float(entry) for entry in data]
    result = {}
    result['parameters'] = data[:-1]
    result['time'] = data[-1]
    message = json.dumps(result)
    connection.send(bytes(message, 'utf-8'))

def requestHandler(request, data, connection):
    global uuid, pid
    if request == 'robot_setup':
        uuid = data['planner_id']

    if request == 'plan_trajectory':
        inputs = data['inputs']
        with open("armour.in", "w") as f:
            f.write(list2str(inputs['q_0']) + '\n')
            f.write(list2str(inputs['q_dot_0']) + '\n')
            f.write(list2str(inputs['q_ddot_0']) + '\n')
            f.write(list2str(inputs['q_des']) + '\n')
            f.write(str(inputs['num_obs']) + '\n')
            for _, vals in inputs['obs'].items():
                f.write(list2str(vals) + '\n')
        pid = subprocess.Popen(['./armour'])

    if request == 'request_result':
        if pid is not None:
            if pid.poll() == 0:
                returnResult(connection)
            else:
                connection.send(b'{}')

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind((HOST, PORT))
    print("Awaiting connection")
    while True:
        sock.listen()
        connection, address = sock.accept()
        with connection:
            print(f"Connected by {address}")
            # main server loop
            while True:
                print("Awaiting message")
                message = readPacket(connection)
                if message is None:
                    break
                #print(message['request'])
                print(message['data'])
                requestHandler(message['request'], message['data'], connection)
        uuid = ''
        pid = None

