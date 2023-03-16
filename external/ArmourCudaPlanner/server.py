import socket
import json
import subprocess
import sys
import threading
import os

HOST = "0.0.0.0"
PORT = 65535
ACK_MSG = bytes('OK!', 'utf-8')
RESEND_MSG = bytes('BAD', 'utf-8')
NUM_THREADS = '4'
ALL_TYPES = ['joint_position_center', 'joint_position_radius', 'control_input_radius', 'constraints', 'wrench_values', 'force_constraint_radius']

def computeMsgChk(message_bytes : bytes):
    return (sum(message_bytes) % 256).to_bytes(1, byteorder='big')

def computeMsgLenChk(message_length_bytes : bytes):
    msg_size_sum_chk = sum(message_length_bytes) % 256
    msg_size_xor_chk = 255
    for b in message_length_bytes:
        msg_size_xor_chk ^= b
    return msg_size_sum_chk.to_bytes(1, byteorder='big') \
           + msg_size_xor_chk.to_bytes(1, byteorder='big')

def drainAndRequestResend(connection):
    timeout = connection.gettimeout()
    connection.settimeout(0)
    while True:
        try:
            b = connection.recv(4096)
            if not b:
                return None
        except:
            break
    connection.settimeout(timeout)
    connection.send(RESEND_MSG)
    return False

def readPacket(connection):
    buffer = connection.recv(1024)
    if not buffer:
        return None

    if len(buffer) < 6:
        return drainAndRequestResend(connection)

    msg_size = int.from_bytes(buffer[0:3], byteorder='big', signed=False)

    # Validate size
    msg_size_chk = computeMsgLenChk(buffer[0:3])
    #msg_size_sum_chk = sum(buffer[0:3]) % 256
    if msg_size_chk[0] != buffer[3]:
        return drainAndRequestResend(connection)
    #msg_size_xor_chk = 255
    #for b in buffer[0:3]:
    #    msg_size_xor_chk ^= b
    if msg_size_chk[1] != buffer[4]:
        return drainAndRequestResend(connection)

    message = buffer[5:]
    while len(message) < msg_size+1:
        buffer = connection.recv(1024)
        if not buffer:
            return None
        message = message + buffer
    checksum = message[-1]
    message = message[:-1]

    if checksum == computeMsgChk(message)[0]:
        connection.send(ACK_MSG)
    else:
        return drainAndRequestResend(connection)
    return json.loads(message)

uuids = {}
uuidlock = threading.Lock()

#pid = None

def list2str(data):
    return ' '.join('{:.10f}'.format(k) for k in data)

def sendMessage(connection, message):
    message = bytes(message, 'utf-8')
    try:
        msg_len_bytes = len(message).to_bytes(3, byteorder='big')
    except:
        print("Message too large!")
        return False
    msg_len_chk = computeMsgLenChk(msg_len_bytes)
    msg_chk = computeMsgChk(message)
    #try .to_bytes(3, byteorder='big')

    connection.send(msg_len_bytes + msg_len_chk + message + msg_chk)
    return True

def returnResult(connection, planner_id, request_types):
    with open(f"buffer/{planner_id}.out", "r") as f:
        data = list(f)
    data = [float(entry) for entry in data]
    result = {}
    result['result'] = 'plan_complete'
    result['parameters'] = data[:-1]
    result['time'] = data[-1]
    if len(result['parameters']) == 1:
        result['parameters'] = []

    if len(request_types):
        if not isinstance(request_types, list):
            request_types = [request_types]
        result['requested_data'] = {}
    if 'ALL' in request_types:
        request_types = ALL_TYPES
    for request in request_types:
        with open(f"buffer/{planner_id}.{request}", "r") as f:
            result['requested_data'][request] = f.read()

    message = json.dumps(result)
    if not sendMessage(connection, message):
        sendMessage(connection, '{"result":"plan_failed"}')

def requestHandler(request, data, connection):
    global uuids, uuidlock
    if request == 'robot_setup':
        with uuidlock:
            uuids[data['planner_id']] = None

    if request == 'plan_trajectory':
        inputs = data['inputs']
        uuid = data['planner_id']
        with open(f"buffer/{uuid}.in", "w") as f:
            f.write(list2str(inputs['q_0']) + '\n')
            f.write(list2str(inputs['q_dot_0']) + '\n')
            f.write(list2str(inputs['q_ddot_0']) + '\n')
            f.write(list2str(inputs['q_des']) + '\n')
            f.write(str(inputs['num_obs']) + '\n')
            for _, vals in inputs['obs'].items():
                f.write(list2str(vals) + '\n')
        my_env = {**os.environ, 'OMP_NUM_THREADS': NUM_THREADS}
        pid = subprocess.Popen(['./armour', uuid], env=my_env)
        with uuidlock:
            uuids[data['planner_id']] = pid

    if request == 'request_result':
        #with uuidlock:
        pid = uuids[data['planner_id']]
        timeout = data['timeout']
        if pid is not None:
            try:
                if pid.wait(timeout) == 0:
                    returnResult(connection, data['planner_id'], data.get('type',[]))
                else:
                    sendMessage(connection,'{"result":"plan_failed"}')
            except:
                sendMessage(connection,'{"result":"plan_ongoing"}')

    if request == 'stop_plan':
        pid = uuids[data['planner_id']]
        try:
            pid.kill()
        except:
            pass

def connectionHandler(connection, address):
    with connection:
        print(f"Connected by {address}")
        # main server loop
        while True:
            print(f"Awaiting message from {address}")
            message = readPacket(connection)
            if message is None:
                break
            if not message:
                print("Received bad message!")
                continue
            print("Handling", message['request'])
            #print(message['data'])
            requestHandler(message['request'], message['data'], connection)
    print(f"Connection by {address} closed")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind((HOST, PORT))
    while True:
        print("Awaiting connection")
        sock.listen()
        connection, address = sock.accept()
        threading.Thread(target = connectionHandler, args=(connection, address)).start()
        uuid = ''
        pid = None

