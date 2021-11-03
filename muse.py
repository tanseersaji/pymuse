MUSE_MAC_PREFIX = '00:55:DA:'
PRIMARY_SERVICE = '0000fe8d-0000-1000-8000-00805f9b34fb'
CHARACTERISTIC_UUID_TEMPLATE = '273e00{}-4c4d-454d-96be-f03bac821358'
EOL = '\n'
GATT_CHARACTERISTIC_IDS = {
    'SERIAL': 1,
    # packets start with a 16-bit sequence number
    # eeg signals are all 12-bit numbers concatenated together
    'SIGNAL_AUX_LEFT': 2,  #                                              p63
    'SIGNAL_TP9': 3,       # p20, p21, p22, p23, p50, p51, p52, p60, p61, p63
    'SIGNAL_FP1': 4,       # p20, p21, p22, p23, p50, p51, p52, p60, p61, p63
    'SIGNAL_FP2': 5,       # p20, p21, p22, p23, p50, p51, p52, p60, p61, p63
    'SIGNAL_TP10': 6,      # p20, p21, p22, p23, p50, p51, p52, p60, p61, p63
    'SIGNAL_AUX_RIGHT': 7, # p20            p23  p50            p60       p63
    'DRL_REF': 8,          # p20  p21  p22  p23  p50  p51  p52  p60  p61  p63
    # imu data is sent as 16-bit numbers, 3 3-axis vectors at a time
    'GYRO': 9,             # p20  p21            p50  p51       p60  p61  p63
    'ACCELEROMETER': 0xa,  # p20  p21            p50  p51       p60  p61  p63
    # battery packet format is uncertain
    'BATTERY': 0xb,        # p20  p21  p22  p23  p50  p51  p52  p60  p61  p63
    'MAGNETOMETER': 0xc,   #                        not on muse S
    'PRESSURE': 0xd,       #                        not on muse S
    'ULTRA_VIOLET': 0xe,   #                        not on muse S
    # PPG appears to be 24bit intensity data.
    'PPG_AMBIENT': 0xf,    #                     p50  p51  p52  p60  p61  p63
    'PPG_IR': 0x10,        #                     p50  p51  p52  p60  p61  p63
    'PPG_RED': 0x11,       #                     p50  p51  p52  p60  p61  p63
    # thermistor has 12-bit format, like the eeg signals
    # it looks like lower numbers indicate warmer temperature
    'THERMISTOR': 0x12,    # p20       p22       p50            p60       p63
}
GATT_CHARACTERISTIC_UUIDS = {    
    name: CHARACTERISTIC_UUID_TEMPLATE.format('%02x' % id)
    for name, id in GATT_CHARACTERISTIC_IDS.items()
}

# muse 1, not implemented, rfcomm sdp
SPP_UUID = '00001101-0000-1000-8000-00805F9B34FB'


print('importing ...')

import bt_bleak as bt
import json
import threading
import socket
import time

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

# Enable broadcasting mode
server.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Set a timeout so the socket does not block
# indefinitely when trying to receive data.
server.settimeout(0.2)

iface = bt.interfaces()[0]
print('scanning for muses ...')
devices = None
def update(devlist):
    print('UPDATE')
    global devices
    devices = [mac for mac, name in devlist if mac.startswith(MUSE_MAC_PREFIX)]
    print('found {} devices starting with {}: {}'.format(len(devices), MUSE_MAC_PREFIX, devices))
    #devices = [iface.device(mac) for mac in devices]
    #devices = [device for device in devices if device]
    #print('able to connect to {} of them: {}'.format(len(devices), devices))
    #devices = [device for device in (iface.device(mac) for mac, name in devlist if mac.startswith(MUSE_MAC_PREFIX)) if device]
    if len(devices):
        print('found')
        iface.stop_scanning()
    else:
        print('... no connectable muses yet, {} other devices ... {}'.format(len(devlist), devlist))
iface.start_scanning(update)

devices = [iface.device(mac) for mac in devices]
devices = [device for device in devices if device]
print('found {}'.format([device.info() for device in devices]))
try:
    print('connecting to {} ...'.format(devices[0]))
except:
    print("Failed")
    exit()
device = devices[0]
print('connected to {}'.format(device.info()))

# from fractions import Fraction
import json

class Bits12:
    def __init__(self, device, name):
        self._name = name
        self._characteristic = GATT_CHARACTERISTIC_UUIDS[name]
        self._gatt = device.characteristic(PRIMARY_SERVICE, self._characteristic)
        self._gatt.subscribe(self._recv)
        print("Broadcasting", name)

    def _recv(self, data : bytes):
        # First two bytes represent the sequence number which is an unsigned int.
        seq = int.from_bytes(data[0:2], 'big', signed=False)

        samples = []
        for i in range(2, len(data), 3):
            # two 12-bit values made out of three 8-bit values
            samples.append(data[i] << 4 | data[i + 1] >> 4)
            samples.append((data[i + 1] & 0xf) << 8 | data[i + 2])
        samples = [((sample - 0x800) * 125)/ 256 for sample in samples]
        
        server.sendto((json.dumps({
            'name': self._name,
            'timestamp': time.time(),
            'seq': seq,
            'samples': [float(sample) for sample in samples],
        })+EOL).encode('utf-8'), ('<broadcast>', 37020))


class Bits24:
    def __init__(self, device, name):
        self._name = name
        self._characteristic = GATT_CHARACTERISTIC_UUIDS[name]
        self._gatt = device.characteristic(PRIMARY_SERVICE, self._characteristic)
        self._gatt.subscribe(self._recv)
        print("Broadcasting", name)

    def _recv(self, data : bytes):
        seq = int.from_bytes(data[0:2], 'big', signed=False)
        samples = []
        for i in range(2, len(data), 3):
            samples.append(int.from_bytes(data[i:i+3], 'big', signed=False))

        server.sendto((json.dumps({
            'name': self._name,
            'seq': seq,
            'timestamp': time.time(),
            'samples': [float(sample) for sample in samples],
        })+EOL).encode('utf-8'), ('<broadcast>', 37020))

class Imu:
    def __init__(self, device, name, characteristic, scale):
        self._scale = scale
        self._name = name
        self._characteristic = characteristic
        self._gatt = device.characteristic(PRIMARY_SERVICE, characteristic)
        self._gatt.subscribe(self._recv)
        self._accum = []

        print("Broadcasting", name)

    def _vector(self, data : bytes):
        return [
            int.from_bytes(data[0:2], 'big', signed=True) * self._scale,
            int.from_bytes(data[2:4], 'big', signed=True) * self._scale,
            int.from_bytes(data[4:6], 'big', signed=True) * self._scale
        ]
    def _recv(self, data : bytes):
        seq = int.from_bytes(data[0:2], 'big', signed=False)
        samples = [self._vector(data[2:8]), self._vector(data[8:14]), self._vector(data[14:20])]

        server.sendto((json.dumps({
            'name': self._name,
            'seq': seq,
            'timestamp': time.time(),
            'samples': [sample for sample in samples],
        })+EOL).encode('utf-8'), ('<broadcast>', 37020))

class Accelerometer(Imu):
    def __init__(self, device):
        # result is in G's, proportion of gravity at sea level
        super().__init__(device, 'accelerometer', GATT_CHARACTERISTIC_UUIDS['ACCELEROMETER'], 1/16384)

# what model is the IMU to verify these units?
class Gyroscope(Imu):
    def __init__(self, device):
        # result may be in degrees/s
            # i'm not sure where this decimal comes from.
            # it's in the muse-js source code.  it doesn't multiply out to an integral
            # number of degrees, or units per 16-bit value, or anything.
        super().__init__(device, 'gyroscope', GATT_CHARACTERISTIC_UUIDS['GYRO'], 0.0074768)


class Telemetry:
    # DONE FOR NOW
    def __init__(self, device):
        self._gatt = device.characteristic(PRIMARY_SERVICE, GATT_CHARACTERISTIC_UUIDS['BATTERY'])
        self._gatt.subscribe(self._recv)
        print("Broadcasting", "Telemetry")

    def _recv(self, data : bytes):
        seq = int.from_bytes(data[0:2], 'big', signed=False)
        # battery is expected to contain a mv field, an adc mv field, a data enabled flag, a percentage remaining
        batt = int.from_bytes(data[2:4], 'big', signed=False)/512
        fuelgaugemv = (int.from_bytes(data[4:6], 'big', signed=False) * 10)/22
        temp = int.from_bytes(data[8:10], 'big', signed=False)
        print((json.dumps({
            'name': 'telemetery',
            'seq': seq,
            'batt': float(batt),
            'fuelgauagemv': float(fuelgaugemv),
            'temp': temp,
            'unknown': [i for i in data[10:]]
        })+EOL).encode('utf-8'), ('<broadcast>', 37020))

class Ctrl:
    def __init__(self, device):
        self._gatt = device.characteristic(PRIMARY_SERVICE, GATT_CHARACTERISTIC_UUIDS['SERIAL'])
        self._data = b''
        self._recvd = []
        self._gatt.subscribe(self._recv)
    def status(self):
        result = self.send('s')
        if 'rs' in result:
            result['running_state'] = result['rs']
            del result['rs']
        if 'ts' in result:
            result['test_mode'] = result['ts']
            del result['ts']
        result['preset'] = '%02X' % result['ps']
        del result['ps']
        result['sn']
        result['hn']
        result['id']
        result['bp']
        result['mac_address'] = result['ma']
        del result['ma']
        # hn, id, bp, ma, sn
        return result
    def version(self, ver : int):
        # rc, ap, sp, tp, hw, bn, fw, bl, pv
        # bn is an integer
        result = self.send('v' + str(ver))
        return result
    def _recv(self, data : bytes):
        data = data[1:data[0]+1]
        self._data += data
        if self._data[-1] == ord('}'):
            self._recvd.append(json.loads(self._data))
            self._resultevent.set()
            self._data = b''

    def send(self, data : str):
        print('SERIAL ->', data)
        self._resultevent = threading.Event()
        self._gatt.write(bytes([len(data) + 1, *(ord(character) for character in data), ord('\n')]))
        self._resultevent.wait()
        result = self._recvd.pop(0)
        if result['rc'] == 0:
            return result
        else:
            # i'm not sure these error code names are right
            raise ValueError('invalid command', data, result, ['FAILURE','TIMEOUT','OVERLOADED','UNIMPLEMENTED'][result['rc']-1])
    #def recv(self):
    #    return self.data.pop(0)

ctrl = Ctrl(device)

# *1   boot to headset state
# h    stop streaming / halt
# d    start streaming (handlers initialised prior to send)
# s    status / version check
# vX   version,x=1
# pXX  preset
# k    keepalive, 2.5s timeout?

#accel = bt_bluezero.Characteristic(device, PRIMARY_SERVICE, ACCELEROMETER_CHARACTERISTIC)
#accel.subscribe(lambda data: print('accel', data))
print('sending control stop command; if things freeze command sequence may need improvement')
print(ctrl.send('v1')) # version, maybe protocol version?
print(ctrl.send('h')) # stop streaming
print(ctrl.send('p63')) # preset
#print(ctrl.send('s')) # status
print(ctrl.status())
#print(ctrl.send('k')) # keepalive
#print(ctrl.send('g408c'))
#print(ctrl.send('?'))




# telemetry = Telemetry(device)
gyroscope = Gyroscope(device)
accelerometer = Accelerometer(device)
eeg = {
   name: Bits12(device, name)
   for name, _ in GATT_CHARACTERISTIC_UUIDS.items()
   if name.startswith('SIGNAL_') or name == 'DRL_REF'
}

ppg = {
    name: Bits24(device, name) 
    for name, _ in GATT_CHARACTERISTIC_UUIDS.items()
    if name.startswith('PPG_')
}

print(ctrl.send('d')) # start streaming
print("Started Broadcasting")



while True:
    try:
        print(ctrl.send('k'))
        time.sleep(1.5)
    except KeyboardInterrupt as e:
        print("Disconnecting...")
        print(ctrl.send('h'))
        del(device)
        exit()        
