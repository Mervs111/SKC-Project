import sim
import sys
import numpy as np
import keyboard
import time
import m_p3dx as p3dx

state_control = 0

def connect():
    sim.simxFinish(-1)
    clientID=sim.simxStart('127.0.0.1',1999,True,True,5000,5)
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect')
    return clientID

def object_distNor_estimate(sensor, side='front'):
    if side == 'front':
        # Depan: Jarak dari sensor tengah (3 & 4)
        d = min(sensor[4], sensor[3])
        # Arah: Sensor Kiri (5) dikurangi Sensor Kanan (2)
        q = sensor[5] - sensor[2]
    else:
        # Belakang: Jarak dari sensor tengah (11 & 12)
        d = min(sensor[11], sensor[12])
        # Arah: Sensor Kiri (10) dikurangi Sensor Kanan (13)
        q = sensor[10] - sensor[13]
    return d, q

def kontroler(d, q, side='front'):
    v_max = 0.3
    w_max = 0.2

    # Parameters
    K1 = -0.5
    K2 = -0.01

    dRef = 0.5
    QRef = 0.0

    # Kalkulasi dasar (dibuat seolah-olah maju)
    v = K1 * (dRef - d)
    w = K2 * (QRef - q)
    
    # Jika mode mengikuti dari belakang
    if side == 'back':
        v = -v  # Kecepatan linear dibalik agar mundur mengejar objek
        w = -w  # Kecepatan sudut dibalik agar EKOR robot yang berbelok ke objek

    # Saturation
    if v > v_max: v = v_max
    elif v < -v_max: v = -v_max

    if w > w_max: w = w_max
    elif w < -w_max: w = -w_max

    return v, w

def keyboard_control():
    ret_velocity = [0.0, 0.0]
    if keyboard.is_pressed('a'):
        ret_velocity = [0.0, 0.02]
    elif keyboard.is_pressed('d'):
        ret_velocity = [0.0, -0.02]
    elif keyboard.is_pressed('w'):
        ret_velocity = [0.1, 0.0]
    elif keyboard.is_pressed('s'):
        ret_velocity = [-0.1, 0.0]
    return ret_velocity

## ------------ Main Program ----------------
print('Main Program Started')
clientID = connect()
sensorsHandle = p3dx.getSensorsHandle(clientID)
motorsHandle = p3dx.getMotorHandle(clientID)

velRobot = [0.0, 0.0] 
_velRobot = [0.0, 0.0] 

np.set_printoptions(precision=2)

## ---------- Routine 100 Hz ----------------
while True:
    sensorValue = p3dx.getDistance(clientID, sensorsHandle)
    
    # Ambil estimasi untuk kedua sisi
    d_front, q_front = object_distNor_estimate(sensorValue, 'front')
    d_back, q_back = object_distNor_estimate(sensorValue, 'back')

    # Pemilihan mode kontrol secara berurutan dan rapi
    if state_control == 1:
        velRobot[0], velRobot[1] = kontroler(d_front, q_front, 'front')
        d, q = d_front, q_front
    elif state_control == 2:
        velRobot[0], velRobot[1] = kontroler(d_back, q_back, 'back')
        d, q = d_back, q_back
    else:
        velRobot = keyboard_control()
        d, q = d_front, q_front # Default display saja saat manual

    # Logika Toggle Tombol
    if keyboard.is_pressed('c'):
        state_control = 0 if state_control == 1 else 1
        time.sleep(0.3) # Debounce
        print('Switched to Object Following (Front)' if state_control == 1 else 'Switched to Keyboard')
        
    if keyboard.is_pressed('v'):
        state_control = 0 if state_control == 2 else 2
        time.sleep(0.3) # Debounce
        print('Switched to Object Following (Back)' if state_control == 2 else 'Switched to Keyboard')

    print('State = %d, d = %.2f, q = %.2f, v = %.2f, w = %.2f' % (state_control, d, q, velRobot[0], velRobot[1]))

    # Acceleration and deceleration
    _velRobot[0] = 0.1 * velRobot[0] + 0.9 * _velRobot[0]
    _velRobot[1] = 0.1 * velRobot[1] + 0.9 * _velRobot[1]

    # Robot kinematics
    p3dx.robotWheelKine(clientID, motorsHandle, _velRobot)

    # Press q to quit
    if keyboard.is_pressed('q'):
        p3dx.robotWheelKine(clientID, motorsHandle, [0.0, 0.0])
        break
        
    time.sleep(0.01)

# Stop the simulation
sim.simxFinish(clientID)
print('Main Program Ended')