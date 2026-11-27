import sim
import sys
import numpy as np
import keyboard
import time
import m_p3dx as p3dx

# ====================================================================
#  OBJECT AVOIDANCE - BRAITENBERG VEHICLE
# --------------------------------------------------------------------
#  Ide dasar: setiap sensor terhubung langsung ke roda kiri & kanan
#  dengan bobot tertentu. Semakin dekat objek -> sensor makin "aktif"
#  -> roda di sisi berlawanan diperlambat -> robot berbelok menjauh.
#
#  vLeft  = v0 + sum( wL[i] * detect[i] )
#  vRight = v0 + sum( wR[i] * detect[i] )
#
#  Sensor 0..7  = sensor depan (0 = paling kiri, 7 = paling kanan)
#  Sensor 8..15 = sensor belakang (tidak dipakai, bobot 0)
# ====================================================================

# Jarak sensor [m]
noDetectionDist = 0.5   # di atas jarak ini objek diabaikan
maxDetectionDist = 0.2  # di bawah jarak ini sensor dianggap aktif penuh

# Kecepatan dasar roda [rad/s]
v0 = 2.0

# Bobot Braitenberg (sama seperti demo Pioneer bawaan CoppeliaSim)
# Objek di KIRI (sensor 0) -> roda KANAN diperlambat banyak -> belok kanan
# Objek di KANAN (sensor 7) -> roda KIRI diperlambat banyak -> belok kiri
braitenbergL = np.array([-0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6,
                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
braitenbergR = np.array([-1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2,
                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])


def connect():
    sim.simxFinish(-1)
    clientID=sim.simxStart('127.0.0.1',1999,True,True,5000,5)
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect')
    return clientID

def sensor_activation(sensor):
    # Ubah jarak [m] menjadi tingkat aktivasi 0..1
    # (0 = tidak ada objek, 1 = objek sangat dekat)
    detect = np.zeros(16)
    for i in range(16):
        dist = sensor[i]
        if dist < noDetectionDist:
            if dist < maxDetectionDist:
                dist = maxDetectionDist
            detect[i] = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
    return detect

def braitenberg(detect):
    vLeft = v0 + np.dot(braitenbergL, detect)
    vRight = v0 + np.dot(braitenbergR, detect)
    return vLeft, vRight

## ------------ Main Program ----------------
print('Main Program Started')
clientID = connect()
sensorsHandle = p3dx.getSensorsHandle(clientID)
motorsHandle = p3dx.getMotorHandle(clientID)

np.set_printoptions(precision=2)

## ---------- Routine 100 Hz ----------------
while True:
    sensorValue = p3dx.getDistance(clientID, sensorsHandle)

    detect = sensor_activation(sensorValue)
    vLeft, vRight = braitenberg(detect)

    print('front =', sensorValue[0:8], ' vL = %.2f, vR = %.2f' % (vLeft, vRight))

    # setWheelVel: veloCmd[0] -> motor kanan, veloCmd[1] -> motor kiri
    p3dx.setWheelVel(clientID, motorsHandle, [vRight, vLeft])

    # Press q to quit
    if keyboard.is_pressed('q'):
        p3dx.setWheelVel(clientID, motorsHandle, [0.0, 0.0])
        break

    time.sleep(0.01)

# Stop the simulation
sim.simxFinish(clientID)
print('Main Program Ended')
