import sim
import sys
import numpy as np
import keyboard
import time
import m_p3dx as p3dx

def connect():
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',1999,True,True,5000,5) # Connect to CoppeliaSim

    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect')

    return clientID

## ------------ Main Program ----------------
print('Main Program Started')

#? Connect to the simulator
clientID = connect()

#? Get the sensors handles
sensorsHandle = p3dx.getSensorsHandle(clientID)

#? Get the motors handles
motorsHandle = p3dx.getMotorHandle(clientID)

#? Start the simulation
# sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

velRobot = [0.0, 0.0] # [m/s, rad/s]
_velRobot = [0.0, 0.0] # [m/s, rad/s]
## ---------- Routine 100 Hz ----------------
while True:

    sensorValue = p3dx.getDistance(clientID,sensorsHandle)
    # ====================================================================
    #? LOGIKA ANTI-NABRAK (COLLISION AVOIDANCE)
    # Ambil data sensor depan (indeks 2, 3, 4, 5)
    front_sensors = sensorValue[2:6]
    
    # Cek apakah ada benda dalam jarak 0.5 meter atau kurang
    # (Nilai 2.0 berarti aman, jadi kita cari yang <= 0.5)
    obstacle_detected = any(dist <= 0.5 for dist in front_sensors)
    
    if obstacle_detected:
        # Jika ada objek di depan, dan perintahnya adalah maju (w)
        if velRobot[0] > 0.0:
            velRobot[0] = 0.0 # Paksa kecepatan maju jadi 0 (Berhenti)
            print("AWAS ADA OBJEK! Kecepatan maju dinonaktifkan. Silakan belok atau mundur.")
    # ====================================================================
    #? Print sensor 2 to 5
    print(sensorValue[2:6])

 #? Keyboard control
    if keyboard.is_pressed('w'):
        velRobot[0] = 0.3
        velRobot[1] = 0.0
    elif keyboard.is_pressed('s'):
        velRobot[0] = -0.3
        velRobot[1] = 0.0
    elif keyboard.is_pressed('a'):
        velRobot[0] = 0.0
        velRobot[1] = 0.02
    elif keyboard.is_pressed('d'):
        velRobot[0] = 0.0
        velRobot[1] = -0.02
    else:
        velRobot[0] = 0.0
        velRobot[1] = 0.0


    #? Acceleration and deceleration
    _velRobot[0] = 0.1*velRobot[0] + 0.9*_velRobot[0]
    _velRobot[1] = 0.1*velRobot[1] + 0.9*_velRobot[1]

        #? Robot kinematics
    p3dx.robotWheelKine(clientID,motorsHandle,velRobot)

    #? Press q to quit
    if keyboard.is_pressed('q'):
        break
    time.sleep(0.01)




#? Stop the simulation
sim.simxFinish
print('Main Program Ended')