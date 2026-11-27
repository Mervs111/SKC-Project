import time
import numpy as np
import m_p3dx as p3dx          #[ROBOT] m_p3dx versi robot (folder ini); sim & keyboard tidak dipakai

# ====================================================================
#  KEYBOARD CONTROL + ANTI-NABRAK, versi ROBOT JetRacer
#  Perubahan dari versi CoppeliaSim ditandai  #[ROBOT]
#
#  w = maju, s = mundur, a / d = maju sambil belok kiri / kanan, q = keluar
#  Tahan tombolnya; robot berhenti ~0.6 s setelah tombol dilepas.
# ====================================================================

V_MAJU  = 0.2     #[ROBOT] [m/s] di Coppelia 0.3; mulai pelan dulu di robot (batas driver 0.5)
V_BELOK = 0.15    #[ROBOT] [m/s] Ackermann TIDAK bisa berputar di tempat (v = 0 -> diam),
                  #        jadi a/d = maju sambil belok
W_BELOK = 0.02    # sama dengan Coppelia (robot meniru P3-DX, belokannya sama)
JARAK_AMAN = 0.5  # [m] objek lebih dekat dari ini -> gerak ke arah itu diblokir

## ------------ Main Program ----------------
print('Main Program Started')

#? Connect to the robot
clientID = p3dx.connect()                 #[ROBOT] pengganti sim.simxStart

#? Get the sensors handles
sensorsHandle = p3dx.getSensorsHandle(clientID)

#? Get the motors handles
motorsHandle = p3dx.getMotorHandle(clientID)

velRobot = [0.0, 0.0] # [m/s, rad/s]
_velRobot = [0.0, 0.0] # [m/s, rad/s]
t_print = 0.0
np.set_printoptions(precision=2)
print('w/s = maju/mundur, a/d = maju belok, q = keluar')

## ---------- Routine 100 Hz ----------------
while True:

    k = p3dx.tombol()                     #[ROBOT] pengganti keyboard.is_pressed(...)
    sensorValue = p3dx.getDistance(clientID, sensorsHandle)

    #? Keyboard control
    if k == 'w':
        velRobot = [V_MAJU, 0.0]
    elif k == 's':
        velRobot = [-V_MAJU, 0.0]
    elif k == 'a':
        velRobot = [V_BELOK, W_BELOK]     #[ROBOT] dulu [0.0, 0.02]
    elif k == 'd':
        velRobot = [V_BELOK, -W_BELOK]    #[ROBOT] dulu [0.0, -0.02]
    else:
        velRobot = [0.0, 0.0]

    # ====================================================================
    #? LOGIKA ANTI-NABRAK (COLLISION AVOIDANCE)
    #[ROBOT] Dipindah ke SETELAH keyboard. Di versi Coppelia cek ini ada
    #        sebelum keyboard, jadi velRobot langsung ditimpa lagi oleh
    #        tombol 'w' dan rem tidak pernah bekerja.
    front_sensors = sensorValue[2:6]      # depan (indeks 2, 3, 4, 5)
    rear_sensors = sensorValue[10:14]     #[ROBOT] belakang (indeks 10..13), untuk mundur
    obstacle_front = any(dist <= JARAK_AMAN for dist in front_sensors)
    obstacle_rear = any(dist <= JARAK_AMAN for dist in rear_sensors)

    blocked = ''
    if obstacle_front and velRobot[0] > 0.0:
        velRobot = [0.0, 0.0]
        _velRobot = [min(_velRobot[0], 0.0), 0.0]   #[ROBOT] rem langsung, tanpa menunggu filter
        blocked = 'AWAS ADA OBJEK DI DEPAN! Silakan mundur (s).'
    if obstacle_rear and velRobot[0] < 0.0:
        velRobot = [0.0, 0.0]
        _velRobot = [max(_velRobot[0], 0.0), 0.0]
        blocked = 'AWAS ADA OBJEK DI BELAKANG! Silakan maju.'
    # ====================================================================

    #? Acceleration and deceleration
    _velRobot[0] = 0.1*velRobot[0] + 0.9*_velRobot[0]
    _velRobot[1] = 0.1*velRobot[1] + 0.9*_velRobot[1]

    #? Robot kinematics
    p3dx.robotWheelKine(clientID, motorsHandle, _velRobot)   #[ROBOT] pakai yang sudah difilter

    #? Print sensor 2 to 5 (10x per detik supaya terminal terbaca)
    if time.monotonic() - t_print > 0.1:
        t_print = time.monotonic()
        print('depan', sensorValue[2:6], ' v = %.2f, w = %.3f  %s' % (_velRobot[0], _velRobot[1], blocked))

    #? Press q to quit
    if k == 'q':
        break
    time.sleep(0.01)


p3dx.disconnect(clientID)                 #[ROBOT] robot berhenti; dulu `sim.simxFinish` (tanpa kurung, tidak terpanggil)
print('Main Program Ended')
