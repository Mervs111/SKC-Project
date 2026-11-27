import time
import numpy as np
import m_p3dx as p3dx          #[ROBOT] m_p3dx versi robot (folder ini); sim & keyboard tidak dipakai

# ====================================================================
#  OBJECT FOLLOWER (DEPAN + BELAKANG), versi ROBOT JetRacer
#  Perubahan dari versi CoppeliaSim ditandai  #[ROBOT]
#
#  Manual : w / s = maju / mundur, a / d = maju sambil belok
#  c = ikuti objek di DEPAN (tekan lagi = kembali manual)
#  v = ikuti objek di BELAKANG (tekan lagi = kembali manual)
#  q = keluar.  Tekan c / v sekali saja (jangan ditahan).
# ====================================================================

state_control = 0

#[ROBOT] Jangkauan sonar robot 1.0 m; di luar itu nilainya 2.0 (tidak ada
#        deteksi). Di Coppelia, objek hilang membuat robot ngebut maju
#        (v jenuh 0.3 m/s); di robot lebih aman berhenti.
OBJEK_HILANG = 1.0      # [m] d lebih besar dari ini -> objek dianggap hilang -> berhenti
SONAR_BASI   = 1.0      # [s] data sonar lebih tua dari ini -> robot berhenti

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

    #[ROBOT] Objek hilang -> berhenti
    if d > OBJEK_HILANG:
        return 0.0, 0.0

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

    # [ROBOT] Catatan: robot Ackermann hanya bisa berbelok selama v != 0.
    # Saat d = dRef (v = 0) robot diam dan tidak bisa menghadap ke objek.
    return v, w

def keyboard_control(k):                    #[ROBOT] tombol dari p3dx.tombol()
    ret_velocity = [0.0, 0.0]
    if k == 'a':
        ret_velocity = [0.1, 0.02]          #[ROBOT] dulu [0.0, 0.02]: Ackermann tidak bisa putar di tempat
    elif k == 'd':
        ret_velocity = [0.1, -0.02]         #[ROBOT] dulu [0.0, -0.02]
    elif k == 'w':
        ret_velocity = [0.1, 0.0]
    elif k == 's':
        ret_velocity = [-0.1, 0.0]
    return ret_velocity

## ------------ Main Program ----------------
print('Main Program Started')
clientID = p3dx.connect()                   #[ROBOT] pengganti sim.simxStart
sensorsHandle = p3dx.getSensorsHandle(clientID)
motorsHandle = p3dx.getMotorHandle(clientID)

velRobot = [0.0, 0.0]
_velRobot = [0.0, 0.0]
t_print = 0.0

np.set_printoptions(precision=2)
print('w/s/a/d = manual, c = ikuti depan, v = ikuti belakang, q = keluar')

## ---------- Routine 100 Hz ----------------
while True:
    k = p3dx.tombol()                       #[ROBOT] pengganti keyboard.is_pressed
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
        velRobot = keyboard_control(k)
        d, q = d_front, q_front # Default display saja saat manual

    #[ROBOT] Sonar tidak datang lagi (LiDAR / jaringan mati) -> berhenti
    if state_control != 0 and p3dx.sensorAge(clientID) > SONAR_BASI:
        velRobot = [0.0, 0.0]

    # Logika Toggle Tombol
    #[ROBOT] lepasTombol() pengganti time.sleep(0.3): satu tekan = satu kali ganti mode
    if k == 'c':
        state_control = 0 if state_control == 1 else 1
        p3dx.lepasTombol()
        print('Switched to Object Following (Front)' if state_control == 1 else 'Switched to Keyboard')

    if k == 'v':
        state_control = 0 if state_control == 2 else 2
        p3dx.lepasTombol()
        print('Switched to Object Following (Back)' if state_control == 2 else 'Switched to Keyboard')

    if time.monotonic() - t_print > 0.1:    #[ROBOT] print 10x per detik
        t_print = time.monotonic()
        print('State = %d, d = %.2f, q = %.2f, v = %.2f, w = %.2f' % (state_control, d, q, velRobot[0], velRobot[1]))

    # Acceleration and deceleration
    _velRobot[0] = 0.1 * velRobot[0] + 0.9 * _velRobot[0]
    _velRobot[1] = 0.1 * velRobot[1] + 0.9 * _velRobot[1]

    # Robot kinematics
    p3dx.robotWheelKine(clientID, motorsHandle, _velRobot)

    # Press q to quit
    if k == 'q':
        p3dx.robotWheelKine(clientID, motorsHandle, [0.0, 0.0])
        break

    time.sleep(0.01)

# Stop the robot
p3dx.disconnect(clientID)                   #[ROBOT] pengganti sim.simxFinish
print('Main Program Ended')
