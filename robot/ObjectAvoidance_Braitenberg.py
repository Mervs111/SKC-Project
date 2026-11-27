import time
import numpy as np
import m_p3dx as p3dx          #[ROBOT] m_p3dx versi robot (folder ini); sim & keyboard tidak dipakai

# ====================================================================
#  OBJECT AVOIDANCE - BRAITENBERG VEHICLE, versi ROBOT JetRacer
#  Perubahan dari versi CoppeliaSim ditandai  #[ROBOT]
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
#
#  [ROBOT] Mobil Ackermann tidak bisa berputar di tempat. Di depan dinding
#  atau di pojok, Braitenberg memberi vLeft ~ vRight ~ 0 atau vLeft ~ -vRight
#  (P3-DX berputar di tempat), tapi robot ini malah DIAM selamanya. Karena itu
#  ditambah "mundur sambil belok": kalau kecepatan maju dari Braitenberg
#  < V_MACET, atau objek < JARAK_MUNDUR di depan, robot mundur sebentar sambil
#  memutar moncongnya menjauhi objek (seperti putar balik 3 langkah), lalu
#  Braitenberg jalan lagi.
# ====================================================================

# Jarak sensor [m]
noDetectionDist = 0.5   # di atas jarak ini objek diabaikan
maxDetectionDist = 0.2  # di bawah jarak ini sensor dianggap aktif penuh

# Kecepatan dasar roda [rad/s]
v0 = 2.0                # = 0.0975 * 2.0 = 0.2 m/s

# Bobot Braitenberg (sama seperti demo Pioneer bawaan CoppeliaSim)
# Objek di KIRI (sensor 0) -> roda KANAN diperlambat banyak -> belok kanan
# Objek di KANAN (sensor 7) -> roda KIRI diperlambat banyak -> belok kiri
braitenbergL = np.array([-0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6,
                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])
braitenbergR = np.array([-1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2,
                          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0])

#[ROBOT] Mundur sambil belok
V_MACET      = 0.03     # [m/s] kecepatan maju Braitenberg di bawah ini -> robot macet -> mundur
JARAK_MUNDUR = 0.15     # [m] objek di depan (sensor 2..5) lebih dekat dari ini -> mundur
MUNDUR_DETIK = 1.5      # [s] lama mundur
KUNCI_DETIK  = 3.0      # [s] mundur lagi dalam waktu ini -> arah belok sama dengan sebelumnya
                        #     (tanpa ini robot maju-mundur kiri-kanan terus di pojok)
PHI_MUNDUR   = 1.5      # [rad/s] kecepatan roda rata-rata saat mundur (~0.15 m/s)
SONAR_BASI   = 1.0      # [s] data sonar lebih tua dari ini -> robot berhenti
R_RODA       = 0.0975   # [m] jari-jari roda P3-DX


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

def pilih_arah_mundur(sensor):                                  #[ROBOT]
    # Objek lebih dekat di kiri -> moncong diputar ke kanan (-1), dan sebaliknya.
    return -1.0 if min(sensor[0:4]) < min(sensor[4:8]) else 1.0

def kecepatan_mundur(arah):                                     #[ROBOT]
    # Mundur sambil memutar moncong: arah +1 = berlawanan jarum jam (kiri),
    # -1 = searah jarum jam (kanan). Kit menjaga arti omega saat mundur sama
    # dengan P3-DX. Satu roda 0, roda lain -2*PHI_MUNDUR = belok paling tajam.
    dphi = PHI_MUNDUR * arah
    return -PHI_MUNDUR - dphi, -PHI_MUNDUR + dphi               # vLeft, vRight

## ------------ Main Program ----------------
print('Main Program Started')
clientID = p3dx.connect()                                       #[ROBOT] pengganti sim.simxStart
sensorsHandle = p3dx.getSensorsHandle(clientID)
motorsHandle = p3dx.getMotorHandle(clientID)

np.set_printoptions(precision=2)
mundur_sampai = -1e9                                            #[ROBOT]
arah_mundur = 1.0
t_print = 0.0
print('Robot jalan sendiri. Tekan q untuk berhenti.  !! Percobaan pertama: angkat roda dulu.')

## ---------- Routine 100 Hz ----------------
while True:
    k = p3dx.tombol()                                           #[ROBOT] pengganti keyboard.is_pressed
    sensorValue = p3dx.getDistance(clientID, sensorsHandle)
    now = time.monotonic()

    detect = sensor_activation(sensorValue)
    vLeft, vRight = braitenberg(detect)
    status = ''

    #[ROBOT] Mundur kalau macet (Braitenberg minta putar di tempat) atau objek terlalu dekat
    macet = R_RODA * (vLeft + vRight) / 2.0 < V_MACET
    if (macet or min(sensorValue[2:6]) < JARAK_MUNDUR) and now >= mundur_sampai:
        if now - mundur_sampai > KUNCI_DETIK:
            arah_mundur = pilih_arah_mundur(sensorValue)
        mundur_sampai = now + MUNDUR_DETIK
        vMundur = kecepatan_mundur(arah_mundur)
    if now < mundur_sampai:
        if min(sensorValue[10:14]) < JARAK_MUNDUR:
            vLeft, vRight = 0.0, 0.0                            # belakang juga tertutup: diam
            status = 'TERJEPIT, berhenti'
        else:
            vLeft, vRight = vMundur
            status = 'MUNDUR'

    #[ROBOT] Sonar tidak datang lagi (LiDAR / jaringan mati) -> berhenti
    if p3dx.sensorAge(clientID) > SONAR_BASI:
        vLeft, vRight = 0.0, 0.0
        status = 'DATA SONAR BASI, berhenti'

    if now - t_print > 0.1:                                     #[ROBOT] print 10x per detik
        t_print = now
        print('front =', sensorValue[0:8], ' vL = %.2f, vR = %.2f  %s' % (vLeft, vRight, status))

    # setWheelVel: veloCmd[0] -> motor kanan, veloCmd[1] -> motor kiri
    # (m_p3dx versi robot menjaga urutan ini sama dengan Coppelia)
    p3dx.setWheelVel(clientID, motorsHandle, [vRight, vLeft])

    # Press q to quit
    if k == 'q':
        p3dx.setWheelVel(clientID, motorsHandle, [0.0, 0.0])
        break

    time.sleep(0.01)

# Stop the robot
p3dx.disconnect(clientID)                                       #[ROBOT] pengganti sim.simxFinish
print('Main Program Ended')
