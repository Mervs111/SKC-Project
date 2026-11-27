# Program P3-DX versi robot JetRacer

Isi folder ini adalah program CoppeliaSim di folder utama yang sudah disesuaikan
untuk robot AMR JetRacer ([kit mahasiswa](https://github.com/ismarintan98/jetracer-kit-mahasiswa)).
Setiap perubahan dari versi Coppelia ditandai komentar `#[ROBOT]`.

| File | Isi |
|---|---|
| `m_p3dx.py` | Pengganti `m_p3dx.py` Coppelia. Membungkus `lib/p3dx.py` dari kit, plus posisi dari `/odom` |
| `KeyboardControl.py` | w/s/a/d + rem anti-nabrak depan & belakang |
| `ObjectAvoidance_Braitenberg.py` | Braitenberg + mundur sambil belok kalau macet |
| `ObjectFollower_Forward+Reverse.py` | Ikuti objek depan (`c`) / belakang (`v`) |
| `pose_control.py` | Menuju titik target, log CSV |
| `pose_control_avoidance.py` | Pose control + Braitenberg, log CSV |

## Menjalankan (di PC Linux)

Ikuti dulu bagian 1–2 README kit (`./setup.sh`, cek `ros2 topic list`). Lalu
clone repo ini **di dalam folder kit**, supaya ikut ter-mount di `/workspace`:

```bash
cd ~/jetracer-kit-mahasiswa                 # folder kit
git clone https://github.com/Mervs111/SKC-Project.git
export ROBOT_IP=10.17.36.90                 # IP dari layar OLED robot

./ros.sh python3 /workspace/SKC-Project/robot/KeyboardControl.py
./ros.sh python3 /workspace/SKC-Project/robot/ObjectAvoidance_Braitenberg.py
./ros.sh python3 /workspace/SKC-Project/robot/ObjectFollower_Forward+Reverse.py
./ros.sh python3 /workspace/SKC-Project/robot/pose_control.py 1.5 0.5 0       # x [m], y [m], gamma [derajat]
./ros.sh python3 /workspace/SKC-Project/robot/pose_control_avoidance.py 2.0 0 0
```

Update kode: `cd ~/jetracer-kit-mahasiswa/SKC-Project && git pull`.

`q` = keluar. Ctrl-C juga aman: robot berhenti dan log tetap tersimpan.
Log pose control (`log_*.csv`) tersimpan di folder ini.

**Percobaan pertama: angkat roda dulu dari lantai.**

## Yang berubah dibanding Coppelia

- **Tidak ada `sim` dan `keyboard`.** Pakai `p3dx.connect()`, `p3dx.tombol()`
  dan `p3dx.disconnect()`. Modul `keyboard` di Linux butuh root, jadi tidak dipakai.
- **Urutan roda tetap `[phiR, phiL]`** seperti `m_p3dx` kalian. Kit memakai
  `[phiL, phiR]`, dan `m_p3dx.py` di sini yang menukarnya. Jangan ganti ke
  `import p3dx` langsung: Braitenberg dan pose control akan belok ke arah yang salah.
- **Ackermann tidak bisa berputar di tempat** (v = 0 berarti diam):
  - tombol `a`/`d` = maju sambil belok;
  - pose control: selama menuju target, kecepatan maju minimal `V_MIN` = 0.1 m/s;
  - Braitenberg: kalau robot "minta" berputar di tempat atau objek < 15 cm di
    depan, robot mundur 1.5 s sambil belok penuh. Arah beloknya dikunci 3 s
    supaya tidak maju-mundur terus di pojok.
  - Di akhir pose control, arah hadap (gamma) tidak bisa dikoreksi di tempat.
- **Posisi robot & target.** Tidak ada `Disc` lagi; targetnya `TARGET` atau
  argumen program. Posisi diambil dari `/odom`: (0, 0, 0) = posisi robot saat
  program mulai, x = maju, y = ke kiri. Odometri melenceng makin jauh. Untuk
  posisi peta pakai `SUMBER_POSE = "map"` sambil menjalankan lokalisasi kit.
- **Pengaman:** object follower berhenti kalau objek hilang (> 1 m). Semua
  program berhenti kalau data sonar/posisi tidak datang lagi.
- Gain dan matriks kinematika **tidak diubah**. Robot meniru P3-DX Coppelia
  (r = 0.0975 m, jarak roda 0.331 m), jadi gain hasil tuning di simulasi tetap berlaku.

## Belum diuji di robot sungguhan

Semua program sudah diuji dengan simulasi robot Ackermann tiruan (sonar, `/odom`,
keyboard), tapi belum di robotnya. Yang mungkin perlu disetel di lapangan:
`V_MIN` (kalau robot tidak mau jalan pelan), `JARAK_MUNDUR`, `MUNDUR_DETIK`,
dan kecepatan di `KeyboardControl.py`. Mode `SUMBER_POSE = "map"` belum diuji sama sekali.
