# StillUselessRos2 - Robotis OP3 Manager (ROS2 Jazzy)

Program kontrol robot ROBOTIS OP3 yang diport dari ROS1 Noetic ke ROS2 Jazzy.

## Fitur
- Deteksi bola menggunakan HSV color masking
- Walking control
- Head tracking ke arah bola
- Task control (find ball → approach → kick → getup)
- Kalibrasi warna bola dan lapangan
- Action editor untuk edit gerakan robot
- Tool transfer action antar robot

---

## Setup di Robot Baru (Fresh Install)

### 1. Install ROS2 Jazzy
```bash
# Ikuti panduan resmi: https://docs.ros.org/en/jazzy/Installation.html
sudo apt install ros-jazzy-ros-base
source /opt/ros/jazzy/setup.bash
```

### 2. Install tools yang dibutuhkan
```bash
sudo apt install python3-vcstool python3-rosdep python3-colcon-common-extensions
```

### 3. Clone repo ini
```bash
git clone https://github.com/afifidf/StillUselessRos2.git ~/ros2iwandwi
cd ~/ros2iwandwi
```

### 4. Pull semua ROBOTIS dependencies dengan vcs
```bash
vcs import < ros2iwandwi.repos
```

Perintah ini akan otomatis clone package-package berikut ke folder `src/`:

| Folder | Package | Fungsi |
|---|---|---|
| `src/DynamixelSDK` | `dynamixel_sdk` | Low-level komunikasi motor |
| `src/ROBOTIS-Framework` | `robotis_controller`, `robotis_device` | Core robot controller |
| `src/ROBOTIS-Framework-msgs` | `robotis_controller_msgs` | Messages untuk controller |
| `src/ROBOTIS-Math` | `robotis_math` | Library matematika robot |
| `src/ROBOTIS-OP3` | `op3_manager`, `op3_action_module`, `op3_walking_module`, dll | Stack utama OP3 |
| `src/ROBOTIS-OP3-msgs` | `op3_walking_module_msgs`, dll | Messages untuk OP3 |
| `src/ROBOTIS-OP3-Tools` | `op3_action_editor` | Tool edit gerakan robot |

### 5. Install system dependencies dengan rosdep
```bash
sudo rosdep init    # skip jika sudah pernah dijalankan
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> `rosdep` otomatis install dependencies sistem seperti `libncurses-dev`, `libyaml-cpp-dev`, `python3-opencv`, `mpg321`, dll.

### 6. Build workspace
```bash
colcon build
source install/setup.bash
```

> Tambahkan `source ~/ros2iwandwi/install/setup.bash` ke `~/.bashrc` supaya tidak perlu source setiap kali:
> ```bash
> echo "source ~/ros2iwandwi/install/setup.bash" >> ~/.bashrc
> ```

---

## Cara Menjalankan

### Bringup (wajib dijalankan pertama)
Menjalankan `op3_manager` (core controller) dan kamera.
Robot akan bergerak ke **initial pose** saat pertama jalan.
```bash
ros2 launch robotis_manager robotis_bringup.launch.py
```

### Program Utama (robot siap bermain)
Jalankan setelah bringup. Robot akan:
- Deteksi bola
- Jalan mendekat ke bola
- Tendang bola
- Bangun sendiri jika jatuh
- Head tracking ke arah bola
```bash
ros2 launch robotis_manager robotis_main.launch.py
```

### Kalibrasi Warna (tanpa robot hardware)
Untuk kalibrasi HSV bola orange dan lapangan hijau.
```bash
ros2 launch robotis_manager robotis_calibrate.launch.py
```
Window yang muncul:
- `frame_ball` : kamera utama + titik bola terdeteksi
- `field_mask` : mask lapangan hijau
- `ball_mask`  : mask bola orange
- `hsv`        : frame HSV
- `Calibrate`  : **geser trackbar untuk ubah nilai HSV**

Hasil kalibrasi otomatis tersimpan ke file `.txt` di folder `data/`.

### Action Editor (edit gerakan robot)
Untuk membuat atau memodifikasi gerakan robot.
Robot harus terhubung via USB.
```bash
ros2 launch robotis_manager robotis_action_editor.launch.py
```
Atau cara lama:
```bash
ros2 run op3_action_editor executor.py
```

---

## Transfer Action Antar Robot

Lihat panduan lengkap di **[TRANSFER_ACTION.md](TRANSFER_ACTION.md)**

Singkatnya — jalankan di Robot 3 untuk copy action ke Robot 2:
```bash
# Ambil file dari Robot 2
scp robotis@<IP_ROBOT2>:~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin ~/robot2_motion.bin

# Copy page action yang diinginkan (misal page 83 = kick kanan, 84 = kick kiri)
ros2 run robotis_manager action_bin_tool copy \
  ~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin \
  ~/robot2_motion.bin 83 84

# Kirim balik ke Robot 2
scp ~/robot2_motion.bin robotis@<IP_ROBOT2>:~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin
```

---

## Struktur Package

```
ros2iwandwi/
├── ros2iwandwi.repos          # File vcs untuk pull ROBOTIS dependencies
├── README.md                  # File ini
├── TRANSFER_ACTION.md         # Panduan transfer action antar robot
└── src/
    └── robotis_manager/
        ├── launch/
        │   ├── robotis_bringup.launch.py        # Launch pertama (op3_manager + kamera)
        │   ├── robotis_main.launch.py           # Launch utama robot main
        │   ├── robotis_calibrate.launch.py      # Kalibrasi warna
        │   └── robotis_action_editor.launch.py  # Edit gerakan robot
        ├── config/
        │   └── camera_param.yaml               # Parameter kamera USB
        ├── robotis_manager/
        │   ├── main_node_handler.py            # Entry point vision node
        │   ├── main_task_control.py            # State machine utama
        │   ├── main_motion_driver.py           # Driver walking & action
        │   ├── main_head_tracking.py           # Head tracking
        │   ├── main_calibrate.py               # Kalibrasi HSV
        │   ├── vision_main.py                  # Deteksi bola
        │   ├── vision_image.py                 # Kamera & image processing
        │   ├── vision_utils.py                 # HSV masking, blob detection
        │   ├── vision_ros_handler.py           # ROS2 pub/sub vision
        │   ├── motion_data_flow_handler.py     # ROS2 pub/sub motion
        │   ├── motion_module.py                # Konstanta action & walking
        │   ├── action_bin_tool.py              # Tool baca/tulis motion_4095.bin
        │   └── data/
        │       ├── robotis_orange_ball.txt     # Nilai HSV bola orange
        │       ├── robotis_green_field.txt     # Nilai HSV lapangan hijau
        │       └── robotis_main_vision_configurator.json
        └── package.xml
```

---

## Topic Penting

| Topic | Tipe | Publisher | Subscriber |
|---|---|---|---|
| `/usb_cam_node/image_raw` | `sensor_msgs/Image` | `usb_cam_node` | `main_node_handler` |
| `/robotis/ball_position` | `geometry_msgs/Point` | `main_node_handler` | `main_motion_driver`, `main_head_tracking` |
| `/robotis/walking/command` | `std_msgs/String` | `main_motion_driver` | `op3_walking_module` |
| `/robotis/action/page_num` | `std_msgs/Int32` | `main_motion_driver` | `op3_action_module` |
| `/robotis/head_control/set_joint_states` | `sensor_msgs/JointState` | `main_head_tracking` | `op3_head_control_module` |

---

## Troubleshooting

### Robot tidak bergerak ke initial pose
Pastikan `op3_manager` sudah jalan dan robot terhubung via USB (`/dev/ttyUSB0`).

### Kamera tidak terdeteksi
```bash
ls /dev/video*
# Ganti device di config/camera_param.yaml jika bukan /dev/video0
```

### Build error: package not found
Pastikan sudah jalankan `vcs import < ros2iwandwi.repos` dan `rosdep install`.

### Trackbar kalibrasi tidak ngaruh ke gambar
Pastikan `main_node_handler` dan `main_calibrate` jalan di saat yang sama (pakai `robotis_calibrate.launch.py`).
