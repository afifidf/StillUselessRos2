# Tutorial Transfer Action Antar Robot OP3

Panduan ini menjelaskan cara memindahkan action custom (misal kick) dari satu robot ke robot lain menggunakan `action_bin_tool.py`.

---

## Konsep Dasar

Semua action/gerakan robot disimpan dalam satu file binary:
```
~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin
```

File ini berisi **256 page**, tiap page = satu action. Contoh page yang umum dipakai:

| Page | Nama Action   |
|------|---------------|
| 1    | standup       |
| 9    | walk ready    |
| 83   | right kick    |
| 84   | left kick     |
| 122  | getup front   |
| 123  | getup back    |

---

## Step 1 — Cari Tahu Username dan IP Robot

### Cari Username
Di robot (colok keyboard + monitor), jalankan:
```bash
whoami
```
Username default robot ROBOTIS OP3 biasanya: `robotis`

### Cari IP Address Robot
Di robot, jalankan:
```bash
hostname -I
```
Contoh output: `192.168.1.13` → itu IP robot tersebut.

### Cari IP dari Robot Lain / Laptop
Kalau semua device konek ke WiFi/LAN yang sama:
```bash
# Scan semua device di jaringan (ganti 192.168.1 sesuai jaringan kamu)
nmap -sn 192.168.1.0/24
```
Atau:
```bash
arp -a
```

### Test Koneksi
```bash
# Ping
ping 192.168.1.XX

# SSH
ssh robotis@192.168.1.XX
```

> **Catatan:** Robot 2 dan Robot 3 harus terhubung ke WiFi/LAN yang sama.

---

## Step 2 — Lihat Daftar Action di Robot

Jalankan di robot manapun untuk melihat semua action yang ada:
```bash
python3 ~/ros2iwandwi/src/robotis_manager/robotis_manager/action_bin_tool.py list \
  ~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin
```

Contoh output:
```
 Page | Name           | Steps | Speed | Repeat | Next | Exit
-----------------------------------------------------------------
    1 | standup        |     1 |    32 |      1 |    0 |    0
   83 | r_kick         |     5 |    32 |      1 |    0 |    0
   84 | l_kick         |     5 |    32 |      1 |    0 |    0
  122 | f_get_up       |     4 |    32 |      1 |    0 |    0
```

### Lihat Detail Satu Action
```bash
python3 ~/ros2iwandwi/src/robotis_manager/robotis_manager/action_bin_tool.py info \
  ~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin \
  83
```

---

## Step 3 — Transfer Action dari Robot 3 ke Robot 2

Semua command dijalankan **di Robot 3**.

### 3a. Ambil file .bin dari Robot 2
```bash
scp robotis@<IP_ROBOT2>:~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin ~/robot2_motion.bin
```
Ganti `<IP_ROBOT2>` dengan IP Robot 2, misal `192.168.1.12`.

### 3b. Copy page action yang diinginkan
```bash
python3 ~/ros2iwandwi/src/robotis_manager/robotis_manager/action_bin_tool.py copy \
  ~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin \
  ~/robot2_motion.bin \
  83 84
```
Angka `83 84` adalah nomor page yang mau dicopy. Bisa ditambah sesuai kebutuhan, misal `83 84 122 123`.

### 3c. Kirim file yang sudah diupdate ke Robot 2
```bash
scp ~/robot2_motion.bin robotis@<IP_ROBOT2>:~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin
```

### 3d. (Opsional) Hapus file sementara di Robot 3
File `robot2_motion.bin` di Robot 3 sudah tidak dibutuhkan setelah dikirim. Boleh dihapus untuk kebersihan, tapi boleh juga disimpan sebagai backup.
```bash
rm ~/robot2_motion.bin
```

---

## Verifikasi

Setelah transfer, cek di **Robot 2** apakah page yang dicopy sudah ada:
```bash
python3 ~/ros2iwandwi/src/robotis_manager/robotis_manager/action_bin_tool.py info \
  ~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin \
  83
```

---

## Ringkasan Command (Copy-Paste)

Ganti `<IP_ROBOT2>` dan nomor page sesuai kebutuhan.

```bash
# Di Robot 3 — ambil file Robot 2
scp robotis@<IP_ROBOT2>:~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin ~/robot2_motion.bin

# Di Robot 3 — copy page action
python3 ~/ros2iwandwi/src/robotis_manager/robotis_manager/action_bin_tool.py copy \
  ~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin \
  ~/robot2_motion.bin \
  83 84

# Di Robot 3 — kirim balik ke Robot 2
scp ~/robot2_motion.bin robotis@<IP_ROBOT2>:~/ros2iwandwi/install/op3_action_module/share/op3_action_module/data/motion_4095.bin

# Di Robot 3 — hapus file sementara
rm ~/robot2_motion.bin
```
