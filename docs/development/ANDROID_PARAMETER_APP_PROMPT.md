# Prompt Aplikasi Android Konfigurasi AUV

Salin prompt berikut ke AI coding tool pilihan Anda:

---

Buat aplikasi Android native untuk konfigurasi parameter AUV SAUVAC berbasis **Kotlin + Jetpack Compose**. Aplikasi hanya digunakan saat robot berada di permukaan/bench, terhubung langsung ke access point Wi-Fi Pico W. Firmware menyediakan **UDP JSON**, bukan HTTP, REST, WebSocket, atau web server di Pico.

## Koneksi perangkat

- Minta operator memilih SSID `SAUVAC-XXXX` dan memasukkan password WPA2 yang disediakan pemilik perangkat.
- Alamat Pico W tetap `192.168.4.1`, port UDP `4210`.
- Android harus terhubung ke access point robot tanpa internet. Tangani pesan Android “Wi-Fi tidak memiliki internet” dengan mempertahankan koneksi ke jaringan tersebut.
- Tambahkan tombol Connect/Disconnect, indikator Wi-Fi, indikator firmware, status arm/FSM, kesiapan sensor, umur data sensor, dan pesan error yang jelas.
- Setelah terhubung kirim `{"req_id":N,"op":"get"}` dan polling status hanya selama halaman utama terbuka, maksimal 1 kali per detik.
- Gunakan UDP request-response: setiap permintaan punya `req_id` integer unik dan balasan membawa `req_id` yang sama. Timeout 1 detik, satu retry maksimum, abaikan balasan dengan ID yang sudah kedaluwarsa.
- Batasi request JSON di bawah 384 byte. Jangan mengirim command mentah, alamat internet, DNS, HTTP, atau paket broadcast.

## Protokol

Permintaan yang diizinkan:

```json
{"req_id":1,"op":"get"}
{"req_id":2,"op":"set_pid","axis":"depth","kp":0.6,"ki":0.0,"kd":0.015}
{"req_id":3,"op":"set_thruster","id":1,"reverse":true,"trim":1.0,"pulse_min":1000,"pulse_neutral":1500,"pulse_max":2000}
{"req_id":4,"op":"set_power_limit","percent":25}
{"req_id":5,"op":"set_deadband","microseconds":25}
{"req_id":6,"op":"set_balance","axis":"vert","percent":0.0}
{"req_id":7,"op":"set_autorun_timing","delay_sec":10,"tare_sec":3,"level_sec":2}
{"req_id":8,"op":"apply"}
{"req_id":9,"op":"save"}
{"req_id":10,"op":"discard"}
```

Balasan menggemakan `req_id`; `id` pada `set_thruster` adalah nomor thruster. `get` membalas konfigurasi `settings` (draft firmware saat ini), flag `dirty`, dan status:
`armed`, `fsm_state`, `imu_ready`, `pressure_ready`, `depth_calibrated`, `sensor_age_ms`.
Balasan `get` juga memuat `protocol_version: 1`; tolak atau beri peringatan untuk versi protokol yang tidak dikenal.
Jangan anggap data sensor siap jika flag tidak menyatakan siap atau umurnya lebih dari 500 ms.

Aplikasi menyimpan baseline konfigurasi hasil `get` lokal di memori selama sesi, dan menjaga draft UI sendiri. Setelah `set_*` sukses, ambil `get` ulang untuk sinkronisasi draft firmware. Jangan menganggap `apply` menyimpan EEPROM.

## Layar dan alur konfigurasi

1. **Beranda / Connect** — status koneksi, firmware, disarmed/armed, status sensor dan umur data.
2. **PID** — depth dan roll saja; numeric input, validasi 0–100, tampilkan nilai aktif vs draft. Pitch harus ditampilkan sebagai “tidak didukung firmware”.
3. **Thrusters** — enam motor, pin hanya-baca, reverse, trim 0.1–2.5, serta pulsa min/neutral/max dengan rentang dan invariant min < neutral < max.
4. **Safety & autorun** — power cap 1–100%, ESC deadband 0–80 µs, horizontal/vertical balance -20..20%, autorun timing sesuai validasi firmware. Autorun harus sudah nonaktif untuk mengubah timing; mengaktifkan atau menonaktifkannya hanya dapat dilakukan melalui USB.
5. **Review changes** — daftar nilai current → pending sebelum apply/save.
6. **Backup** — export/import profil konfigurasi JSON lokal; validasi penuh sebelum mengirim. Jangan menyimpan passphrase Wi-Fi di backup atau log.

Workflow wajib: edit → validasi lokal → `set_*` untuk stage di firmware → tampilkan nilai pending → operator menekan Apply → kirim `apply` → verifikasi dengan `get` → tombol Save eksplisit untuk menyimpan EEPROM. `discard` membatalkan perubahan runtime yang belum disimpan dan mengembalikan konfigurasi terakhir yang tersimpan di EEPROM. Jika operasi gagal, pertahankan input lokal dan jelaskan penyebabnya; jangan tampilkan toast sukses palsu.

## Pengamanan dan UX

- Semua perubahan firmware ditolak kecuali sistem **idle dan disarmed**; aplikasi harus menonaktifkan Apply/Save saat status lain dan tetap menangani penolakan dari firmware.
- Jangan menambahkan kontrol motor, arm, run mission, ESC calibration, raw serial, atau emergency-stop via jaringan. Ini jalur konfigurasi parameter saja.
- `apply` bukan berarti permanen; hanya `save` yang menulis EEPROM. Tampilkan status “belum disimpan”.
- Konfirmasi kedua sebelum Save, perubahan autorun timing, power limit di atas 25%, atau perubahan reverse/pulse.
- Jangan pernah menganggap Wi-Fi sebagai emergency stop atau link kendali bawah air. Aplikasi harus menyatakan koneksi ini hanya untuk setup saat AUV di atas air/bench.
- WPA2 passphrase dimasukkan pengguna dan tidak disimpan/log. Jangan hard-code SSID password atau token rahasia ke source aplikasi.
- Jangan menyimpan parameter kandidat otomatis ke EEPROM. Tidak ada flashing firmware dari aplikasi.
- UI industrial yang rapi, sederhana, responsif, dapat dibaca outdoor, touch target minimal 48dp, accessibility labels, keyboard numeric untuk nilai, dan error banner yang persisten.

Sertakan unit test untuk JSON protocol, timeout/retry, validasi nilai, unmatched response ID, offline/stale sensors, perubahan yang ditolak saat armed, serta verifikasi `apply` berbeda dari `save`. Pisahkan repository UDP dari UI agar protokol dapat diuji dengan fake transport.

---

## Firmware: enable access point

Password sengaja tidak disimpan dalam repositori. Sebelum build firmware untuk perangkat, salin `src/wireless_secrets.example.h` menjadi `src/wireless_secrets.h` dan ganti passphrase dengan nilai unik minimal 12 karakter. Header lokal ini diabaikan Git. Tanpa file/password yang layak, access point nonaktif dan USB serial tetap bekerja.

Access point konfigurasi memakai SSID unik per Pico, alamat `192.168.4.1`, WPA2, dan UDP port `4210`. Periksa status Wi-Fi dengan serial saat bench. Jangan mengandalkan koneksi radio untuk kontrol saat AUV tenggelam.
