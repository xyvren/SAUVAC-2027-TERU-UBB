# Audit Web Simulasi AUV SAUVAC

Tanggal: 18 November 2026  
Status: audit statis kode dan inspeksi browser lokal; belum diverifikasi pada Pico W.

Pembaruan arsitektur 19 November 2026: aplikasi utama tetap di port 8080.
`config.html` dan `server.py` kini hanya menjadi pengalihan kompatibilitas ke
tab Pengaturan; `server.py` tidak lagi membuat serial bridge atau API kedua.

## Cakupan dan batasan

Fokus audit adalah aplikasi utama yang dijalankan oleh
[`start.bat`](./simulasi_auv/start.bat): [`index.html`](./simulasi_auv/index.html),
[`app.py`](./simulasi_auv/app.py), [`hydrodynamics.js`](./simulasi_auv/hydrodynamics.js),
[`sim3d.js`](./simulasi_auv/sim3d.js), dan [`rl_agent.py`](./simulasi_auv/rl_agent.py).
Jalur konfigurasi lama di port 8081 ([`server.py`](./simulasi_auv/server.py) /
[`config.html`](./simulasi_auv/config.html)) hanya diperiksa untuk memahami
arsitektur, bukan diaudit penuh.

Browser berhasil memuat HTML dari `file:` untuk memeriksa struktur halaman.
Server `http://127.0.0.1:8080` tidak berjalan (connection refused), sehingga
endpoint API dan bridge serial tidak dapat diuji langsung. Permintaan API dari
`file:` mendapat 403 karena bukan origin server aplikasi; itu bukan bukti bahwa
API gagal saat halaman dibuka melalui server yang benar. Inspeksi browser juga
menampilkan warning Three.js tentang properti `clearcoat` yang tidak didukung
oleh `MeshStandardMaterial`.

## Temuan

### Tinggi — Gerakan selam/naik manual pada simulasi 2D/3D terbalik

- Status: diperbaiki. Pulsa T5/T6 sekarang mengikuti polaritas firmware pada
  kontrol manual, misi virtual 2D, dan model mock server.
- Bukti sebelum perbaikan: [`index.html`](./simulasi_auv/index.html#L4466)
  sebelumnya mengirim `[1320, 1680]` untuk `dive` dan `[1680, 1320]` untuk
  `surface`. Sekarang pasangan pulsa dibalik agar kedua thruster memberi gaya
  ke arah yang sama.
- Firmware menetapkan polaritas T5 `reverse=0` dan T6 `reverse=1`
  ([`config.h`](./src/config.h#L29), `DEFAULT_THRUSTER_REVERSE`). Allocator
  simulasi memakai polaritas yang sama
  ([`hydrodynamics.js`](./simulasi_auv/hydrodynamics.js#L375),
  `thrustAllocation`). Dengan aturan tersebut, pulsa lama membuat `dive`
  menghasilkan heave negatif dan `surface` positif; kondisi ini sudah
  diperbaiki pada kontrol manual dan misi virtual.
- Dampak: tombol Q/E, tombol pad selam/naik, arah vektor thruster, dan gerak
  model dapat berlawanan dengan label. Ini merusak validitas simulasi gerak.
- Perbaikan diterapkan: pasangan T5/T6 sekarang berlawanan relatif terhadap
  1500 µs, termasuk pada misi 2D dan mock backend. Uji browser mengonfirmasi
  arah heave yang diharapkan.

### Tinggi — Telemetri mock atau lama dapat tampil sebagai sensor langsung

- Bukti sebelum perbaikan: [`app.py`](./simulasi_auv/app.py#L634) pada
  `_mock_tick` mengisi pembacaan sintetis dan menyetel `imu_ready` serta
  `pressure_ready` ke `True` saat serial tidak terhubung. API kini menandai
  `sensor_source` sebagai `simulation` atau `hardware`.
- Status: diperbaiki. API sekarang mengirim `sensor_source`; UI membedakan
  `HARDWARE LIVE`, simulasi lokal, dan data offline/kedaluwarsa.
- Bukti UI: [`index.html`](./simulasi_auv/index.html#L2390) kini mensyaratkan
  sumber, koneksi, umur telemetri, dan flag kesiapan sebelum memberi label
  `HARDWARE LIVE`.
- Dampak: setelah Pico terputus atau dalam fallback mock, kartu bertajuk
  “Pembacaan Sensor Langsung” bisa menampilkan angka simulasi/terakhir sebagai
  data sensor siap. Status koneksi terputus di header tidak cukup untuk
  membedakan sumber angka di setiap kartu.
- Perbaikan diterapkan: sumber simulasi diberi label `SIMULASI LOKAL · BUKAN
  DATA HARDWARE`; nilai simulasi tetap tersedia untuk digital twin, bukan
  ditampilkan sebagai telemetri hardware.

### Tinggi — Apply gain PID dapat menyimpan konfigurasi parsial setelah gagal

- Status: diperbaiki. Urutan apply berhenti pada perintah pertama yang gagal
  dan `save` hanya dijalankan setelah kedua PID berhasil.
- Bukti sebelum perbaikan: endpoint `/api/rl_pid/apply` mengirim kedua perintah
  PID dan `save` tanpa menghentikan urutan ketika ada kegagalan. Sekarang
  [`app.py`](./simulasi_auv/app.py#L1473) berhenti pada hasil gagal pertama dan
  hanya menyimpan jika kedua perubahan berhasil.
- Dampak: jika salah satu perubahan ditolak atau timeout, endpoint tetap
  mencoba menyimpan konfigurasi yang mungkin hanya berubah sebagian. UI
  akhirnya hanya menampilkan kegagalan generik, sementara state firmware
  mungkin telah berubah.
- Perbaikan diterapkan: respons menyertakan hasil perintah, status offline, dan
  alasan berhenti; `save` dilewati setelah kegagalan.

### Sedang — Tombol tare kompas selalu melaporkan sukses meski endpoint menolak

- Status: diperbaiki. UI mengirim JSON, memeriksa status HTTP dan payload,
  serta memberi peringatan jika sinkronisasi gagal. Tare ini hanya mengubah
  referensi tampilan/server, bukan kalibrasi sensor firmware.
- Bukti sebelum perbaikan: [`index.html`](./simulasi_auv/index.html#L3227),
  `tareCompass()`, sebelumnya mengirim POST tanpa header/body JSON dan tidak
  memeriksa respons. Sekarang request mengirim `{}` dan memeriksa HTTP serta
  payload.
- Bukti backend: [`app.py`](./simulasi_auv/app.py#L1221), `do_POST`, menangani
  tare sebagai referensi lokal/server tanpa menimpa telemetri yaw hardware.
  Validasi `Content-Type` dan body JSON tetap diberlakukan oleh `do_POST`.
- Dampak sebelum perbaikan: server menolak request, tetapi UI menampilkan
  sukses. Sekarang kegagalan memunculkan peringatan, dan pesan sukses
  menjelaskan bahwa tare hanya mengubah referensi lokal/server, bukan
  kalibrasi sensor firmware.

### Sedang — Polling dapat mempertahankan status lama dan membuat request menumpuk

- Status: diperbaiki. Polling kini menjadwalkan request berikutnya setelah
  request selesai, memakai timeout 2 detik, dan menandai data offline untuk
  respons non-2xx, payload tidak valid, maupun error jaringan.
- Bukti sebelum perbaikan: `pollState()` tidak memeriksa status HTTP dan
  dijalankan lewat interval tetap. Sekarang [`index.html`](./simulasi_auv/index.html#L2287)
  menunggu request selesai, memakai timeout, dan melempar error pada status
  non-2xx.
- Dampak: server yang merespons lambat dapat menghasilkan request bersamaan dan
  respons lama dapat menimpa yang baru. Jika API mengembalikan 500/403, UI tetap
  mempertahankan koneksi, telemetri, dan status misi dari respons sebelumnya.
- Perbaikan diterapkan: state API dan sensor ditandai offline/kedaluwarsa pada
  error jaringan, timeout, HTTP non-2xx, atau payload invalid. Header, status
  misi, status ARM, dan pulsa thruster juga menandai data terakhir sebagai lama.

### Sedang — Input JSON rute dapat merusak editor sebelum ada validasi skema

- Status: diperbaiki. Seluruh langkah dinormalisasi dan divalidasi sebelum
  mengganti state editor; objek non-valid, aksi, rentang angka, ID motor, jumlah
  langkah, dan ukuran payload ditolak.
- Bukti sebelum perbaikan: `applyJsonToTable()` hanya memeriksa array dan
  mengubah `missionSteps` sebelum render. Sekarang
  [`index.html`](./simulasi_auv/index.html#L2986) memvalidasi setiap elemen ke
  state sementara sebelum mengganti editor.
- Dampak: toast parsing error muncul, tetapi state editor sudah berisi data
  yang salah sehingga render/operasi berikutnya juga gagal. Validasi backend
  upload tidak mencegah kerusakan state editor lokal ini.
- Perbaikan diterapkan: objek, aksi, nilai numerik/rentang, ID motor, jumlah
  langkah, dan ukuran payload divalidasi; input salah ditolak dengan pesan
  spesifik tanpa merusak state editor.

### Sedang — Loop animasi terus berjalan ketika tab simulasi tidak terlihat

- Status: diperbaiki. Loop Canvas dijeda saat halaman tersembunyi atau tab
  tidak relevan; loop Three.js dihentikan ketika viewport 3D tidak terlihat dan
  dilanjutkan kembali saat ditampilkan.
- Bukti sebelum perbaikan: loop Canvas dan Three.js terus berjalan meski view
  tidak terlihat. Sekarang loop Canvas dikelola oleh
  [`scheduleRenderSim`](./simulasi_auv/index.html#L5141). Three.js memakai
  [`syncAnimationState`](./simulasi_auv/sim3d.js#L178).
  Sebelumnya tidak ada penghentian saat tab simulasi
  disembunyikan atau saat halaman berada di background.
- Dampak: halaman dashboard tetap menghitung fisika dan menggambar Canvas;
  setelah 3D dibuka, WebGL juga terus merender meski panelnya tersembunyi.
  Ini membuang CPU/GPU dan baterai pada laptop, khususnya saat simulator
  dibiarkan terbuka lama.
- Perbaikan diterapkan: Canvas dijeda saat tab tidak relevan/halaman hidden;
  Three.js berhenti saat viewport tersembunyi dan dihidupkan kembali saat
  terlihat. Waktu delta direset ketika loop Canvas dilanjutkan.

### Rendah — Material 3D meminta `clearcoat` pada kelas material yang salah

- Status: diperbaiki. Material hull, duct, dan CAD yang memakai `clearcoat`
  sekarang menggunakan `MeshPhysicalMaterial`.
- Bukti sebelum perbaikan: hull, duct, dan STL memakai
  `MeshStandardMaterial` dengan `clearcoat`, yang memicu warning. Sekarang
  [`sim3d.js`](./simulasi_auv/sim3d.js#L591) memakai `MeshPhysicalMaterial`.
- Dampak: lapisan clearcoat diabaikan sehingga tampilan mengkilap yang
  diharapkan tidak diterapkan; warning memenuhi console.
- Perbaikan diterapkan: `MeshPhysicalMaterial` dipakai untuk semua material yang
  mengatur `clearcoat`.

## Catatan keselamatan dan arsitektur

- Tombol “STOP DARURAT” pada web melewati browser → server host → serial USB →
  firmware. Ia tidak independen dari server, OS, USB, atau firmware. Jangan
  mengandalkannya sebagai pemutus darurat fisik; tetap gunakan cutoff fisik.
- Aplikasi utama bind ke loopback `127.0.0.1` dan memeriksa Host/Origin.
  Namun, endpoint `/api/command` di [`app.py`](./simulasi_auv/app.py#L1215)
  meneruskan perintah CLI ke bridge. Jangan ubah bind
  ke semua antarmuka jaringan tanpa autentikasi dan pembatasan perintah.
- Jalur konfigurasi lama tidak lagi menjalankan backend terpisah. Gunakan
  aplikasi utama pada port 8080; port 8081 hanya tersedia jika `server.py`
  lama dijalankan secara manual, dan hanya mengalihkan ke tab Pengaturan.

## Validasi setelah perbaikan

- Diagnostik VS Code tidak menemukan kesalahan pada `app.py`, `index.html`,
  `hydrodynamics.js`, atau `sim3d.js`.
- Re-audit 21 November 2026: server lokal berhasil dijalankan pada
  `127.0.0.1:8080`; `/` mengembalikan HTTP 200 dan `/api/state` mengembalikan
  status mock dengan `connected=false` serta `sensor_source=simulation`.
- Pemeriksaan Chromium dan WebKit menggunakan 1440, 768, dan 390 px:
  tidak ada page error, request HTTP gagal, overflow horizontal, atau blank
  canvas. Chromium mencapai sekitar 60 fps pada ketiga ukuran. WebKit headless
  mencatat 94.3 ms/frame pada 1440 px, 34.2 ms/frame pada 768 px, dan
  18.2 ms/frame pada 390 px; hasil ini perlu diverifikasi pada Safari/GPU nyata
  karena headless WebKit dapat memakai renderer perangkat lunak.
- Penambahan 21 November 2026: tab Konfigurasi memiliki konfirmasi browser untuk
  limit daya, gain PID, deadband, balance, timing Auto-Run, EEPROM save/reset,
  kalibrasi, dan operasi parameter Wi-Fi. Perubahan serial/Wi-Fi yang berhasil
  atau ditolak dicatat sebagai `[CONFIG-AUDIT]` pada `/api/logs` dan ditampilkan
  pada card Riwayat Perubahan Konfigurasi.
- Perbaikan 21 November 2026 untuk blank 3D: renderer WebGL memakai fallback
  error yang jelas jika konteks gagal, ukuran viewport dipaksa setelah tab/3D
  aktif, dan loop Three.js tetap berjalan selama viewport terlihat meskipun
  simulasi fisika masih berhenti. Re-audit Chromium menampilkan kolam, robot,
  dan STL dengan 0 page error.
- Implementasi 21 November 2026 untuk rekomendasi keselamatan: watchdog telemetri
  500 ms dengan netralisasi HIL/serial, status firmware dan kesehatan telemetri,
  whitelist terminal, guard ARM/idle untuk misi dan motor test, konfirmasi upload
  misi, serta audit operasi kritis dan koneksi. Web menyediakan ekspor/restore JSON
  dengan validasi versi dan guidance BOOTSEL/UF2 tanpa flashing otomatis.
- Uji fisika Node.js: 13 dari 13 kasus lulus, termasuk polaritas dive/surface,
  gerak mission, batas kedalaman, dan batas dinding pool.
- Uji browser lokal: allocator menghasilkan heave positif untuk `dive`
  (+6.93) dan negatif untuk `surface` (-5.13); rute default lolos validasi,
  sedangkan JSON `["bad"]` ditolak tanpa mengubah rute; dua polling lambat
  terukur tetap tidak tumpang tindih; tare yang menerima HTTP 415 menampilkan
  peringatan, bukan sukses; telemetri mock dan state lama diberi label
  eksplisit; loop Canvas dijeda/dilanjutkan dan loop 3D berhenti saat tab
  tersembunyi.
- Pico/serial dan hardware belum dapat diuji pada re-audit ini karena tidak ada
  device COM4 terhubung; validasi arah thruster fisik tetap wajib dilakukan di
  bench yang aman sebelum deployment.
