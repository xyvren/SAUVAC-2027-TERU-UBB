# Review Kode AUV SAUVAC — Temuan dan Remediasi

Tanggal: 17 November 2026
Jenis review: static code review, pemeriksaan konfigurasi, dan build firmware
Status setelah remediasi kode: **temuan perangkat lunak ditangani sesuai tabel di bawah; keselamatan operasi belum terbukti dan tetap memerlukan validasi hardware bertahap. Jangan gunakan sebagai dasar uji bertenaga atau wet test tanpa prosedur keselamatan dan kill switch fisik independen.**

Pembaruan 19 November 2026: konfigurasi web sekarang memakai satu backend
`app.py` di port 8080. `server.py` dan `config.html` dipertahankan sebagai
pengalihan kompatibilitas saja; keduanya tidak lagi menyediakan bridge/API
kontrol kedua. Temuan arsitektur historis di bawah merujuk kondisi saat audit.

## Ringkasan eksekutif

Review awal menemukan masalah pada layanan kontrol web, failsafe injeksi, prasyarat sensor misi, PID pitch, validasi misi, dan upload serial. Perbaikan kode yang relevan telah diterapkan; hasil build hanya membuktikan kompilasi, bukan keselamatan aktuator atau ketepatan kontrol.

| Prioritas | Jumlah | Makna |
|---|---:|---|
| P1 — tangani sebelum uji dengan propeller/robot di air | 6 | Dapat menyebabkan kendali tak sah, kendali tanpa sensor, keluaran yang tidak sesuai, atau firmware kehabisan memori |
| P2 — perbaiki sebelum mengandalkan misi/simulator | 9 | Kegagalan persistensi, ketidaksesuaian model, atau gangguan operasi yang signifikan |
| P3 — kualitas/reproducibility | 2 | Risiko pemeliharaan dan klaim dokumentasi |

## Status remediasi

| ID | Status | Perbaikan dan batasan tersisa |
|---|---|---|
| P1-1 | Ditangani | Kedua server hanya bind ke loopback, memvalidasi Host/Origin, membatasi ukuran/format JSON, dan tidak memakai wildcard CORS. Ini bukan autentikasi untuk proses lokal. |
| P1-2 | Ditangani | Timeout injeksi membatalkan misi/countdown, menonaktifkan kendali aktif, melepas arm, dan menetralkan output; tidak ada auto-rearm. Belum diuji di hardware. |
| P1-3 | Ditangani | Misi/PID memerlukan output terpasang, sensor siap dan segar, serta tare depth yang berhasil; prasyarat gagal menolak start. Perlu validasi sensor pada unit fisik. |
| P1-4 | Ditangani sebagai fitur nonaktif | Kendali dan tuning pitch ditolak karena otoritas aktuator belum terbukti. Geometri fisik masih harus diverifikasi. |
| P1-5 | Ditangani | Host dan firmware memvalidasi skema, batas langkah/nilai/mask/aksi, finite numbers, dan batas ukuran sebelum mengganti rute. |
| P1-6 | Ditangani | Buffer upload firmware dibatasi sebelum append; host membatasi payload. Batas maksimum harus tetap disamakan jika protokol diubah. |
| P2-1 | Ditangani | Mount LittleFS gagal secara eksplisit, tanpa format otomatis; rute diparsing dan ditulis melalui file sementara sebelum state aktif diperbarui. |
| P2-2 | Sebagian ditangani | Default posisi/polaritas T5/T6 diselaraskan antara firmware dan simulator serta dokumentasi. Arah dorong, rotasi, dan roll tetap perlu uji fisik tanpa propeller. |
| P2-3 | Ditangani | Transaksi command/upload diserialisasi; akses serial dilindungi lock I/O, dan upload mengambil alih pembacaan respons selama transaksi. Belum diuji pada sambungan USB nyata. |
| P2-4 | Ditangani | Gain/pulsa finite dan dibatasi, invariant urutan pulsa divalidasi saat update/load, serta operasi konfigurasi dibatasi pada kondisi idle-disarmed. |
| P2-5 | Ditangani | Nilai misi dan konfigurasi dirender sebagai teks/elemen DOM; tidak ada interpolasi nilai dinamis misi ke `innerHTML`. |
| P2-6 | Ditangani | Server konfigurasi dipindah ke port 8081, probe launcher memakai endpoint khusus, dan layanan hanya menerima koneksi lokal. |
| P2-7 | Klaim diperbaiki | UI/dokumentasi menandai simulator dan pencarian gain sebagai eksperimental/tereduksi; modelnya tetap bukan oracle keselamatan atau validasi hidrodinamika. |
| P2-8 | Ditangani | Tes motor, kalibrasi ESC, dan perubahan terkait output memerlukan sistem idle-disarmed. Tidak ada uji aktuator fisik. |
| P2-9 | Ditangani | Telemetri menyertakan flag kesiapan sensor/umur data; UI menampilkan kondisi offline/stale dan firmware menolak kendali saat data wajib tidak siap. |
| P3-1 | Ditangani | PlatformIO dipatok ke commit `5d4561a05e3b212660ac6fdd3fbfb328d1988aa1`. |
| P3-2 | Ditangani | Badge dan dokumentasi memisahkan compile test dari verifikasi hardware; klaim model 6-DOF/pitch dan “verified” diperbaiki. |

Temuan berikut mencatat kondisi **sebelum** perbaikan sebagai bukti review historis.

## Temuan awal

### P1 — Tangani sebelum uji bertenaga

#### P1-1 — API kontrol dapat diakses dari jaringan tanpa autentikasi

`app.py` dan server legacy `server.py` bind ke alamat kosong (`0.0.0.0`), bukan hanya loopback. Keduanya melayani `POST /api/command` dan endpoint kendali misi/parameter, dan mengizinkan CORS dari `*`. Tidak ada autentikasi, otorisasi, maupun perlindungan asal request. Jika komputer berada di LAN yang tidak sepenuhnya dipercaya, klien yang dapat mencapai port 8080 dapat meminta firmware menjalankan perintah seperti start misi, tes motor, atau perubahan parameter. Endpoint umum juga menerima perintah serial mentah.

**Dampak:** perintah aktuator dapat dipicu tanpa operator lokal; CORS memperluas kemampuan halaman web lintas-origin untuk berinteraksi dengan API.

**Saran:** bind default ke `127.0.0.1`; bila akses LAN memang dibutuhkan, tambahkan autentikasi dan otorisasi, batasi origin secara eksplisit, validasi daftar perintah/parameter di server, dan aktifkan firewall. Hindari mempublikasikan layanan ini ke jaringan umum.

Referensi: [app.py#L1030](simulasi_auv/app.py#L1030), [app.py#L1070](simulasi_auv/app.py#L1070), [app.py#L1082](simulasi_auv/app.py#L1082), [app.py#L1342](simulasi_auv/app.py#L1342), [server.py#L884](simulasi_auv/server.py#L884), [server.py#L914](simulasi_auv/server.py#L914), [server.py#L936](simulasi_auv/server.py#L936), [server.py#L1146](simulasi_auv/server.py#L1146).

#### P1-2 — Timeout injeksi tidak menghentikan FSM misi

Watchdog memanggil `stopAllThrusters()` saat injeksi sensor berhenti, tetapi tidak membatalkan `MISSION_RUNNING` dan tidak mengubah `armed` kecuali PID berdiri-sendiri aktif. Pada iterasi berikutnya loop masih melihat FSM misi aktif dan `armed`, sehingga misi dapat mengirim ulang output aktuator dengan data sensor fisik/stale. Ini berlawanan dengan komentar yang menyebut keluaran akan dijepit ke netral saat injeksi hilang.

**Dampak:** kehilangan komunikasi simulator/HIL tidak menjamin robot tetap netral.

**Saran:** saat timeout injeksi, lakukan transisi failsafe atomik yang membatalkan countdown/misi, menonaktifkan PID, mengosongkan test/calibration state, melepas arm, dan mempertahankan output netral sampai reset/arm eksplisit. Uji loss-of-link ketika PID, misi, dan tes motor masing-masing aktif.

Tidak terlihat emergency stop fisik independen pada firmware; `stop` hanya tersedia lewat serial/layanan host, sehingga perangkat keras pemutus daya/arm tetap diperlukan.

Referensi: [main.cpp#L858](src/main.cpp#L858), [main.cpp#L860](src/main.cpp#L860), [main.cpp#L895](src/main.cpp#L895), [main.cpp#L944](src/main.cpp#L944).

#### P1-3 — Misi dapat mulai meski sensor kritis tidak siap atau tare gagal

`handleStartMission()` memanggil kalibrasi depth dan IMU, tetapi mengabaikan hasil `false` dan tetap masuk ke `MISSION_RUNNING`/mengaktifkan arm. Jalur countdown juga melanjutkan peluncuran setelah tare menyatakan sensor offline. Selain itu `setup()` mengabaikan nilai balik `attachThrusters()` dan start misi tidak memeriksa `outputsReady`; kegagalan attach sebagian dapat tetap mengizinkan misi dengan output yang tidak lengkap. Pada eksekusi, depth hold bisa menghasilkan heave nol ketika pressure sensor belum siap, sementara aksi translasi tetap berjalan; tidak ada pemeriksaan freshness sensor sebelum langkah diteruskan.

**Dampak:** robot dapat menjalankan lintasan dengan depth/attitude hold tidak tersedia dan laporan misi tetap tampak berjalan.

**Saran:** buat prasyarat arm eksplisit: semua sensor yang diwajibkan oleh aksi misi siap, data cukup baru, depth sudah ditare untuk target depth, dan kalibrasi IMU valid untuk stabilisasi. Jika prasyarat gagal, tolak start atau masuk failsafe—jangan teruskan misi dengan asumsi nilai tersimpan.

Referensi: [main.cpp#L79](src/main.cpp#L79), [main.cpp#L203](src/main.cpp#L203), [main.cpp#L216](src/main.cpp#L216), [main.cpp#L217](src/main.cpp#L217), [main.cpp#L708](src/main.cpp#L708), [main.cpp#L721](src/main.cpp#L721), [main.cpp#L732](src/main.cpp#L732), [main.cpp#L802](src/main.cpp#L802).

#### P1-4 — PID pitch dihitung, tetapi tidak pernah mencapai thruster

Output pitch dihitung saat PID aktif, tetapi semua elemen kolom Pitch pada `DEFAULT_MIXER` bernilai nol. Karena itu `pitchOutput` tidak memengaruhi satu pun output motor. Mixer hanya memberi roll diferensial pada T5/T6 yang ditempatkan di tengah; dari konfigurasi ini tidak tampak lengan momen pitch. README menyatakan stabilisasi pitch sebagai fitur, tetapi kontrol tersebut tidak dapat dicapai melalui mixer ini.

**Dampak:** orientasi pitch tidak distabilkan meskipun CLI/UI melaporkan PID pitch aktif. Mengubah gain tidak memperbaiki hilangnya otoritas aktuator.

**Saran:** verifikasi geometri dan posisi aktual thruster terhadap CAD/robot. Jika memang tidak ada aktuator dengan lengan momen longitudinal, hapus klaim dan status pitch PID atau ubah desain mekanik/mixer. Uji alokasi gaya/momen di meja tanpa propeller sebelum tuning.

Referensi: [config.h#L60](src/config.h#L60), [config.h#L62](src/config.h#L62), [config.h#L63](src/config.h#L63), [config.h#L64](src/config.h#L64), [config.h#L65](src/config.h#L65), [config.h#L66](src/config.h#L66), [config.h#L67](src/config.h#L67), [config.h#L68](src/config.h#L68), [main.cpp#L389](src/main.cpp#L389), [main.cpp#L512](src/main.cpp#L512), [main.cpp#L557](src/main.cpp#L557), [README.id.md#L59](README.id.md#L59).

#### P1-5 — Jalur misi tidak divalidasi; durasi negatif dapat mengunci langkah sangat lama

Parser mengisi `power`, `durationSec`, `targetCm`, dan aksi tanpa pemeriksaan rentang, nilai finite, atau aksi yang dikenal. `UNKNOWN` tidak ditolak. Durasi kemudian dikali 1000 dan di-cast ke `uint32_t`; nilai negatif/di luar rentang representasi menyebabkan konversi floating-point ke integer tidak terdefinisi dalam C++, dan dapat menghasilkan deadline yang tidak dapat diprediksi. Daftar dengan lebih dari `MAX_STEPS` hanya dipotong dengan peringatan, sementara host tidak menegakkan batas yang sama. Daya di-clamp pada beberapa jalur kontrol tetapi data invalid tetap diterima dan dimasukkan ke jalur eksekusi.

**Dampak:** rute yang salah format/bernilai tidak masuk akal bisa menjalankan perilaku yang tidak diharapkan atau menahan misi jauh melewati durasi yang dimaksud.

**Saran:** validasi skema secara ketat sebelum mengganti rute: jumlah langkah, enum aksi, daya 0..limit, durasi positif dengan batas maksimum, target depth dalam batas pool, bitmask hanya enam bit, dan `isfinite()` untuk semua angka. Tolak seluruh rute secara atomik jika satu langkah invalid; jangan mengubah konfigurasi aktif.

Referensi: [mission.cpp#L67](src/mission.cpp#L67), [mission.cpp#L74](src/mission.cpp#L74), [mission.cpp#L93](src/mission.cpp#L93), [mission.cpp#L94](src/mission.cpp#L94), [mission.cpp#L95](src/mission.cpp#L95), [mission.cpp#L152](src/mission.cpp#L152), [mission.h#L38](src/mission.h#L38), [main.cpp#L444](src/main.cpp#L444), [main.cpp#L445](src/main.cpp#L445).

#### P1-6 — Buffer upload serial tidak dibatasi ukuran

Mode `jalur upload` mengakumulasi setiap karakter input ke `String` sampai marker EOF atau timeout 30 detik. Tidak ada batas payload sebelum append. Pada 115200 baud, sumber input dapat mengirim lebih banyak data daripada kapasitas SRAM efektif RP2040 dalam jendela tersebut; pertumbuhan `String` dan fragmentasi heap dapat menyebabkan alokasi gagal atau crash. Parser JSON yang dibatasi 4096 byte tidak melindungi buffer yang sudah terlanjur menampung payload.

**Dampak:** upload tidak valid/besar dapat menghabiskan heap dan menghentikan kendali firmware.

**Saran:** terapkan batas byte yang konservatif sebelum append, hentikan upload ketika batas tercapai, validasi panjang dan jumlah langkah, serta pastikan kegagalan upload meninggalkan rute sebelumnya tetap utuh.

Referensi: [cli.cpp#L48](src/cli.cpp#L48), [cli.cpp#L1068](src/cli.cpp#L1068), [cli.cpp#L1280](src/cli.cpp#L1280), [cli.cpp#L1291](src/cli.cpp#L1291), [mission.h#L38](src/mission.h#L38).

### P2 — Perbaiki sebelum mengandalkan misi/simulator

#### P2-1 — LittleFS otomatis diformat ketika mount gagal

Saat `LittleFS.begin()` gagal, kode langsung memanggil `LittleFS.format()`. Kegagalan mount akibat korupsi atau masalah media dapat menghapus rute yang tersimpan tanpa persetujuan atau backup; setelah itu firmware membuat rute default. Pada jalur upload, parser juga mengganti langkah aktif di RAM sebelum file dibuka, lalu hasil `serializeJsonPretty()` tidak diperiksa tetapi fungsi tetap mengembalikan sukses. Jadi kegagalan membuka/menulis dapat membuat misi RAM berbeda dari misi persisten atau memberi konfirmasi sukses palsu.

**Saran:** jangan format otomatis pada kegagalan mount. Gagal dengan status jelas dan output tetap netral; sediakan perintah format eksplisit dan backup/validasi checksum rute. Tulis ke file sementara, verifikasi jumlah byte/parse ulang, lalu tukar file dan state aktif hanya setelah commit benar-benar sukses.

Referensi: [mission.cpp#L240](src/mission.cpp#L240), [mission.cpp#L242](src/mission.cpp#L242), [mission.cpp#L123](src/mission.cpp#L123), [mission.cpp#L303](src/mission.cpp#L303), [mission.cpp#L315](src/mission.cpp#L315), [mission.cpp#L321](src/mission.cpp#L321), [mission.cpp#L323](src/mission.cpp#L323).

#### P2-2 — Polaritas dan posisi T5/T6 berbeda antara firmware, simulator, dan README

Firmware menetapkan reverse T5/T6 sebagai `0,1` dan posisi T5 kanan/T6 kiri. `hydrodynamics.js` menggunakan reverse `1,0`; `sim3d.js` menukar posisi, blade rotation, serta reverse untuk pasangan vertikal. README justru menulis arah putaran T5/T6 berlawanan dengan komentar firmware. Akibatnya digital twin dapat memprediksi arah dorong berbeda dari firmware/robot, dan tabel hardware tidak konsisten.

**Dampak:** simulasi atau checklist arah motor dapat memberi keyakinan palsu tentang arah dive/surface dan respons roll.

**Saran:** tetapkan satu sumber konfigurasi bersama untuk posisi, reverse, orientasi, dan arah dorong; sinkronkan dokumentasi dan simulator. Konfirmasi arah fisik dengan propeller dilepas dan lakukan uji dorong terikat sebelum air.

Referensi: [config.h#L26](src/config.h#L26), [config.h#L27](src/config.h#L27), [config.h#L60](src/config.h#L60), [README.id.md#L90](README.id.md#L90), [README.id.md#L91](README.id.md#L91), [hydrodynamics.js#L380](simulasi_auv/hydrodynamics.js#L380), [sim3d.js#L76](simulasi_auv/sim3d.js#L76), [sim3d.js#L77](simulasi_auv/sim3d.js#L77).

#### P2-3 — Serial reader dan uploader tidak tersinkronisasi

`upload_mission_steps()` mengambil `cmd_lock`, tetapi worker thread tetap membaca `self.ser.readline()` tanpa lock yang sama. Kedua jalur dapat mengonsumsi byte respons satu sama lain; khususnya `[UPLOAD] READY` atau `[UPLOAD] SUCCESS` dapat lebih dulu diambil worker, membuat upload timeout atau status serial tidak sinkron.

**Saran:** jadikan satu thread pemilik tunggal objek serial; kirim request melalui antrean dan korelasikan respons berdasarkan state/protokol, termasuk upload.

Referensi: [app.py#L457](simulasi_auv/app.py#L457), [app.py#L483](simulasi_auv/app.py#L483), [app.py#L807](simulasi_auv/app.py#L807), [app.py#L864](simulasi_auv/app.py#L864), [app.py#L876](simulasi_auv/app.py#L876).

#### P2-4 — Nilai PID dan konfigurasi ESC dapat tidak konsisten/tidak aman

CLI menerima gain PID tanpa batas finite atau rentang, dan setter min/max/neutral thruster memvalidasi tiap nilai sendiri-sendiri, bukan invariant `pulseMin < pulseNeutral < pulseMax`. Load EEPROM juga memeriksa rentang individual tetapi tidak memeriksa relasi tersebut. Nilai seperti gain negatif/non-finite atau rentang pulsa yang saling silang dapat menghasilkan kendali atau pembatasan pulsa yang tidak masuk akal.

**Saran:** validasi seluruh struktur sebagai satu unit sebelum diterapkan dan sebelum disimpan; terapkan batas PID yang ditetapkan lewat tuning aman, finite check, serta invariant pulse min < neutral < max.

Referensi: [cli.cpp#L658](src/cli.cpp#L658), [cli.cpp#L666](src/cli.cpp#L666), [cli.cpp#L673](src/cli.cpp#L673), [cli.cpp#L680](src/cli.cpp#L680), [cli.cpp#L878](src/cli.cpp#L878), [config.h#L216](src/config.h#L216), [config.h#L219](src/config.h#L219), [config.h#L222](src/config.h#L222).

#### P2-5 — Data misi masuk ke `innerHTML` di dashboard konfigurasi

`config.html` membangun tabel misi dengan `innerHTML` dari nilai `name`, `action`, dan properti langkah lainnya. Nama langkah dapat berasal dari file misi/upload. Bila halaman legacy ini digunakan dan nilai tersebut tidak dipercaya, markup aktif dapat dieksekusi di origin dashboard.

**Saran:** bangun elemen DOM dan isi data dengan `textContent`/atribut yang divalidasi; jangan interpolasi konten misi ke `innerHTML`.

Referensi: [config.html#L2052](simulasi_auv/config.html#L2052), [config.html#L2059](simulasi_auv/config.html#L2059).

#### P2-6 — Dua server berbeda berbagi port, dan launcher konfigurasi dapat salah memilih server

`app.py` dan `server.py` sama-sama bind ke port 8080. Launcher konfigurasi melakukan probe `/api/state`, yang juga tersedia pada server utama, lalu dapat menganggap server yang salah sudah berjalan. Pada server utama, `/config` dan `/config.html` diarahkan ke `index.html`, bukan `config.html`. Menjalankan launcher/server yang berbeda dapat menghasilkan port conflict atau UI/config API yang tidak sesuai.

**Saran:** satukan entry point atau tetapkan port dan kontrak API yang berbeda; buat launcher memeriksa identitas/version endpoint server, bukan hanya status 200.

Referensi: [app.py#L1065](simulasi_auv/app.py#L1065), [app.py#L1342](simulasi_auv/app.py#L1342), [server.py#L908](simulasi_auv/server.py#L908), [server.py#L1146](simulasi_auv/server.py#L1146), [start_konfigurasi.py#L18](simulasi_auv/start_konfigurasi.py#L18), [start_konfigurasi.py#L32](simulasi_auv/start_konfigurasi.py#L32).

#### P2-7 — Simulator bukan oracle keselamatan untuk firmware

`hydrodynamics.js` mengklaim model Fossen/6-DOF, tetapi implementasi yang ditinjau menggunakan dinamika planar surge/heave/pitch ditambah yaw sederhana; model sway dan roll tidak menjadi state/dinamika terkopel setara. `rl_agent.py` juga mencari kandidat gain lewat sampling acak dan scoring simulasi depth 1D, bukan policy Actor-Critic penuh sebagaimana narasi file. Ditambah mismatch polaritas T5/T6, simulasi tidak dapat dipakai sebagai bukti bahwa mixer/PID aman pada robot fisik.

**Saran:** labeli dengan jelas parameter/DOF yang benar-benar dimodelkan; uji closed-loop di SIL/HIL dengan fixture, log sensor/aktuator, dan ground truth thruster sebelum menyebut hasil sebagai validasi fisik.

Referensi: [hydrodynamics.js#L368](simulasi_auv/hydrodynamics.js#L368), [hydrodynamics.js#L485](simulasi_auv/hydrodynamics.js#L485), [hydrodynamics.js#L536](simulasi_auv/hydrodynamics.js#L536), [rl_agent.py#L206](simulasi_auv/rl_agent.py#L206), [rl_agent.py#L293](simulasi_auv/rl_agent.py#L293).

#### P2-8 — Tes motor/kalibrasi ESC dapat menimpa output misi aktif

Perintah `tes` memulai output test tanpa menolak kondisi `MISSION_RUNNING`; loop utama memprioritaskan test pulse selama dua detik, kemudian misi berjalan kembali dari langkah/timer yang sama. Perintah kalibrasi ESC/raw pulse juga tidak memiliki interlock FSM yang mencegahnya saat misi berjalan.

**Dampak:** satu perintah test atau kalibrasi dapat mendadak mengganti kombinasi thrust di tengah misi dan membuat misi melanjutkan setelah gangguan.

**Saran:** tolak tes motor/ESC saat countdown, misi, atau PID aktif; atau lakukan transisi disarm/cancel yang eksplisit sebelum operasi test, dan minta konfirmasi operator.

Referensi: [main.cpp#L144](src/main.cpp#L144), [main.cpp#L172](src/main.cpp#L172), [main.cpp#L643](src/main.cpp#L643), [main.cpp#L661](src/main.cpp#L661), [main.cpp#L895](src/main.cpp#L895), [main.cpp#L944](src/main.cpp#L944).

#### P2-9 — Status kesiapan sensor tidak dibawa ke telemetri dashboard

Firmware memiliki flag `imuReady` dan `pressureReady`, tetapi paket `$TEL` hanya mengirim nilai numerik sensor dan tidak mengirim flag kesiapan. Bridge mengisi angka tersebut ke state tanpa status validitas. Akibatnya dashboard dapat menampilkan kedalaman nol/sudut nol sebagai data normal ketika sensor tidak siap, sehingga operator tidak dapat membedakan nilai valid dari fallback/stale.

**Saran:** sertakan flag ready dan timestamp/umur sampel dalam telemetri/API; tampilkan sensor offline/stale secara eksplisit dan cegah kontrol misi jika data yang dibutuhkan tidak valid.

Referensi: [sensors.h#L10](src/sensors.h#L10), [sensors.h#L12](src/sensors.h#L12), [cli.cpp#L1210](src/cli.cpp#L1210), [cli.cpp#L1221](src/cli.cpp#L1221), [app.py#L302](simulasi_auv/app.py#L302), [app.py#L329](simulasi_auv/app.py#L329).

### P3 — Kualitas dan reproducibility

#### P3-1 — Platform PlatformIO tidak dipatok ke versi/commit

`platformio.ini` mengambil platform dari URL repository Git tanpa tag/commit tetap. Versi platform/core/toolchain dapat berubah ketika build berikutnya dilakukan, walaupun file firmware dan kode sumber sama.

**Saran:** pin platform ke release/commit yang diuji; catat versi framework, compiler, dan dependency dalam proses release.

Referensi: [platformio.ini#L6](platformio.ini#L6), [platformio.ini#L11](platformio.ini#L11).

#### P3-2 — Dokumentasi klaim keamanan/build lebih kuat daripada bukti yang tersedia

README menyebut 500 ms serial watchdog, pitch stabilization, 6-DOF, dan build “Verified (0 Warnings)”. Build lokal memang berhasil tanpa warning yang terlihat, tetapi watchdog memiliki celah pada misi aktif, pitch mixer tidak punya otoritas, dan simulator tidak memodelkan 6-DOF penuh. Klaim tersebut perlu diperjelas agar operator tidak menyamakan simulasi/build dengan verifikasi keselamatan.

**Saran:** perbarui README setelah P1/P2 ditangani; pisahkan status compile, unit/integration test, bench test, dan wet test.

Referensi: [README.id.md#L9](README.id.md#L9), [README.id.md#L59](README.id.md#L59), [README.id.md#L63](README.id.md#L63).

## Peta area

| Area | File utama | Catatan review |
|---|---|---|
| Firmware main loop, motor output, FSM, PID | [src/main.cpp](src/main.cpp) | Failsafe injeksi, preflight sensor, output/interlock; pitch nonaktif |
| Konfigurasi EEPROM, limit, mixer | [src/config.h](src/config.h) | Validasi konfigurasi, invariant pulse, dan fallback |
| Parser/eksekusi misi dan LittleFS | [src/mission.cpp](src/mission.cpp), [src/mission.h](src/mission.h) | Validasi ketat, batas upload, staging persisten |
| Sensor IMU/pressure dan injection | [src/sensors.cpp](src/sensors.cpp), [src/sensors.h](src/sensors.h) | Sensor init/retry dan data injeksi |
| Serial CLI/protokol upload | [src/cli.cpp](src/cli.cpp), [src/cli.h](src/cli.h) | Input tervalidasi, batas upload, interlock test/ESC |
| API bridge utama | [simulasi_auv/app.py](simulasi_auv/app.py) | Loopback, validasi API, dan serial terkoordinasi |
| Server dashboard legacy | [simulasi_auv/server.py](simulasi_auv/server.py) | Loopback dan port terpisah |
| UI kontrol dan misi | [simulasi_auv/index.html](simulasi_auv/index.html), [simulasi_auv/config.html](simulasi_auv/config.html) | Rendering teks aman; status sensor terlihat |
| Model fisika/visual | [simulasi_auv/hydrodynamics.js](simulasi_auv/hydrodynamics.js), [simulasi_auv/sim3d.js](simulasi_auv/sim3d.js), [simulasi_auv/rl_agent.py](simulasi_auv/rl_agent.py) | Tetap model eksperimental, bukan validasi fisik |
| Launcher dan route sampel | [simulasi_auv/start.bat](simulasi_auv/start.bat), [simulasi_auv/start_simulasi.py](simulasi_auv/start_simulasi.py), [simulasi_auv/start_konfigurasi.py](simulasi_auv/start_konfigurasi.py), [simulasi_auv/jalur.json](simulasi_auv/jalur.json) | Probe server konfigurasi dengan endpoint khusus |
| Build dan hardware test tools | [platformio.ini](platformio.ini), [tools/](tools/) | Build lokal lulus; skrip tes/flashing berinteraksi dengan hardware |
| Binary, STL, gambar, vendor JS | `firmware/`, `simulasi_auv/assets/`, `simulasi_auv/libs/`, `docs/images/` | Binary firmware diperbarui dari build lokal; aset/vendor tidak diaudit semantik internalnya |

## Validasi yang dilakukan

- `pio run`: **PASS** untuk Pico W / RP2040 dengan platform commit yang dipatok. Pemakaian: RAM 73,116 / 262,144 byte (27.9%); Flash 412,096 / 1,044,480 byte (39.5%).
- Binary `.uf2` dan `.bin` di `firmware/` diperbarui; SHA-256 masing-masing cocok dengan output build lokal.
- Python AST: **PASS** untuk seluruh file Python yang berubah.
- `node --check`: **PASS** untuk `hydrodynamics.js` dan `sim3d.js`; parser JavaScript inline **PASS** untuk `index.html` dan `config.html`.
- Uji DOM dashboard: **PASS**; teks langkah yang menyerupai HTML tetap menjadi teks dan kontrol konfigurasi beserta ID-nya terbentuk.
- Parse `simulasi_auv/jalur.json`: **PASS**, 5 langkah.
- `git diff --check` dijalankan dengan penanganan CRLF (`core.whitespace=cr-at-eol`): **PASS**.
- Tidak ada tes hardware, flashing, propeller, HIL, atau wet test. Skrip `tools/flash_and_test*.py` dapat menggerakkan thruster; jangan menjalankannya tanpa prosedur keselamatan, propeller dilepas, dan pemutus daya fisik independen.

## Referensi eksternal yang diperiksa

- [Arduino-Pico Servo Library](https://arduino-pico.readthedocs.io/en/latest/servo.html): Servo menggunakan PIO; hingga 8 servo paralel dengan asumsi PIO tidak dipakai resource lain. Enam output secara kapasitas didukung oleh core yang dipilih, tetapi tetap perlu verifikasi board-level.
- [Arduino-Pico Wire/I2C](https://arduino-pico.readthedocs.io/en/latest/wire.html): RP2040 memiliki `Wire` dan `Wire1`; pemilihan SDA/SCL harus dilakukan sebelum `begin()` dan pin harus mendukung I2C1. Urutan yang digunakan firmware sesuai dokumentasi ini.
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) dan [Pico pinout](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf): rujukan untuk batas MCU dan pemetaan GPIO; verifikasi akhir pin harus terhadap board/wiring fisik yang dipakai.
- Datasheet MS5803 dan MPU-6050 tidak berhasil diambil oleh pembaca web pada sesi ini; perhitungan kompensasi sensor dan skala sensor karena itu belum disahkan terhadap revisi datasheet/komponen breakout yang benar.

## Validasi yang masih wajib dilakukan di hardware

1. Konfirmasi wiring, pin, orientasi/polaritas, posisi, arah dorong, dan otoritas roll pada robot/CAD.
2. Uji failsafe kehilangan serial/injeksi, sensor stale, reset, dan kegagalan sensor dengan propeller dilepas.
3. Uji upload berulang, putus sambung USB, dan pemulihan LittleFS tanpa kehilangan rute.
4. Lakukan bench test/HIL bertahap dengan fixture dan kill switch fisik independen; wet test hanya setelah hasil bench/HIL ditinjau.
