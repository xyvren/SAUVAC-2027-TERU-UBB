# Panduan Hardware-in-the-Loop (HIL)

Mode HIL menghubungkan model kolam virtual ke output ESC pada Pico W. Mode ini terpisah dari simulasi virtual biasa dan selalu nonaktif saat aplikasi dibuka.

## Batas dan failsafe

- HIL hanya menerima enam pulsa dalam rentang **1425-1575 µs** (maksimum ±75 µs dari netral 1500 µs).
- Pico W mematikan HIL dan menetralkan thruster jika heartbeat serial terputus lebih dari **350 ms**. Injeksi sensor virtual memiliki watchdog firmware tambahan.
- Firmware hanya mengizinkan HIL saat output siap, sistem idle/disarmed, autorun off, injeksi sensor belum aktif, dan semua nilai netral thruster tepat 1500 µs.
- Simulasi mengirim sensor virtual melalui `$INJ` dan pulsa aktuator terbatas melalui `$HIL`; `$ACT` dari Pico dipakai kembali sebagai input dinamika. HIL ini menguji jalur sensor/aktuator dan arah respons model, **bukan** menguji kontrol PID atau mengukur gaya dorong underwater.
- Jangan menafsirkan gerak model sebagai pengukuran thrust nyata. Validasi thrust di air memerlukan fixture/tangki dan pengukuran tersendiri.

## Prosedur bench

1. Lepas semua propeller. Ikat rangka agar tidak bergerak dan siapkan cutoff daya ESC yang dapat dijangkau tanpa mendekati thruster.
2. Pastikan area bebas, catu daya dan konektor benar, serta tidak ada misi/tes motor aktif. Jangan gunakan prosedur ini di dekat orang atau benda yang dapat terluka/rusak.
3. Build firmware dari source terbaru dengan `pio run -e picow`, lalu flash UF2 hasil build (`.pio/build/picow/firmware.uf2`). UF2 lama dapat belum memiliki perintah HIL. Setelah reboot, verifikasi hanya dengan `help` dan `hil status`; bantuan harus mencantumkan `hil <on|off|status>` dan status harus `DISARMED`. Jangan kirim `hil on` hanya untuk memeriksa versi. Hindari `tools/flash_and_verify.py` untuk verifikasi ini karena skrip tersebut menjalankan tes thruster.
4. Jalankan aplikasi simulator dan sambungkan Pico W. Buka tab simulasi, centang konfirmasi keselamatan, lalu tekan **ARM HIL**. Jika kondisi interlock tidak terpenuhi, arm harus ditolak.
5. Jalankan misi kolam virtual atau gunakan kemudi virtual. Output fisik tetap dibatasi ±75 µs; gunakan **STOP / DISARM** untuk menetralkan segera.
6. Jika tab ditutup, browser disembunyikan, serial terputus, atau heartbeat berhenti, firmware harus menetralkan output lewat watchdog. Verifikasi pulsa kembali netral sebelum melepas rangka.

Perintah firmware `hil on` tersedia untuk bench serial dan tetap tunduk pada interlock firmware. Perintah `stop` selalu membatalkan mode HIL dan menetralkan semua output.
