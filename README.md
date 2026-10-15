# Inverse Kinematics dengan CCD - Lengan 3 Segmen

Tugas UTS mata kuliah **Animasi Komputer**. Program Python + raylib yang menggerakkan
lengan 3 segmen agar ujungnya selalu mengikuti posisi mouse, dengan metode
**Cyclic Coordinate Descent (CCD)**.

Nama / NIM: Faris Maulana Al Ba I / 1313624004

## Fitur

- Lengan 3 segmen (panjang 130, 100, 70 piksel) dengan ujung yang mengikuti kursor mouse.
- **Mode otomatis**: CCD dijalankan tiap frame (maks. 10 iterasi, *warm start* dari pose frame sebelumnya).
- **Mode langkah demi langkah**: satu sendi per langkah, dengan vektor *a* (pivot → ujung lengan)
  dan *b* (pivot → target) beserta sudut putar (delta) yang akan diterapkan.
- Grafik konvergensi: jarak ujung ke target pada tiap iterasi (skala logaritmik).
- Batas sendi (opsional): sendi 2 dan 3 dibatasi ±100°.
- Penanda target di luar jangkauan: lengan meregang lurus ke arah target.

## Persyaratan dan instalasi

- Python 3.7 atau lebih baru (memakai `dataclasses`) dan paket `raylib`.

```
pip install raylib
```

Paket `raylib` menyediakan modul `pyray`. Jangan memasang paket bernama `pyray` dari PyPI:
itu paket lain yang tidak berkaitan dengan raylib.

Memakai virtual environment bersifat opsional. Contoh di Windows:

```
python -m venv myvenv
myvenv\Scripts\activate
pip install raylib
```

Di Linux/macOS aktivasinya `source myvenv/bin/activate`.

## Menjalankan

```
python main.py
```

Jalankan dari dalam folder proyek ini.

## Kontrol

| Tombol | Fungsi |
|---|---|
| Mouse | Posisi target |
| `TAB` | Ganti mode otomatis ↔ langkah demi langkah |
| `SPACE` | (mode langkah) terapkan satu langkah CCD pada sendi aktif |
| `L` | Batas sendi hidup/mati |
| `G` | Tampilkan/sembunyikan grafik konvergensi (mode otomatis) |
| `R` | Reset pose awal |
| `ESC` | Keluar |

## Struktur modul

| File | Tanggung jawab |
|---|---|
| `config.py` | Konstanta: panjang segmen, pose awal, batas sendi, parameter CCD, ukuran jendela |
| `kinematics.py` | Forward kinematics `fk()`, `wrap_angle()`, `reach_range()` |
| `ccd_solver.py` | Algoritma CCD: `joint_delta()`, `ccd_joint_step()`, `ccd()` |
| `render.py` | Konversi koordinat dunia ↔ layar dan semua fungsi menggambar (raylib) |
| `main.py` | Loop aplikasi, input keyboard/mouse, mode otomatis dan langkah |
| `test_ik.py` | Uji mandiri tanpa raylib |

`kinematics.py` dan `ccd_solver.py` tidak bergantung pada raylib, sehingga bisa diuji
langsung di terminal. Aturan CCD hanya ditulis di satu tempat (`ccd_joint_step`):
fungsi `ccd()` dan mode langkah sama-sama memanggilnya.

## Cara kerja singkat

**Forward kinematics.** Sudut tiap sendi bersifat lokal (relatif terhadap segmen sebelumnya).
Sudut global segmen ke-i adalah jumlah sudut lokal sampai sendi ke-i; posisi tiap sendi dihitung
dengan menjumlahkan `panjang × (cos, sin)` sudut globalnya.

**CCD.** Satu iterasi memproses sendi dari yang terluar ke yang terdalam. Pada sendi *i*:

1. *a* = vektor dari pivot sendi *i* ke ujung lengan; *b* = vektor dari pivot yang sama ke target.
2. Sudut putar `delta = atan2(a × b, a · b)` membuat *a* sejajar dengan *b*
   (tanda cross menentukan arah putar).
3. Sudut sendi ditambah `delta`, dibungkus ke (−π, π], lalu dijepit bila ada batas sendi.
4. FK dihitung ulang sebelum sendi berikutnya diproses.

Iterasi diulang sampai jarak ujung ke target < `1e-3` atau jumlah iterasi maksimum tercapai.

**Konvensi koordinat.** Koordinat dunia memakai sumbu y ke atas dan sudut positif
berlawanan arah jarum jam (sama dengan materi kuliah). Konversi ke layar (sumbu y ke bawah)
hanya terjadi di `render.py`.

## Pengujian

```
python test_ik.py
```

Berisi 10 uji, antara lain: FK cocok dengan contoh materi P10 (L = 2, 1; θ = 30°, 45° → ujung ≈ (1,991; 1,966)),
satu langkah CCD yang dihitung dengan tangan, contoh buku Parent (lengan 15, 10, 5; target (−20, 5)) yang
konvergen dalam 4 iterasi dengan sudut akhir ≈ 146,3°, 87,4°, −125,1°, perilaku target di luar jangkauan,
kepatuhan batas sendi, dan kesetaraan `ccd(max_iter=1)` dengan tiga langkah manual.

## Konfigurasi

Semua nilai ada di `config.py`:

| Konstanta | Arti |
|---|---|
| `LENGTHS` | Panjang tiap segmen |
| `START_ANGLES` | Pose awal (radian); sengaja tidak lurus |
| `LIMITS_ON` | Batas sendi yang dipakai saat `L` aktif |
| `MAX_ITER_PER_FRAME` | Iterasi CCD maksimum per frame |
| `TOL` | Toleransi jarak untuk dianggap konvergen |

## Perilaku yang perlu diketahui

- Target di luar jangkauan (jarak > total panjang segmen): CCD tidak gagal; lengan meregang lurus
  ke arah target dan jarak sisa tetap.
- Saat mouse bergerak, jumlah iterasi per frame hampir selalu mencapai maksimum karena target terus berpindah.
- Dengan batas sendi aktif, ujung lengan tidak dapat mendekati pangkal lebih dekat dari sekitar 88 piksel.
- CCD bergantung pada pose awal: lengan yang lurus sempurna dengan target tepat di belakang pangkal dapat
  tidak konvergen dalam 10 iterasi. Karena itu pose awal dibuat tidak lurus.

## Rujukan

- Parent, R. (2012). *Computer Animation: Algorithms and Techniques*, Bab 5.3.
- Materi kuliah Animasi Komputer, Pertemuan 10–11 (Inverse Kinematics: Analitis & CCD).