#pragma once
#include <Arduino.h>
#include <strings.h>

// ==============================================================================
// PROFIL INKUBASI PER JENIS TELUR
// ==============================================================================
// Nilai di bawah adalah acuan umum untuk inkubator udara paksa (forced-air).
// Sesuaikan dengan panduan spesies/strain dan karakteristik alat Anda.
//
// Cara menambah jenis telur: tambahkan baris baru SEBELUM baris "KUSTOM".
//   - name   : nama tampil di LCD (maks 7 huruf KAPITAL agar muat di layar)
//   - days   : lama inkubasi (hari)
//   - temp   : target suhu (deg C), batas menu 20.0 - 50.0
//   - hum    : target kelembapan (%), batas menu 30 - 90
//   - lock   : jumlah hari TERAKHIR sebelum tetas (fase lockdown): pemutaran telur
//              dihentikan (servo diam di posisi tengah) dan suhu diturunkan.
//              0 = tidak ada fase lockdown.
//   - drop   : penurunan suhu selama lockdown (deg C), mis. 0.5 -> 37.5 jadi 37.0
//   - custom : true = tidak menimpa suhu/lembab/hari (pengaturan manual)
//
// PENTING: profil "KUSTOM" harus tetap di urutan TERAKHIR.

struct EggProfile {
  const char* name;
  uint8_t     days;
  float       temp;
  uint8_t     hum;
  uint8_t     lock;
  float       drop;
  bool        custom;
};

static const EggProfile EGG_PROFILES[] = {
  //  nama       hari  suhu   lembab  lock  drop   custom
  { "AYAM",       21,  37.5f,  55,    3,   0.5f,  false },
  { "BEBEK",      28,  37.5f,  60,    3,   0.5f,  false },
  { "ENTOK",      35,  37.5f,  60,    4,   0.5f,  false },
  { "PUYUH",      17,  37.5f,  50,    3,   0.5f,  false },
  { "ANGSA",      30,  37.5f,  55,    3,   0.5f,  false },
  { "KALKUN",     28,  37.5f,  55,    3,   0.5f,  false },
  { "MERPATI",    18,  37.5f,  55,    3,   0.5f,  false },
  { "KUSTOM",     21,  37.5f,  55,    3,   0.5f,  true  },   // <-- harus terakhir
};

#define EGG_PROFILE_COUNT   ((int)(sizeof(EGG_PROFILES) / sizeof(EGG_PROFILES[0])))
#define EGG_CUSTOM_INDEX    (EGG_PROFILE_COUNT - 1)

// Batas target hari inkubasi yang bisa diatur
#define EGG_DAYS_MIN   1
#define EGG_DAYS_MAX   60

/** @brief Mencari indeks profil dari nama (tidak peka huruf besar/kecil). -1 bila tidak ada. */
inline int egg_find(const char* name) {
  if (!name) return -1;
  for (int i = 0; i < EGG_PROFILE_COUNT; i++) {
    if (strcasecmp(name, EGG_PROFILES[i].name) == 0) return i;
  }
  return -1;
}