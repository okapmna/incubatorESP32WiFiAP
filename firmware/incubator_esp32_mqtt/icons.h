/**
 * @file icons.h
 * @brief Bitmap ikon untuk layar TFT inkubator (monokrom 1-bit, 16x11 piksel).
 *
 * Format: baris demi baris, MSB di kiri, 2 byte per baris (16 piksel).
 * Digambar dengan tft.drawBitmap(x, y, bitmap, w, h, warna) -- bit 1 digambar, bit 0 transparan.
 * Simpan file ini di folder yang sama dengan incubator_firmware.ino.
 *
 * Untuk mengganti ikon: gambar ulang di image2cpp (mode "Arduino code, single bitmap",
 * ukuran sesuai ICON_W x ICON_H) lalu tempel array hasilnya di sini.
 */

#ifndef ICONS_H
#define ICONS_H

#include <Arduino.h>

#define ICON_W 16
#define ICON_H 11

// Ikon WiFi (3 busur + titik). Dipakai untuk status terhubung dan portal aktif;
// beda status dibedakan lewat warna.
//
//  ....########....
//  ..###......###..
//  .##..........##.
//  ##...######...##
//  ...###....###...
//  ..##........##..
//  ......####......
//  .....##..##.....
//  ................
//  .......##.......
//  .......##.......
static const unsigned char ICON_WIFI[] PROGMEM = {
  0x0F, 0xF0,
  0x38, 0x1C,
  0x60, 0x06,
  0xC7, 0xE3,
  0x1C, 0x38,
  0x30, 0x0C,
  0x03, 0xC0,
  0x06, 0x60,
  0x00, 0x00,
  0x01, 0x80,
  0x01, 0x80
};

// Garis coret diagonal. Digambar di atas ICON_WIFI (warna merah) untuk status terputus.
//
//  ..##............
//  ...##...........
//  ....##..........
//  .....##.........
//  ......##........
//  .......##.......
//  ........##......
//  .........##.....
//  ..........##....
//  ...........##...
//  ............##..
static const unsigned char ICON_WIFI_SLASH[] PROGMEM = {
  0x30, 0x00,
  0x18, 0x00,
  0x0C, 0x00,
  0x06, 0x00,
  0x03, 0x00,
  0x01, 0x80,
  0x00, 0xC0,
  0x00, 0x60,
  0x00, 0x30,
  0x00, 0x18,
  0x00, 0x0C
};

// 16x16 Temperature Icon Bitmap
const unsigned char icon_temperature[] PROGMEM = {
    0x03, 0xC0, //     ####     
    0x02, 0x40, //     #  #     
    0x02, 0x40, //     #  #     
    0x02, 0x40, //     #  #     
    0x02, 0x40, //     #  #     
    0x02, 0x40, //     #  #     
    0x02, 0x40, //     #  #     
    0x0E, 0x70, //   ###  ###   
    0x12, 0x48, //  #  #  #  #  
    0x12, 0x48, //  #  #  #  #  
    0x22, 0x44, // #   #  #   # 
    0x20, 0x04, // #          # 
    0x20, 0x04, // #          # 
    0x10, 0x08, //  #        #  
    0x1F, 0xF8, //  ##########  
    0x00, 0x00  //              
};

// 16x16 Humidity Icon Bitmap
const unsigned char icon_humidity[] PROGMEM = {
    0x01, 0x80, //       ##     
    0x02, 0x40, //      #  #    
    0x02, 0x40, //      #  #    
    0x04, 0x20, //     #    #   
    0x04, 0x20, //     #    #   
    0x08, 0x10, //    #      #  
    0x08, 0x10, //    #      #  
    0x10, 0x08, //   #        # 
    0x10, 0x08, //   #        # 
    0x20, 0x04, //  #          #
    0x20, 0x04, //  #          #
    0x20, 0x04, //  #          #
    0x10, 0x08, //   #        # 
    0x10, 0x08, //   #        # 
    0x0F, 0xF0, //    ########  
    0x00, 0x00  //              
};


#endif // ICONS_H