#ifndef HEPTA_GS_TELEMETRY_H
#define HEPTA_GS_TELEMETRY_H

#include "mbed.h"
#include "HEPTA_COM.h"

// ============================================================
// パケットサイズ・設定
// ============================================================

#define PACKET_MAX_SIZE     30

// この値未満になると省電力モードに入る
#define POWER_SAVE_VOLTAGE  3.6f


// ============================================================
// データ変換関数
// ============================================================

unsigned short encode_battery(float value);

short encode_temperature(float value);

short encode_acceleration(float value);

short encode_gyro(float value);

short encode_magnetic(float value);


// ============================================================
// パケット作成用補助関数
// ============================================================

void put_u16(unsigned char *packet,
             int *idx,
             unsigned short value);

void put_i16(unsigned char *packet,
             int *idx,
             short value);

unsigned char calc_checksum(unsigned char *packet,
                            int length);


// ============================================================
// 動作補助関数
// ============================================================

void check_ground_command(HEPTA_COM *com,
                          bool *mission_mode);

void send_packet_as_hex_text(HEPTA_COM *com,
                             unsigned char *packet,
                             int length);

void print_packet_hex(RawSerial *pc,
                      unsigned char *packet,
                      int length);

#endif