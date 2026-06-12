#include "HEPTA_GS_Telemetry.h"

// ============================================================
// 内部用の制限関数
// ============================================================

static short clamp_i16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (short)value;
}

static unsigned short clamp_u16(float value)
{
    if (value > 65535.0f) return 65535;
    if (value < 0.0f) return 0;
    return (unsigned short)value;
}


// ============================================================
// データ変換関数
// ------------------------------------------------------------
// センサで取得した値を，地上局ソフトで扱うための整数値に変換します．
// 変換係数などの細かい設定は，ここにまとめています．
// ============================================================

unsigned short encode_battery(float value)
{
    return clamp_u16(value / (3.3f * 1.431f) * 4096.0f);
}

short encode_temperature(float value)
{
    return clamp_i16(value * 10.0f);
}

short encode_acceleration(float value)
{
    return clamp_i16(value / 9.8f * 512.0f);
}

short encode_gyro(float value)
{
    return clamp_i16(value * 2048.0f / 125.0f);
}

short encode_magnetic(float value)
{
    return clamp_i16(value);
}


// ============================================================
// 16 bitデータを2 byteに分けてpacket[]へ入れる
// ------------------------------------------------------------
// 例：0x1234 を，0x12 と 0x34 の2 byteに分けて入れます．
// ============================================================

void put_u16(unsigned char *packet,
             int *idx,
             unsigned short value)
{
    packet[(*idx)++] = (value >> 8) & 0xFF;
    packet[(*idx)++] = value & 0xFF;
}

void put_i16(unsigned char *packet,
             int *idx,
             short value)
{
    put_u16(packet, idx, (unsigned short)value);
}


// ============================================================
// チェックサム計算
// ------------------------------------------------------------
// Header 0x7E は除いて，それ以降のbyteを足し合わせます．
// ============================================================

unsigned char calc_checksum(unsigned char *packet,
                            int length)
{
    unsigned char sum = 0;

    for (int i = 1; i < length; i++) {
        sum += packet[i];
    }

    return sum;
}


// ============================================================
// 地上局からのコマンド確認
// ------------------------------------------------------------
// 'a'：ミッションモード開始
// 'b'：ミッションモード停止
// ============================================================

void check_ground_command(HEPTA_COM *com,
                          bool *mission_mode)
{
    int rcmd = 0;
    int cmdflag = 0;

    com->xbee_receive(&rcmd, &cmdflag);

    if (cmdflag == true) {

        if (rcmd == 'a') {
            *mission_mode = true;
        }

        if (rcmd == 'b') {
            *mission_mode = false;
        }
    }
}


// ============================================================
// packet[] をHEX文字列として送信する
// ------------------------------------------------------------
// packet[] の中身は，バイナリパケットと同じ構造です．
//
// ただし今回は教材用に，Tera Termで確認できるよう
// 各byteを "7E 01 11 ..." の文字列として送信します．
// ============================================================

void send_packet_as_hex_text(HEPTA_COM *com,
                             unsigned char *packet,
                             int length)
{
    char text[4];

    for (int i = 0; i < length; i++) {

        sprintf(text, "%02X", packet[i]);

        com->putc(text[0]);
        com->putc(text[1]);

        if (i < length - 1) {
            com->putc(' ');
        }
    }

    com->putc('\r');
    com->putc('\n');
}


// ============================================================
// PC確認用表示
// ============================================================

void print_packet_hex(RawSerial *pc,
                      unsigned char *packet,
                      int length)
{
    for (int i = 0; i < length; i++) {
        pc->printf("%02X ", packet[i]);
    }

    pc->printf("\r\n");
}