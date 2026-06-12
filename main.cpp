#include "mbed.h"
#include "HEPTA_EPS.h"
#include "HEPTA_SENSOR.h"
#include "HEPTA_COM.h"
#include "HEPTA_GS_Telemetry.h"


RawSerial pc(USBTX, USBRX, 9600);
HEPTA_EPS eps(p16, p26);
HEPTA_SENSOR sensor(p17,
                    p28, p27, 0x19, 0x69, 0x13,
                    p13, p14, p25, p24);
HEPTA_COM com(p9, p10, 9600);


// 初期設定（スタンバイモード，受信パケット数0）
bool mission_mode = false;
unsigned short telemetry_seq = 0;


int main()
{
    float battery_voltage = 0.0f;
    float temperature = 0.0f;

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;

    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    unsigned char packet[PACKET_MAX_SIZE];

    pc.printf("HEPTA Ground Station Telemetry Start\r\n");

    while (1) {

        // モードに関わらず取得する基本データ，温度と電圧を取得
        eps.vol(&battery_voltage);
        sensor.temp_sense(&temperature);

        // 地上局からコマンドが送られてきていないか確認
        check_ground_command(&com, &mission_mode);


        // パケットタイプ（ミッションパケットは9軸センサが追加される）初期化
        // 衛星モード（スタンバイ，ミッション，省電力）初期化
        unsigned char packet_type = 0x10;
        unsigned char mode_byte   = 0x00;

        // 省電力モード設定
        if (battery_voltage < POWER_SAVE_VOLTAGE) {
            packet_type = 0x12;
            mode_byte   = 0x02;

        //ミッションモード設定
        } else if (mission_mode == true) {
            packet_type = 0x11;
            mode_byte   = 0x01;
            sensor.sen_acc(&ax, &ay, &az);
            sensor.sen_gyro(&gx, &gy, &gz);
            sensor.sen_mag(&mx, &my, &mz);

        // スタンバイモード設定
        } else {
            packet_type = 0x10;
            mode_byte   = 0x00;
        }


        // モードに合わせてテレメトリデータを取得する
        unsigned short battery_raw = encode_battery(battery_voltage);
        short temperature_raw = encode_temperature(temperature);

        short ax_raw = 0;
        short ay_raw = 0;
        short az_raw = 0;

        short gx_raw = 0;
        short gy_raw = 0;
        short gz_raw = 0;

        short mx_raw = 0;
        short my_raw = 0;
        short mz_raw = 0;

        if (mode_byte == 0x01) {
            ax_raw = encode_acceleration(ax);
            ay_raw = encode_acceleration(ay);
            az_raw = encode_acceleration(az);

            gx_raw = encode_gyro(gx);
            gy_raw = encode_gyro(gy);
            gz_raw = encode_gyro(gz);

            mx_raw = encode_magnetic(mx);
            my_raw = encode_magnetic(my);
            mz_raw = encode_magnetic(mz);
        }


        // パケットを作る
        int idx = 0;
        telemetry_seq++;

        unsigned char payload_length = 0x05;

        if (mode_byte == 0x01) {
            payload_length = 0x17;
        } else {
            payload_length = 0x05;
        }

        // ---- Packet Header ----
        packet[idx++] = 0x7E;        // Header：パケット開始
        packet[idx++] = 0x01;        // Version：パケット形式バージョン
        packet[idx++] = packet_type; // Packet Type：パケット種別

        // ---- Packet Counter ----
        put_u16(packet, &idx, telemetry_seq); // Sequence：通番

        // ---- Packet Information ----
        packet[idx++] = payload_length; // Length：Mode以降のデータ長
        packet[idx++] = mode_byte;      // Mode：衛星状態

        // ---- Basic Telemetry Data ----
        put_u16(packet, &idx, battery_raw);      // Battery Voltage
        put_i16(packet, &idx, temperature_raw);  // Temperature

        // ---- Mission Telemetry Data ----
        // ミッションモードのときだけ，9軸センサデータを入れます．
        if (mode_byte == 0x01) {
            put_i16(packet, &idx, ax_raw);       // Acceleration X
            put_i16(packet, &idx, ay_raw);       // Acceleration Y
            put_i16(packet, &idx, az_raw);       // Acceleration Z

            put_i16(packet, &idx, gx_raw);       // Gyro X
            put_i16(packet, &idx, gy_raw);       // Gyro Y
            put_i16(packet, &idx, gz_raw);       // Gyro Z

            put_i16(packet, &idx, mx_raw);       // Magnetic X
            put_i16(packet, &idx, my_raw);       // Magnetic Y
            put_i16(packet, &idx, mz_raw);       // Magnetic Z
        }

        // ---- Error Check ----
        packet[idx++] = calc_checksum(packet, idx); // Checksum
       
        // パケットを16進で送る
        send_packet_as_hex_text(&com, packet, idx);
        print_packet_hex(&pc, packet, idx);

        wait(1.0);
    }
}