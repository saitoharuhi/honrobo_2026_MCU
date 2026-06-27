#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "sken_library/OTOS.h"
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// インスタンス定義
OTOS otos;
WT901 wt901;
Uart debug_uart;

// 自己位置データ (グローバル座標)
volatile float self_x = 0.0f;
volatile float self_y = 0.0f;
volatile float self_yaw = 0.0f;

// OTOSデータバッファと過去のデータ
float maus_data[36];
float past_maus_data_0 = 0.0f;
float past_maus_data_1 = 0.0f;
bool is_first = true;

// 自己位置推定のタイマー割り込み関数 (10ms周期)
void pos_estimation_interrupt() {
	// WT901の角度を更新
	wt901.update();
	wt901_data_t gyro_data = wt901.getData();
	self_yaw = std::round(gyro_data.yaw.angle * 1000.0f) / 1000.0f;

	// OTOSからデータを取得 (9要素読み込む)
	otos.get_odom(maus_data, 9);

	if (is_first) {
		past_maus_data_0 = maus_data[0];
		past_maus_data_1 = maus_data[1];
		is_first = false;
		return;
	}

	// 1. OTOSの内部累積値の差分（ミリメートル単位）を計算
	float d_odom0 = maus_data[0] - past_maus_data_0; // センサX軸(odom_0)の差分
	float d_odom1 = maus_data[1] - past_maus_data_1; // センサY軸(odom_1)の差分
	float rad_otos = maus_data[2] * (M_PI / 180.0f);

	// 2. OTOSの内部角度で逆回転させ、センサローカル移動量 dx_sensor, dy_sensor を計算
	float dx_sensor =  d_odom0 * cos(rad_otos) + d_odom1 * sin(rad_otos);
	float dy_sensor = -d_odom0 * sin(rad_otos) + d_odom1 * cos(rad_otos);

	// 3. ロボットのローカル移動量にマッピング
	// 既存コード x = maus_data[1]*25.4 (odom_1), y = maus_data[0]*25.4 (odom_0) より、
	// ロボットの進行方向(X)はセンサのY軸(odom_1)、横方向(Y)はセンサのX軸(odom_0)に対応。
	float d_forward = dy_sensor;
	float d_lateral = dx_sensor;

	// 4. WT901の角度でグローバル座標の増分に変換し、自己位置を更新
	float rad_gyro = self_yaw * (M_PI / 180.0f);
	float next_x = self_x + (d_forward * cos(rad_gyro) - d_lateral * sin(rad_gyro));
	float next_y = self_y + (d_forward * sin(rad_gyro) + d_lateral * cos(rad_gyro));
	self_x = std::round(next_x * 1000.0f) / 1000.0f;
	self_y = std::round(next_y * 1000.0f) / 1000.0f;

	// 過去データの保存
	past_maus_data_0 = maus_data[0];
	past_maus_data_1 = maus_data[1];
}


int main(void) {
	// システムの初期化
	sken_system.init();

	// OTOSの初期化 (SCL=B9, SDA=B8, I2C_1)
	otos.init(B9, B8, I2C_1);

	// WT901の初期化 (TX=B10, RX=C5, SERIAL3, 115200)
	wt901.init(B10, C5, SERIAL3, 115200);

	// デバッグ用シリアルの初期化 (TX=A2, RX=A3, SERIAL2, 115200)
	debug_uart.init(A2, A3, SERIAL2, 115200);

	// 10ms周期で自己位置推定のタイマー割り込みを登録 (ID: 0)
	sken_system.addTimerInterruptFunc(pos_estimation_interrupt, 0, 10);

	// 起動時の初期化ウェイト
	HAL_Delay(500);
	
	// ジャイロの初期角度を0にリセット
	wt901.resetAngle();

	char uart_buf[100];

	while (1) {
		// 計算された自己位置とジャイロ角度をST-Linkの仮想COMポートへ出力
		int len = sprintf(uart_buf, "X: %.3f mm, Y: %.3f mm, Yaw: %.3f deg\r\n", self_x, self_y, self_yaw);
		debug_uart.write((uint8_t*)uart_buf, len);

		HAL_Delay(100); // 100ms周期で出力
	}
}
