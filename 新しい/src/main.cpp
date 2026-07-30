#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "sken_library/OTOS.h"
#include "sken_library/encoder.h"
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// エンコーダ物理パラメータ
#define ENCODER_PPR 8192
#define WHEEL_DIAMETER 38.1f   // タイヤ径 38.1mm
#define ROBOT_RADIUS 530.0f    // 配置半径 530mm

// インスタンス定義
OTOS otos;
WT901 wt901;
Uart debug_uart;

Encoder enc1, enc2, enc3, enc4;
Encoder_data e_data1, e_data2, e_data3, e_data4;

Gpio user_button;
Gpio user_led;

// 自己位置データ (グローバル座標)
volatile float self_x = 0.0f;
volatile float self_y = 0.0f;
volatile float self_yaw = 0.0f;

// 自己位置推定モード (0: 4輪+センサ, 1: 3輪+センサ, 2: センサのみ, 3: 4輪のみ, 4: 3輪のみ)
volatile int position_mode = 0;
volatile bool mode_changed_flag = false;

// OTOSデータバッファと過去のデータ
float maus_data[36];
float past_maus_data_0 = 0.0f;
float past_maus_data_1 = 0.0f;
bool is_first = true;

// エンコーダの過去データ
float past_enc_dist1 = 0.0f;
float past_enc_dist2 = 0.0f;
float past_enc_dist3 = 0.0f;
float past_enc_dist4 = 0.0f;
float past_gyro_yaw = 0.0f;
bool is_enc_first = true;

// 自己位置およびジャイロのリセット用関数
void reset_self_position(float x, float y, float yaw) {
	self_x = x;
	self_y = y;
	is_first = true;
	is_enc_first = true;
	if (yaw >= -180.0f && yaw <= 180.0f) {
		wt901.resetAngle();
		self_yaw = yaw;
		past_gyro_yaw = yaw;
	}
}

// LED点滅用関数
void blink_led(int count) {
	for (int i = 0; i < count; i++) {
		user_led.write(HIGH);
		HAL_Delay(100);
		user_led.write(LOW);
		HAL_Delay(100);
	}
}

// 自己位置推定のタイマー割り込み関数 (10ms周期)
void pos_estimation_interrupt() {
	// WT901の角度を更新
	wt901.update();
	wt901_data_t gyro_data = wt901.getData();
	float current_gyro_yaw = gyro_data.yaw.angle;

	// エンコーダの値を更新
	enc1.interrupt(&e_data1);
	enc2.interrupt(&e_data2);
	enc3.interrupt(&e_data3);
	enc4.interrupt(&e_data4);

	// 初回起動時、またはモード切り替え時は前回のバッファを同期して移動差分を0から開始する
	if (is_first || is_enc_first || mode_changed_flag) {
		past_maus_data_0 = maus_data[0];
		past_maus_data_1 = maus_data[1];
		past_enc_dist1 = e_data1.distance;
		past_enc_dist2 = e_data2.distance;
		past_enc_dist3 = e_data3.distance;
		past_enc_dist4 = e_data4.distance;
		past_gyro_yaw = current_gyro_yaw;
		if (is_first || is_enc_first) {
			self_yaw = current_gyro_yaw;
		}
		is_first = false;
		is_enc_first = false;
		mode_changed_flag = false;
		return;
	}

	// 1. ジャイロの角度変化量を計算 (回り込み処理)
	float d_theta_gyro = current_gyro_yaw - past_gyro_yaw;
	if (d_theta_gyro > 180.0f) {
		d_theta_gyro -= 360.0f;
	} else if (d_theta_gyro < -180.0f) {
		d_theta_gyro += 360.0f;
	}

	// 2. エンコーダの移動量（前回からの増分）を計算
	float ds1 = e_data1.distance - past_enc_dist1;
	float ds2 = e_data2.distance - past_enc_dist2;
	float ds3 = e_data3.distance - past_enc_dist3;
	float ds4 = e_data4.distance - past_enc_dist4;

	// 3. OTOSからデータを取得してグローバル移動量を計算
	otos.get_odom(maus_data, 9);
	float d_odom0 = maus_data[0] - past_maus_data_0;
	float d_odom1 = maus_data[1] - past_maus_data_1;
	float rad_otos = maus_data[2] * (M_PI / 180.0f);

	float dx_sensor =  d_odom0 * cos(rad_otos) + d_odom1 * sin(rad_otos);
	float dy_sensor = -d_odom0 * sin(rad_otos) + d_odom1 * cos(rad_otos);
	float d_forward_otos = dy_sensor;
	float d_lateral_otos = dx_sensor;

	float rad_gyro = self_yaw * (M_PI / 180.0f);
	float d_otos_x = d_forward_otos * cos(rad_gyro) - d_lateral_otos * sin(rad_gyro);
	float d_otos_y = d_forward_otos * sin(rad_gyro) + d_lateral_otos * cos(rad_gyro);

	// モードに応じた計算処理
	float d_fused_x = 0.0f;
	float d_fused_y = 0.0f;
	float d_theta_fused = 0.0f;

	if (position_mode == 0) {
		// 【モード0】: 4輪+センサ (四角形の辺中点配置)
		float dx_enc = (ds4 - ds2) / 2.0f;
		float dy_enc = (ds1 - ds3) / 2.0f;
		float d_theta_enc_rad = (ds1 + ds2 + ds3 + ds4) / (4.0f * ROBOT_RADIUS);
		float d_theta_enc = d_theta_enc_rad * (180.0f / M_PI);

		d_theta_fused = 0.7f * d_theta_gyro + 0.3f * d_theta_enc;

		// 計測輪の移動量をグローバル座標に変換
		float d_enc_x = dx_enc * cos(rad_gyro) - dy_enc * sin(rad_gyro);
		float d_enc_y = dx_enc * sin(rad_gyro) + dy_enc * cos(rad_gyro);

		d_fused_x = 0.7f * d_otos_x + 0.3f * d_enc_x;
		d_fused_y = 0.7f * d_otos_y + 0.3f * d_enc_y;
	}
	else if (position_mode == 1) {
		// 【モード1】: 3輪+センサ (正三角形の辺中点配置: enc1, enc2, enc3 を使用)
		float dx_enc = (ds1 + ds3) / 2.0f - ds2;
		float dy_enc = (ds1 - ds2) / 1.7320508f; // (ds1 - ds2) / sqrt(3)
		float d_theta_enc_rad = (ds1 + ds2 + ds3) / (3.0f * ROBOT_RADIUS);
		float d_theta_enc = d_theta_enc_rad * (180.0f / M_PI);

		d_theta_fused = 0.7f * d_theta_gyro + 0.3f * d_theta_enc;

		// 計測輪の移動量をグローバル座標に変換
		float d_enc_x = dx_enc * cos(rad_gyro) - dy_enc * sin(rad_gyro);
		float d_enc_y = dx_enc * sin(rad_gyro) + dy_enc * cos(rad_gyro);

		d_fused_x = 0.7f * d_otos_x + 0.3f * d_enc_x;
		d_fused_y = 0.7f * d_otos_y + 0.3f * d_enc_y;
	}
	else if (position_mode == 2) {
		// 【モード2】: センサのみ (計測輪なし)
		d_theta_fused = d_theta_gyro;
		d_fused_x = d_otos_x;
		d_fused_y = d_otos_y;
	}
	else if (position_mode == 3) {
		// 【モード3】: 4輪のみ (純オドメトリ)
		float dx_enc = (ds4 - ds2) / 2.0f;
		float dy_enc = (ds1 - ds3) / 2.0f;
		float d_theta_enc_rad = (ds1 + ds2 + ds3 + ds4) / (4.0f * ROBOT_RADIUS);
		float d_theta_enc = d_theta_enc_rad * (180.0f / M_PI);

		d_theta_fused = d_theta_enc;

		// 角度を更新した後の角度でグローバル座標に射影
		float next_yaw_temp = self_yaw + d_theta_fused;
		if (next_yaw_temp > 180.0f) next_yaw_temp -= 360.0f;
		else if (next_yaw_temp < -180.0f) next_yaw_temp += 360.0f;
		float rad_enc_yaw = next_yaw_temp * (M_PI / 180.0f);

		d_fused_x = dx_enc * cos(rad_enc_yaw) - dy_enc * sin(rad_enc_yaw);
		d_fused_y = dx_enc * sin(rad_enc_yaw) + dy_enc * cos(rad_enc_yaw);
	}
	else {
		// 【モード4】: 3輪のみ (純オドメトリ - 正三角形配置: enc1, enc2, enc3 を使用)
		float dx_enc = (ds1 + ds3) / 2.0f - ds2;
		float dy_enc = (ds1 - ds2) / 1.7320508f;
		float d_theta_enc_rad = (ds1 + ds2 + ds3) / (3.0f * ROBOT_RADIUS);
		float d_theta_enc = d_theta_enc_rad * (180.0f / M_PI);

		d_theta_fused = d_theta_enc;

		// 角度を更新した後の角度でグローバル座標に射影
		float next_yaw_temp = self_yaw + d_theta_fused;
		if (next_yaw_temp > 180.0f) next_yaw_temp -= 360.0f;
		else if (next_yaw_temp < -180.0f) next_yaw_temp += 360.0f;
		float rad_enc_yaw = next_yaw_temp * (M_PI / 180.0f);

		d_fused_x = dx_enc * cos(rad_enc_yaw) - dy_enc * sin(rad_enc_yaw);
		d_fused_y = dx_enc * sin(rad_enc_yaw) + dy_enc * cos(rad_enc_yaw);
	}

	// 角度・位置の積算
	float next_yaw = self_yaw + d_theta_fused;
	if (next_yaw > 180.0f) {
		next_yaw -= 360.0f;
	} else if (next_yaw < -180.0f) {
		next_yaw += 360.0f;
	}
	self_yaw = std::round(next_yaw * 1000.0f) / 1000.0f;

	float next_x = self_x + d_fused_x;
	float next_y = self_y + d_fused_y;
	self_x = std::round(next_x * 1000.0f) / 1000.0f;
	self_y = std::round(next_y * 1000.0f) / 1000.0f;

	// 過去データの保存
	past_maus_data_0 = maus_data[0];
	past_maus_data_1 = maus_data[1];
	past_enc_dist1 = e_data1.distance;
	past_enc_dist2 = e_data2.distance;
	past_enc_dist3 = e_data3.distance;
	past_enc_dist4 = e_data4.distance;
	past_gyro_yaw = current_gyro_yaw;
}

int main(void) {
	// システムの初期化
	sken_system.init();

	// スイッチとLEDの初期化 (青いユーザーボタン: C13、緑のLED: A5)
	user_button.init(C13, INPUT_PULLUP);
	user_led.init(A5, OUTPUT);

	// エンコーダの初期化 (10ms周期でカウントを反映するため、periodは10とする)
	enc1.init(A0, A1, TIMER5, WHEEL_DIAMETER, ENCODER_PPR, 10);
	enc2.init(A15, B3, TIMER2, WHEEL_DIAMETER, ENCODER_PPR, 10);
	enc3.init(B6, B7, TIMER4, WHEEL_DIAMETER, ENCODER_PPR, 10);
	enc4.init(C6, C7, TIMER3, WHEEL_DIAMETER, ENCODER_PPR, 10);

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

	char uart_buf[120];
	uint32_t last_uart_tick = HAL_GetTick();
	bool last_button_state = HIGH;

	while (1) {
		// ボタン状態の取得とエッジ検出 (チャタリング対策)
		bool current_button_state = user_button.read();
		if (last_button_state == HIGH && current_button_state == LOW) {
			HAL_Delay(20); // チャタリングウェイト
			if (user_button.read() == LOW) {
				// モードを循環インクリメント (0 ➔ 1 ➔ 2 ➔ 3 ➔ 4 ➔ 0)
				position_mode = (position_mode + 1) % 5;
				mode_changed_flag = true;

				// シリアル出力で切り替えを通知
				int msg_len = sprintf(uart_buf, "\r\n[System] Mode Switched to: %d\r\n", position_mode);
				debug_uart.write((uint8_t*)uart_buf, msg_len);

				// LEDを点滅通知 (モード0: 1回, モード1: 2回, モード2: 3回, モード3: 4回, モード4: 5回)
				blink_led(position_mode + 1);
			}
		}
		last_button_state = current_button_state;

		// 10ms周期で自己位置とジャイロ角度をシリアル出力 (非ブロッキング処理)
		uint32_t current_tick = HAL_GetTick();
		if (current_tick - last_uart_tick >= 10) {
			int len = sprintf(uart_buf, "X: %.3f mm, Y: %.3f mm, Yaw: %.3f deg (Mode:%d)\r\n", self_x, self_y, self_yaw, position_mode);
			debug_uart.write((uint8_t*)uart_buf, len);
			last_uart_tick = current_tick;
		}
	}
}





