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
#define WHEEL_DIAMETER 78.2f   // タイヤ径 78.2mm
#define ROBOT_RADIUS 370.0f    // 配置半径 370mm

// エンコーダの接続ピン定義 (元コード of 標準順序)
#define ENC1_PIN_A A0
#define ENC1_PIN_B A1

#define ENC2_PIN_A A15
#define ENC2_PIN_B B3

#define ENC3_PIN_A B6
#define ENC3_PIN_B B7

#define ENC4_PIN_A C6
#define ENC4_PIN_B C7

// 各エンコーダの回転方向定義 (1.0f: そのまま, -1.0f: 反転)
// ※実機での左回り1周旋回テストに基づき、すべてがプラス加算されるように符号を極性整合させました
#define ENC1_DIR  1.0f
#define ENC2_DIR -1.0f
#define ENC3_DIR  1.0f
#define ENC4_DIR  1.0f

// インスタンス定義
OTOS otos;
WT901 wt901;
Uart debug_uart;

Encoder enc1, enc2, enc3, enc4;
Encoder_data e_data1, e_data2, e_data3, e_data4;

Gpio user_button;
Gpio user_led;

// C++の名前マングリングを防ぎ、CubeMonitorでそのままの名前でシンボルを見つけられるようにする
extern "C" {
	// 自己位置データ (グローバル座標)
	volatile float self_x = 0.0f;
	volatile float self_y = 0.0f;
	volatile float self_yaw = 0.0f;

	// STM32CubeMonitor監視用グローバル変数
	volatile float monitor_e1_dist = 0.0f;
	volatile float monitor_e2_dist = 0.0f;
	volatile float monitor_e3_dist = 0.0f;
	volatile float monitor_e4_dist = 0.0f;

	// ジャイロ生角度監視用
	volatile float monitor_gyro_yaw = 0.0f;
}

// 自己位置推定モード (0: 4輪+センサ, 1: 3輪+センサ, 2: センサのみ, 3: 4輪のみ, 4: 3輪のみ)
volatile int position_mode = 0;
volatile bool mode_changed_flag = false;

// 外部スレッド/割り込みからの非同期リセット要求用フラグとバッファ
volatile bool request_reset_position = false;
volatile float req_reset_x = 0.0f;
volatile float req_reset_y = 0.0f;
volatile float req_reset_yaw = 0.0f;

// ジャイロリセット開始時刻と強制固定フラグ
volatile uint32_t gyro_reset_tick = 0;
volatile float reset_target_yaw = 0.0f;
volatile bool is_resetting_yaw = false;

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
// ※非同期スレッドからのI2C/UART/メモリ競合によるハングアップを防ぐため、
//   ここではリクエストを書き留めてタイマー割り込み側で安全にリセット処理を実行させます。
void reset_self_position(float x, float y, float yaw) {
	req_reset_x = x;
	req_reset_y = y;
	req_reset_yaw = yaw;
	request_reset_position = true;
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
	// 外部（メインループやCAN受信など）からのリセット要求を安全に割り込みコンテキスト内で実行
	if (request_reset_position) {
		self_x = req_reset_x;
		self_y = req_reset_y;
		is_enc_first = true; // エンコーダ過去値などのバッファ同期要求
		if (req_reset_yaw >= -180.0f && req_reset_yaw <= 180.0f) {
			wt901.resetAngle(); // 安全なスレッドで高速にリセット
			gyro_reset_tick = HAL_GetTick(); // リセットロック開始時刻を記録
			reset_target_yaw = req_reset_yaw;
			is_resetting_yaw = true; // 強制ロックフラグを有効化
			self_yaw = req_reset_yaw;
			past_gyro_yaw = req_reset_yaw;
		}
		request_reset_position = false; // 要求フラグをクリア
	}

	// WT901の角度を更新
	wt901.update();
	wt901_data_t gyro_data = wt901.getData();
	// ジャイロの回転極性をオドメトリ（反時計回り＝プラス）に一致させるため、符号を反転します
	float current_gyro_yaw = -gyro_data.yaw.angle;

	// 【位置変更】エンコーダの値を更新 (同期ブロックより上で実行し、リセット時の時間差による差分ズレを防ぎます)
	enc1.interrupt(&e_data1);
	enc2.interrupt(&e_data2);
	enc3.interrupt(&e_data3);
	enc4.interrupt(&e_data4);

	// 【位置変更】OTOSからデータを取得 (同期ブロックより上で実行し、リセット時の時間差による差分ズレを防ぎます)
	otos.get_odom(maus_data, 9);

	// 初回起動時、またはモード切り替え時は前回のバッファを同期して移動差分を0から開始する
	if (is_first || is_enc_first || mode_changed_flag) {
		past_maus_data_0 = maus_data[0];
		past_maus_data_1 = maus_data[1];
		past_enc_dist1 = e_data1.distance;
		past_enc_dist2 = e_data2.distance;
		past_enc_dist3 = e_data3.distance;
		past_enc_dist4 = e_data4.distance;
		past_gyro_yaw = current_gyro_yaw;
		if (is_first) { // 電源投入の初回時のみジャイロの初期値をself_yawに反映する
			self_yaw = current_gyro_yaw;
		}
		is_first = false;
		is_enc_first = false;
		mode_changed_flag = false;
		return;
	}

	// 1. ジャイロの角度変化量を計算 (リセット期間中は0とする)
	float d_theta_gyro = 0.0f;
	if (!is_resetting_yaw) {
		d_theta_gyro = current_gyro_yaw - past_gyro_yaw;
		if (d_theta_gyro > 180.0f) {
			d_theta_gyro -= 360.0f;
		} else if (d_theta_gyro < -180.0f) {
			d_theta_gyro += 360.0f;
		}
	} else {
		// リセット途中の移行期間は、過去値を常に同期し続けて変化量を0にする
		past_gyro_yaw = current_gyro_yaw;
	}

	// 2. エンコーダの移動量（前回からの増分）を計算（方向係数を適用）
	float ds1 = (e_data1.distance - past_enc_dist1) * ENC1_DIR;
	float ds2 = (e_data2.distance - past_enc_dist2) * ENC2_DIR;
	float ds3 = (e_data3.distance - past_enc_dist3) * ENC3_DIR;
	float ds4 = (e_data4.distance - past_enc_dist4) * ENC4_DIR;

	// 3. OTOSからデータを取得してグローバル移動量を計算
	float d_odom0 = maus_data[0] - past_maus_data_0;
	float d_odom1 = maus_data[1] - past_maus_data_1;
	float rad_otos = maus_data[2] * (M_PI / 180.0f);

	float dx_sensor =  d_odom0 * cos(rad_otos) + d_odom1 * sin(rad_otos);
	float dy_sensor = -d_odom0 * sin(rad_otos) + d_odom1 * cos(rad_otos);
	float d_forward_otos = dy_sensor;
	float d_lateral_otos = -dx_sensor;

	float rad_gyro = self_yaw * (M_PI / 180.0f);
	float d_otos_x = d_forward_otos * cos(rad_gyro) - d_lateral_otos * sin(rad_gyro);
	float d_otos_y = d_forward_otos * sin(rad_gyro) + d_lateral_otos * cos(rad_gyro);

	// モードに応じた計算処理
	float d_fused_x = 0.0f;
	float d_fused_y = 0.0f;
	float d_theta_fused = 0.0f;

	if (position_mode == 0) {
		// 【モード0】: 4輪+センサ (四角形の辺中点配置)
		float dx_enc = (ds2 - ds4) / 2.0f;
		float dy_enc = (ds3 - ds1) / 2.0f;
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
		float dx_enc = (2.0f * ds3 - ds1 - ds2) / 3.0f;
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
		float dx_enc = (ds2 - ds4) / 2.0f;
		float dy_enc = (ds3 - ds1) / 2.0f;
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
		float dx_enc = (2.0f * ds3 - ds1 - ds2) / 3.0f;
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

	// リセット期間中（1000ms）は強制的に目標角度に固定し続ける (ジャイロリセットの遅れを完全ガード)
	if (is_resetting_yaw) {
		if (HAL_GetTick() - gyro_reset_tick <= 1000) {
			next_yaw = reset_target_yaw;
		} else {
			is_resetting_yaw = false; // 1秒経過でロック解除
		}
	}

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

	// モニター用変数への反映 (方向係数を適用)
	monitor_e1_dist = e_data1.distance * ENC1_DIR;
	monitor_e2_dist = e_data2.distance * ENC2_DIR;
	monitor_e3_dist = e_data3.distance * ENC3_DIR;
	monitor_e4_dist = e_data4.distance * ENC4_DIR;

	// ジャイロの生角度モニター
	monitor_gyro_yaw = current_gyro_yaw;
}

int main(void) {
	// システムの初期化
	sken_system.init();

	// 起動直後の電源および通信バスの安定化ウェイト (1秒)
	// ※周辺センサーの起動を待ち、I2C/シリアルラインのノイズによるフリーズを完全に防止します
	HAL_Delay(1000);

	// スイッチとLEDの初期化 (青いユーザーボタン: C13、緑のLED: A5)
	user_button.init(C13, INPUT_PULLUP);
	user_led.init(A5, OUTPUT);

	// エンコーダの初期化 (10ms周期でカウントを反映するため、periodは10とする)
	enc1.init(ENC1_PIN_A, ENC1_PIN_B, TIMER5, WHEEL_DIAMETER, ENCODER_PPR, 10);
	enc2.init(ENC2_PIN_A, ENC2_PIN_B, TIMER2, WHEEL_DIAMETER, ENCODER_PPR, 10);
	enc3.init(ENC3_PIN_A, ENC3_PIN_B, TIMER4, WHEEL_DIAMETER, ENCODER_PPR, 10);
	enc4.init(ENC4_PIN_A, ENC4_PIN_B, TIMER3, WHEEL_DIAMETER, ENCODER_PPR, 10);

	// OTOS의初期化 (SCL=B9, SDA=B8, I2C_1)
	otos.init(B9, B8, I2C_1);

	// WT901の初期化 (TX=B10, RX=C5, SERIAL3, 115200)
	wt901.init(B10, C5, SERIAL3, 115200);

	// 2. ジャイロのコマンド到達を保証するためのディレイ (1秒)
	HAL_Delay(1000);
	
	// 3. ジャイロの初期角度を0にリセット
	wt901.resetAngle();

	// 4. ジャイロのリセット処理が完了するのを待つ (1秒)
	HAL_Delay(1000);

	// デバッグ用シリアルの初期化 (TX=A2, RX=A3, SERIAL2, 115200)
	debug_uart.init(A2, A3, SERIAL2, 115200);

	// 2ms周期で自己位置推定のタイマー割り込みを登録 (ID: 0)
	sken_system.addTimerInterruptFunc(pos_estimation_interrupt, 0, 2);

	char uart_buf[120];
	uint32_t last_uart_tick = HAL_GetTick();
	bool last_button_state = HIGH;

	// コンパイラの最適化によるグローバル変数削除を防ぐためのダミー参照変数
	volatile float monitor_keep_alive = 0.0f;

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
			int len = sprintf(uart_buf, "X:%.1f Y:%.1f Yaw:%.1f Gyro:%.1f (Mode:%d) | E1:%.1f E2:%.1f E3:%.1f E4:%.1f\r\n", 
				self_x, self_y, self_yaw, monitor_gyro_yaw, position_mode, 
				e_data1.distance * ENC1_DIR, e_data2.distance * ENC2_DIR, e_data3.distance * ENC3_DIR, e_data4.distance * ENC4_DIR);
			debug_uart.write((uint8_t*)uart_buf, len);
			last_uart_tick = current_tick;
		}

		// CubeMonitor用変数が最適化で消去されるのを防止するダミーの読み出し処理
		monitor_keep_alive = monitor_e1_dist + monitor_e2_dist + monitor_e3_dist + monitor_e4_dist + monitor_gyro_yaw;
	}
}
