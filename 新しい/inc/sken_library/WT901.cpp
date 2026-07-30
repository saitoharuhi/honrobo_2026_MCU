/*
 * WT901.cpp
 *
 *  Created on: 2026/06/26
 *      Author: Skenlibrary Master
 */

#include "WT901.h"

WT901::WT901() : yaw_offset_(0.0) {
	data_.roll.angle = 0.0;
	data_.roll.rate = 0.0;
	data_.roll.acceleration = 0.0;
	data_.pitch.angle = 0.0;
	data_.pitch.rate = 0.0;
	data_.pitch.acceleration = 0.0;
	data_.yaw.angle = 0.0;
	data_.yaw.rate = 0.0;
	data_.yaw.acceleration = 0.0;
	for (int i = 0; i < 110; i++) {
		rx_buffer_[i] = 0;
	}
}

void WT901::init(Pin tx_pin, Pin rx_pin, UartNumber uart_num, int baudrate) {
	serial_.init(tx_pin, rx_pin, uart_num, baudrate);
	serial_.startDmaRead(rx_buffer_, 110);
}

bool WT901::update() {
	bool updated = false;
	// 循環バッファの特性上、後ろから前に向かって走査することで、最新のパケットを優先的にデコードする
	for (int i = 110 - 11; i >= 0; i--) {
		if (rx_buffer_[i] == 0x55) {
			uint8_t id = rx_buffer_[i + 1];
			if (id == 0x51 || id == 0x52 || id == 0x53) {
				// チェックサム検証
				uint8_t sum = 0;
				for (int j = 0; j < 10; j++) {
					sum += rx_buffer_[i + j];
				}
				if (sum == rx_buffer_[i + 10]) {
					int16_t raw_data[3];
					raw_data[0] = (int16_t)(rx_buffer_[i + 3] << 8 | rx_buffer_[i + 2]);
					raw_data[1] = (int16_t)(rx_buffer_[i + 5] << 8 | rx_buffer_[i + 4]);
					raw_data[2] = (int16_t)(rx_buffer_[i + 7] << 8 | rx_buffer_[i + 6]);

					if (id == 0x51) { // 加速度
						data_.roll.acceleration = raw_data[0] / 32768.0 * 16.0 * 9.8;
						data_.pitch.acceleration = raw_data[1] / 32768.0 * 16.0 * 9.8;
						data_.yaw.acceleration = raw_data[2] / 32768.0 * 16.0 * 9.8;
						updated = true;
					} else if (id == 0x52) { // 角速度
						data_.roll.rate = raw_data[0] / 32768.0 * 2000.0;
						data_.pitch.rate = raw_data[1] / 32768.0 * 2000.0;
						data_.yaw.rate = raw_data[2] / 32768.0 * 2000.0;
						updated = true;
					} else if (id == 0x53) { // 角度
						data_.roll.angle = raw_data[0] / 32768.0 * 180.0;
						data_.pitch.angle = raw_data[1] / 32768.0 * 180.0;
						
						double yaw_val = raw_data[2] / 32768.0 * 180.0;
						double relative_yaw = yaw_val - yaw_offset_;
						while (relative_yaw > 180.0) relative_yaw -= 360.0;
						while (relative_yaw < -180.0) relative_yaw += 360.0;
						data_.yaw.angle = relative_yaw;
						updated = true;
					}
				}
			}
		}
	}
	return updated;
}

wt901_data_t WT901::getData() const {
	return data_;
}

void WT901::resetAngle() {
	// 現在の角度をオフセットとして記録し、相対角度を0にする
	for (int i = 110 - 11; i >= 0; i--) {
		if (rx_buffer_[i] == 0x55 && rx_buffer_[i + 1] == 0x53) {
			uint8_t sum = 0;
			for (int j = 0; j < 10; j++) sum += rx_buffer_[i + j];
			if (sum == rx_buffer_[i + 10]) {
				int16_t raw_yaw = (int16_t)(rx_buffer_[i + 7] << 8 | rx_buffer_[i + 6]);
				yaw_offset_ = raw_yaw / 32768.0 * 180.0;
				data_.yaw.angle = 0.0;
				break;
			}
		}
	}
}
