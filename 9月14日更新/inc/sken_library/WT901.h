/*
 * WT901.h
 *
 *  Created on: 2026/06/26
 *      Author: Skenlibrary Master
 */

#ifndef SKEN_LIBRARY_WT901_H_
#define SKEN_LIBRARY_WT901_H_

#include "system.h"
#include "uart.h"

struct wt901_data_t {
	struct {
		double angle;
		double rate;
		double acceleration;
	} roll;
	struct {
		double angle;
		double rate;
		double acceleration;
	} pitch;
	struct {
		double angle;
		double rate;
		double acceleration;
	} yaw;
};

class WT901 {
public:
	WT901();
	void init(Pin tx_pin, Pin rx_pin, UartNumber uart_num, int baudrate = 115200);
	bool update();
	wt901_data_t getData() const;
	void resetAngle();

private:
	Uart serial_;
	uint8_t rx_buffer_[110];
	wt901_data_t data_;
	double yaw_offset_;
};

#endif /* SKEN_LIBRARY_WT901_H_ */
