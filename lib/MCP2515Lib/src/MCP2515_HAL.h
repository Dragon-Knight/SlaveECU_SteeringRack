// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
// https://github.com/sandeepmistry/arduino-CAN
// Porting for StarPixel project: Dragon_Knight, https://github.com/Dragon-Knight

#pragma once
#include <inttypes.h>
#include <SPIManager.h>
#include <EasyPinD.h>

class MCP2515Class : public SPIBaseDevice
{
	static constexpr uint32_t NO_CAN_ID = 0xFFFFFFFF;
	
	using func_rx_t = void (*)(uint32_t address, uint8_t *data, uint8_t length);
	
	struct packet_t
	{
		bool flag = false;			// Некий флаг для работы, для tx это инициализация пакета, для rx это готовность пакета
		bool extended = false;		// Флаг exID
		bool rtr = false;			// Флаг RTR
		uint32_t id = NO_CAN_ID;	// Идентификатор пакета CAN
		uint8_t dlc = 0;			// DLC ??
		uint8_t length = 0;			// Длина пакета CAN
		uint8_t data[8] = {0x00};	// Данные пакета CAN
	};
	
	public:
		
		MCP2515Class(EasyPinD::d_pin_t cs_pin, EasyPinD::d_pin_t int_pin, uint32_t spi_prescaler) : 
			SPIBaseDevice(cs_pin, spi_prescaler), 
			_int_pin(int_pin.Port, {int_pin.Pin, GPIO_MODE_EVT_FALLING, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}), 
			_rx{false, false, false, NO_CAN_ID, 0, 0, {0x00}},
			_tx{false, false, false, NO_CAN_ID, 0, 0, {0x00}},
			_onReceive(nullptr),
			_last_tick(0)
		{}
		
		bool begin(uint32_t clock_frequency, uint32_t baud_rate, func_rx_t callback);
		void end();
		
		virtual void Init() override;
		virtual void Tick(uint32_t &time) override;
		
		bool beginPacket(uint16_t id, uint8_t dlc, bool rtr);
		bool beginExtendedPacket(uint32_t id, uint8_t dlc, bool rtr);
		uint8_t write(uint8_t byte){ return write(&byte, 1); }
		uint8_t write(const uint8_t *buffer, uint8_t size);
		bool endPacket();
		
		uint8_t parsePacket();
		
		bool filter(uint16_t id) { return filter(id, 0x7ff); }
		bool filter(uint16_t id, uint16_t mask);
		bool filterExtended(uint32_t id) { return filterExtended(id, 0x1fffffff); }
		bool filterExtended(uint32_t id, uint32_t mask);
		
		bool cmd_observe();
		bool cmd_loopback();
		bool cmd_sleep();
		bool cmd_wakeup();
		
		//void dumpRegisters(Stream& out);
		
	private:
		
		void reset();
		uint8_t readRegister(uint8_t address);
		void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
		void writeRegister(uint8_t address, uint8_t value);
		bool writeReadRegister(uint8_t address, uint8_t value);
		
		EasyPinD _int_pin;
		packet_t _rx;
		packet_t _tx;
		func_rx_t _onReceive;
		uint32_t _last_tick;
		
		
		struct cnf_f
		{
			uint32_t clockFrequency;
			uint32_t baudRate;
			uint8_t cnf[3];
		} _cnf_map[26] = 
		{
			{  8000000, 1000000, {0x00, 0x80, 0x00} },
			{  8000000,  666666, {0xC0, 0xB8, 0x01} },
			{  8000000,  500000, {0x00, 0x90, 0x02} },
			{  8000000,  250000, {0x00, 0xB1, 0x05} },
			{  8000000,  200000, {0x00, 0xB4, 0x06} },
			{  8000000,  125000, {0x01, 0xB1, 0x05} },
			{  8000000,  100000, {0x01, 0xB4, 0x06} },
			{  8000000,   80000, {0x01, 0xBF, 0x07} },
			{  8000000,   50000, {0x03, 0xB4, 0x06} },
			{  8000000,   40000, {0x03, 0xBF, 0x07} },
			{  8000000,   20000, {0x07, 0xBF, 0x07} },
			{  8000000,   10000, {0x0F, 0xBF, 0x07} },
			{  8000000,    5000, {0x1F, 0xBF, 0x07} },

			{ 16000000, 1000000, {0x00, 0xD0, 0x82} },
			{ 16000000,  666666, {0xC0, 0xF8, 0x81} },
			{ 16000000,  500000, {0x00, 0xF0, 0x86} },
			{ 16000000,  250000, {0x41, 0xF1, 0x85} },
			{ 16000000,  200000, {0x01, 0xFA, 0x87} },
			{ 16000000,  125000, {0x03, 0xF0, 0x86} },
			{ 16000000,  100000, {0x03, 0xFA, 0x87} },
			{ 16000000,   80000, {0x03, 0xFF, 0x87} },
			{ 16000000,   50000, {0x07, 0xFA, 0x87} },
			{ 16000000,   40000, {0x07, 0xFF, 0x87} },
			{ 16000000,   20000, {0x0F, 0xFF, 0x87} },
			{ 16000000,   10000, {0x1F, 0xFF, 0x87} },
			{ 16000000,    5000, {0x3F, 0xFF, 0x87} },
		};
};
