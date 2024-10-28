// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
// https://github.com/sandeepmistry/arduino-CAN
// Porting for StarPixel project: Dragon_Knight, https://github.com/Dragon-Knight

#pragma once

#include <inttypes.h>
#include <SPIManager.h>
#include <EasyPinD.h>


#define MCP2515_DEFAULT_CLOCK_FREQUENCY 16e6



class MCP2515Class : public SPIBaseDevice
{
	using func_rx_t = void (*)(uint32_t address, uint8_t *data, uint8_t length);
	
	public:


		MCP2515Class(EasyPinD::d_pin_t cs_pin, EasyPinD::d_pin_t int_pin, uint32_t spi_prescaler) : 
			SPIBaseDevice(cs_pin, spi_prescaler), 
			_int_pin(int_pin.Port, {int_pin.Pin, GPIO_MODE_EVT_FALLING, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}),
			_onReceive(nullptr),
			_packetBegun(false),
			_txId(-1),
			_txExtended(false),
			_txRtr(false),
			_txDlc(0),
			_txLength(0),
			_rxId(-1),
			_rxExtended(false),
			_rxRtr(false),
			_rxDlc(0),
			_rxLength(0)
		{}

   int begin(uint32_t clock_frequency, uint32_t baud_rate, func_rx_t callback);
   void end();
int beginPacket(int id, int dlc, bool rtr);
int beginExtendedPacket(long id, int dlc, bool rtr);

virtual void Init() override;
virtual void Tick(uint32_t &time) override;


   int endPacket();

   int parsePacket();


   int filter(int id) { return filter(id, 0x7ff); }
   int filter(int id, int mask);
   int filterExtended(long id) { return filterExtended(id, 0x1fffffff); }
   int filterExtended(long id, long mask);

   int observe();
   int loopback();
   int sleep();
   int wakeup();


  //void dumpRegisters(Stream& out);

  long packetId();
  bool packetExtended();
  bool packetRtr();
  int packetDlc();
  size_t write(uint8_t byte);
  size_t write(const uint8_t *buffer, size_t size);

  int available();

	private:
		void reset();
		uint8_t readRegister(uint8_t address);
		void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
		void writeRegister(uint8_t address, uint8_t value);
		
		EasyPinD _int_pin;


		uint32_t _last_tick;









func_rx_t _onReceive;

  bool _packetBegun;
  long _txId;
  bool _txExtended;
  bool _txRtr;
  int _txDlc;
  int _txLength;
  uint8_t _txData[8];

  long _rxId;
  bool _rxExtended;
  bool _rxRtr;
  int _rxDlc;
  int _rxLength;
  uint8_t _rxData[8];



// https://ru.wikipedia.org/wiki/Controller_Area_Network
// https://github.com/sandeepmistry/arduino-CAN
		struct packet_t
		{
			bool flag = false;			// Некий флаг для работы, для tx это инициализация пакета, для rx это готовность пакета
			bool extended = false;		// Флаг exID
			bool rtr = false;			// Флаг RTR
			uint32_t id = 0xFFFFFFFF;	// Идентификатор пакета CAN
			uint8_t dlc = 0;			// DLC ??
			uint8_t length = 0;			// Длина пакета CAN
			uint8_t data[8] = {0x00};	// Данные пакета CAN
		};

		packet_t _rx;
		packet_t _tx;




		struct cnf_f
		{
			uint32_t clockFrequency;
			uint32_t baudRate;
			uint8_t cnf[3];
		};

		const cnf_f _cnf_map[26] = 
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
