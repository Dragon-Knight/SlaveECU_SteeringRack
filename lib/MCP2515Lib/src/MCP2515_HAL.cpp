// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
// https://github.com/sandeepmistry/arduino-CAN
// Porting for StarPixel project: Dragon_Knight, https://github.com/Dragon-Knight

#include "MCP2515_HAL.h"

#define REG_BFPCTRL                0x0C
#define REG_TXRTSCTRL              0x0D

#define REG_CANCTRL                0x0F

#define REG_CNF3                   0x28
#define REG_CNF2                   0x29
#define REG_CNF1                   0x2A

#define REG_CANINTE                0x2B
#define REG_CANINTF                0x2C

#define FLAG_RXnIE(n)              (0x01 << n)
#define FLAG_RXnIF(n)              (0x01 << n)
#define FLAG_TXnIF(n)              (0x04 << n)

#define REG_RXFnSIDH(n)            (0x00 + (n * 4))
#define REG_RXFnSIDL(n)            (0x01 + (n * 4))
#define REG_RXFnEID8(n)            (0x02 + (n * 4))
#define REG_RXFnEID0(n)            (0x03 + (n * 4))

#define REG_RXMnSIDH(n)            (0x20 + (n * 0x04))
#define REG_RXMnSIDL(n)            (0x21 + (n * 0x04))
#define REG_RXMnEID8(n)            (0x22 + (n * 0x04))
#define REG_RXMnEID0(n)            (0x23 + (n * 0x04))

#define REG_TXBnCTRL(n)            (0x30 + (n * 0x10))
#define REG_TXBnSIDH(n)            (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n)            (0x32 + (n * 0x10))
#define REG_TXBnEID8(n)            (0x33 + (n * 0x10))
#define REG_TXBnEID0(n)            (0x34 + (n * 0x10))
#define REG_TXBnDLC(n)             (0x35 + (n * 0x10))
#define REG_TXBnD0(n)              (0x36 + (n * 0x10))

#define REG_RXBnCTRL(n)            (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n)            (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n)            (0x62 + (n * 0x10))
#define REG_RXBnEID8(n)            (0x63 + (n * 0x10))
#define REG_RXBnEID0(n)            (0x64 + (n * 0x10))
#define REG_RXBnDLC(n)             (0x65 + (n * 0x10))
#define REG_RXBnD0(n)              (0x66 + (n * 0x10))

#define FLAG_IDE                   0x08
#define FLAG_SRR                   0x10
#define FLAG_RTR                   0x40
#define FLAG_EXIDE                 0x08

#define FLAG_RXM0                  0x20
#define FLAG_RXM1                  0x40





bool MCP2515Class::begin(uint32_t clock_frequency, uint32_t baud_rate, func_rx_t callback)
{
	// Нет необходимости чистить всё полностью. Этого достаточно
	_rx.flag = false;
	_rx.id = NO_CAN_ID;
	_tx.flag = false;
	_tx.id = NO_CAN_ID;
	
	_onReceive = callback;
	
	reset();

	if( writeReadRegister(REG_CANCTRL, 0x80) == false ) return false;
	
	const uint8_t *cnf = nullptr;
	for(uint8_t i = 0; i < (sizeof(_cnf_map) / sizeof(_cnf_map[0])); ++i)
	{
		if(_cnf_map[i].clockFrequency == clock_frequency && _cnf_map[i].baudRate == baud_rate)
		{
			cnf = _cnf_map[i].cnf;
			break;
		}
	}
	
	if(cnf == nullptr) return false;
	
	writeRegister(REG_CNF1, cnf[0]);
	writeRegister(REG_CNF2, cnf[1]);
	writeRegister(REG_CNF3, cnf[2]);
	
	writeRegister(REG_CANINTE, FLAG_RXnIE(1) | FLAG_RXnIE(0));
	writeRegister(REG_BFPCTRL, 0x00);
	writeRegister(REG_TXRTSCTRL, 0x00);
	writeRegister(REG_RXBnCTRL(0), FLAG_RXM1 | FLAG_RXM0);
	writeRegister(REG_RXBnCTRL(1), FLAG_RXM1 | FLAG_RXM0);
	
	if( writeReadRegister(REG_CANCTRL, 0x00) == false ) return false;
	
	return true;
}

void MCP2515Class::end()
{
	return;
}





void MCP2515Class::Init()
{
	_int_pin.Init();
	
	return;
}

void MCP2515Class::Tick(uint32_t &time)
{
	if(time - _last_tick == 0) return;
	_last_tick = time;
	
	if(_int_pin.Read() == GPIO_PIN_RESET)
	{
		if(readRegister(REG_CANINTF) == 0) return;
		
		while(parsePacket() || _rx.id != NO_CAN_ID)
		{
			_onReceive(_rx.id, _rx.data, _rx.length);
		}
	}
	
	return;
}





bool MCP2515Class::beginPacket(uint16_t id, uint8_t dlc, bool rtr)
{
	if(id > 0x7FF) return false;
	if(dlc > 8) return false;
	
	_tx.flag = true;
	_tx.id = id;
	_tx.extended = false;
	_tx.rtr = rtr;
	_tx.dlc = dlc;
	_tx.length = 0;
	memset(_tx.data, 0x00, sizeof(_tx.data));
	
	return true;
}

bool MCP2515Class::beginExtendedPacket(uint32_t id, uint8_t dlc, bool rtr)
{
	if(id > 0x1FFFFFFF) return false;
	if(dlc > 8) return false;
	
	_tx.flag = true;
	_tx.id = id;
	_tx.extended = true;
	_tx.rtr = rtr;
	_tx.dlc = dlc;
	_tx.length = 0;
	memset(_tx.data, 0x00, sizeof(_tx.data));
	
	return true;
}

uint8_t MCP2515Class::write(const uint8_t *buffer, uint8_t size)
{
	if(_tx.flag == false) return 0;
	
	if(size > (sizeof(_tx.data) - _tx.length))
	{
		size = sizeof(_tx.data) - _tx.length;
	}
	memcpy(&_tx.data[_tx.length], buffer, size);
	_tx.length += size;
	
	return size;
}

bool MCP2515Class::endPacket()
{
	if(_tx.flag == false) return false;
	_tx.flag = false;
	
	if(_tx.dlc >= 0)
		_tx.length = _tx.dlc;
	
	uint8_t n = 0;
	if(_tx.extended == true)
	{
		writeRegister(REG_TXBnSIDH(n), _tx.id >> 21);
		writeRegister(REG_TXBnSIDL(n), (((_tx.id >> 18) & 0x07) << 5) | FLAG_EXIDE | ((_tx.id >> 16) & 0x03));
		writeRegister(REG_TXBnEID8(n), (_tx.id >> 8) & 0xff);
		writeRegister(REG_TXBnEID0(n), _tx.id & 0xff);
	} else {
		writeRegister(REG_TXBnSIDH(n), _tx.id >> 3);
		writeRegister(REG_TXBnSIDL(n), _tx.id << 5);
		writeRegister(REG_TXBnEID8(n), 0x00);
		writeRegister(REG_TXBnEID0(n), 0x00);
	}
	
	if(_tx.rtr == true)
	{
		writeRegister(REG_TXBnDLC(n), 0x40 | _tx.length);
	} else {
		writeRegister(REG_TXBnDLC(n), _tx.length);
		
		for(uint8_t i = 0; i < _tx.length; ++i)
		{
			writeRegister(REG_TXBnD0(n) + i, _tx.data[i]);
		}
	}
	
	writeRegister(REG_TXBnCTRL(n), 0x08);
	
	bool aborted = false;
	while(readRegister(REG_TXBnCTRL(n)) & 0x08)
	{
		if(readRegister(REG_TXBnCTRL(n)) & 0x10)
		{
			aborted = true;
			
			modifyRegister(REG_CANCTRL, 0x10, 0x10);
		}
	}
	if(aborted)
	{
		// clear abort command
		modifyRegister(REG_CANCTRL, 0x10, 0x00);
	}
	
	modifyRegister(REG_CANINTF, FLAG_TXnIF(n), 0x00);
	
	return ((readRegister(REG_TXBnCTRL(n)) & 0x70) ? false : true);
}





uint8_t MCP2515Class::parsePacket()
{
	uint8_t n;
	uint8_t intf = readRegister(REG_CANINTF);
	if(intf & FLAG_RXnIF(0))
		n = 0;
	else if(intf & FLAG_RXnIF(1))
		n = 1;
	else
	{
		_rx.id = NO_CAN_ID;
		
		return 0;
	}
	
	_rx.extended = (readRegister(REG_RXBnSIDL(n)) & FLAG_IDE) ? true : false;
	
	uint32_t idA = ((readRegister(REG_RXBnSIDH(n)) << 3) & 0x07f8) | ((readRegister(REG_RXBnSIDL(n)) >> 5) & 0x07);
	if(_rx.extended == true)
	{
		uint32_t idB = (((uint32_t)(readRegister(REG_RXBnSIDL(n)) & 0x03) << 16) & 0x30000) | ((readRegister(REG_RXBnEID8(n)) << 8) & 0xff00) | readRegister(REG_RXBnEID0(n));
		
		_rx.id = (idA << 18) | idB;
		_rx.rtr = (readRegister(REG_RXBnDLC(n)) & FLAG_RTR) ? true : false;
	}
	else
	{
		_rx.id = idA;
		_rx.rtr = (readRegister(REG_RXBnSIDL(n)) & FLAG_SRR) ? true : false;
	}
	
	_rx.dlc = readRegister(REG_RXBnDLC(n)) & 0x0f;
	
	if(_rx.rtr == true)
		_rx.length = 0;
	else
	{
		_rx.length = _rx.dlc;
		
		for(uint8_t i = 0; i < _rx.length; ++i)
		{
			_rx.data[i] = readRegister(REG_RXBnD0(n) + i);
		}
	}
	
	modifyRegister(REG_CANINTF, FLAG_RXnIF(n), 0x00);
	
	return _rx.dlc;
}





bool MCP2515Class::filter(uint16_t id, uint16_t mask)
{
	id &= 0x7ff;
	mask &= 0x7ff;
	
	if( writeReadRegister(REG_CANCTRL, 0x80) == false ) return false;
	
	uint8_t n;
	for(n = 0; n < 2; ++n)
	{
		// standard only
		writeRegister(REG_RXBnCTRL(n), FLAG_RXM0);
		writeRegister(REG_RXBnCTRL(n), FLAG_RXM0);

		writeRegister(REG_RXMnSIDH(n), mask >> 3);
		writeRegister(REG_RXMnSIDL(n), mask << 5);
		writeRegister(REG_RXMnEID8(n), 0);
		writeRegister(REG_RXMnEID0(n), 0);
	}
	
	for(n = 0; n < 6; ++n)
	{
		writeRegister(REG_RXFnSIDH(n), id >> 3);
		writeRegister(REG_RXFnSIDL(n), id << 5);
		writeRegister(REG_RXFnEID8(n), 0);
		writeRegister(REG_RXFnEID0(n), 0);
	}
	
	if( writeReadRegister(REG_CANCTRL, 0x00) == false ) return false;
	
	return true;
}

bool MCP2515Class::filterExtended(uint32_t id, uint32_t mask)
{
	id &= 0x1FFFFFFF;
	mask &= 0x1FFFFFFF;
	
	if( writeReadRegister(REG_CANCTRL, 0x80) == false ) return false;
	
	uint8_t n;
	for(n = 0; n < 2; ++n)
	{
		// extended only
		writeRegister(REG_RXBnCTRL(n), FLAG_RXM1);
		writeRegister(REG_RXBnCTRL(n), FLAG_RXM1);

		writeRegister(REG_RXMnSIDH(n), mask >> 21);
		writeRegister(REG_RXMnSIDL(n), (((mask >> 18) & 0x03) << 5) | FLAG_EXIDE | ((mask >> 16) & 0x03));
		writeRegister(REG_RXMnEID8(n), (mask >> 8) & 0xff);
		writeRegister(REG_RXMnEID0(n), mask & 0xff);
	}
	
	for(n = 0; n < 6; ++n)
	{
		writeRegister(REG_RXFnSIDH(n), id >> 21);
		writeRegister(REG_RXFnSIDL(n), (((id >> 18) & 0x03) << 5) | FLAG_EXIDE | ((id >> 16) & 0x03));
		writeRegister(REG_RXFnEID8(n), (id >> 8) & 0xff);
		writeRegister(REG_RXFnEID0(n), id & 0xff);
	}
	
	if( writeReadRegister(REG_CANCTRL, 0x00) == false ) return false;
	
	return true;
}





bool MCP2515Class::cmd_observe()
{
	return writeReadRegister(REG_CANCTRL, 0x60);
}

bool MCP2515Class::cmd_loopback()
{
	return writeReadRegister(REG_CANCTRL, 0x40);
}

bool MCP2515Class::cmd_sleep()
{
	return writeReadRegister(REG_CANCTRL, 0x01);
}

bool MCP2515Class::cmd_wakeup()
{
	return writeReadRegister(REG_CANCTRL, 0x00);
}






/*
void MCP2515Class::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    byte b = readRegister(i);

    out.print("0x");
    if (i < 16) {
      out.print('0');
    }
    out.print(i, HEX);
    out.print(": 0x");
    if (b < 16) {
      out.print('0');
    }
    out.println(b, HEX);
  }
}
*/





void MCP2515Class::reset()
{
	DeviceActivate();
	uint8_t spi_data[] = {0xC0};
	_spi_interface->TransmitData(this, spi_data, sizeof(spi_data));
	DeviceDeactivate();
	
	//delayMicroseconds(10);
	HAL_Delay(5);
	
	return;
}

uint8_t MCP2515Class::readRegister(uint8_t address)
{
	DeviceActivate();
	uint8_t spi_data[] = {0x03, address};
	_spi_interface->TransmitData(this, spi_data, sizeof(spi_data));
	_spi_interface->ReceiveData(this, spi_data, 1);
	DeviceDeactivate();
	
	return spi_data[0];
}

void MCP2515Class::modifyRegister(uint8_t address, uint8_t mask, uint8_t value)
{
	DeviceActivate();
	uint8_t spi_data[] = {0x05, address, mask, value};
	_spi_interface->TransmitData(this, spi_data, sizeof(spi_data));
	DeviceDeactivate();
	
	return;
}

void MCP2515Class::writeRegister(uint8_t address, uint8_t value)
{
	DeviceActivate();
	uint8_t spi_data[] = {0x02, address, value};
	_spi_interface->TransmitData(this, spi_data, sizeof(spi_data));
	DeviceDeactivate();
	
	return;
}

bool MCP2515Class::writeReadRegister(uint8_t address, uint8_t value)
{
	writeRegister(address, value);
	if(readRegister(address) != value) return false;
	
	return true;
}
