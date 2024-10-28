#pragma once
#include <inttypes.h>
#include <EasyPinD.h>
#include <MCP2515_HAL.h>

extern SPI_HandleTypeDef hspi1;

namespace CAN_SPI
{

	static constexpr EasyPinD::d_pin_t SPI_CS_CAN1 = {GPIOB, GPIO_PIN_11};
	static constexpr EasyPinD::d_pin_t CAN1_RX_INT = {GPIOB, GPIO_PIN_10};
	static constexpr EasyPinD::d_pin_t CAN1_RS = {GPIOB, GPIO_PIN_2};
	static constexpr EasyPinD::d_pin_t CAN1_SENS_PWR = {GPIOA, GPIO_PIN_3};

	static constexpr EasyPinD::d_pin_t SPI_CS_CAN2 = {GPIOB, GPIO_PIN_3};
	static constexpr EasyPinD::d_pin_t CAN2_RX_INT = {GPIOB, GPIO_PIN_4};
	static constexpr EasyPinD::d_pin_t CAN2_RS = {GPIOB, GPIO_PIN_8};
	static constexpr EasyPinD::d_pin_t CAN2_SENS_PWR = {GPIOA, GPIO_PIN_4};





		void _ReverseArray(uint8_t *array, uint8_t length)
		{
			uint8_t i = 0;
			uint8_t j = length - 1;
			uint8_t temp;
			while(i < j)
			{
				temp = array[i];
				array[i] = array[j];
				array[j] = temp;
				
				i++;
				j--;
			}

			return;
		}




	inline void SPI_Config(const SPIManagerInterface::spi_config_t &config)
	{
		if(hspi1.Init.BaudRatePrescaler == config.prescaler && hspi1.Init.FirstBit == config.first_bit) return;

		
		hspi1.Init.BaudRatePrescaler = config.prescaler;
		hspi1.Init.FirstBit = config.first_bit;
		HAL_SPI_Init(&hspi1);
	}

	inline void SPI_Write(uint8_t *data, uint16_t length)
	{
		//HAL_SPI_Transmit(&hspi1, data, length, 100);
		HAL_SPI_WriteFast(&hspi1, data, length, 100);
	}

	inline void SPI_Read(uint8_t *data, uint16_t length)
	{
		//HAL_SPI_Receive(&hspi1, data, length, 100);
		HAL_SPI_ReadFast(&hspi1, data, length, 100);
	}

	inline void SPI_WriteRead(uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
	{
		//HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, length, 200);
		HAL_SPI_WriteReadFast(&hspi1, tx_data, rx_data, length, 200);
	}

	struct __attribute__((packed)) sensor_479452659R_r
	{
		uint8_t crc;
		uint8_t _null;
		uint8_t counter : 4;
		uint8_t error : 4;
		uint8_t _rand;
		int8_t roll;
		int16_t angle;
	};


	uint8_t CalcCRC(uint8_t *data, uint8_t length)
	{
		uint8_t crc = 0xFF;
		
		for(uint8_t i = 0; i < length; ++i)
		{
			crc -= data[i];
		}

		return crc;
	}


	void CAN_RX(uint32_t address, uint8_t *data, uint8_t length)
	{
		if(address == 0x00C2)
		{
			uint8_t crc = CalcCRC(data, length-1);
			_ReverseArray(data, length);

			sensor_479452659R_r *packet = (sensor_479452659R_r *)data;
			packet->roll -= 0x80;
			packet->angle -= 0x8000;
			
			DEBUG_LOG_TOPIC("ExCAN RX", "Addr: %04X, angle:%05d, roll: %03d, counter:%02d, err: %02d, crc:%d:%d:%d\n", address, packet->angle, packet->roll, packet->counter, packet->error, crc, packet->crc, (crc == packet->crc));
		}
		
		//DEBUG_LOG_TOPIC("ExCAN RX", "Addr: %d, Data (%d): %02X %02X %02X %02X %02X %02X %02X %02X\n", address, length, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
	}
	
	
	
	SPIManager<2> manager(SPI_Config, SPI_Write, SPI_Read, SPI_WriteRead);
	MCP2515Class obj1(SPI_CS_CAN1, CAN1_RX_INT, SPI_BAUDRATEPRESCALER_8);
	MCP2515Class obj2(SPI_CS_CAN2, CAN2_RX_INT, SPI_BAUDRATEPRESCALER_8);
	EasyPinD pin_rs_1(CAN1_RS.Port, {CAN1_RS.Pin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}, GPIO_PIN_SET);
	EasyPinD pin_sens_1(CAN1_SENS_PWR.Port, {CAN1_SENS_PWR.Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}, GPIO_PIN_RESET);
	EasyPinD pin_rs_2(CAN2_RS.Port, {CAN2_RS.Pin, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}, GPIO_PIN_SET);
	EasyPinD pin_sens_2(CAN2_SENS_PWR.Port, {CAN2_SENS_PWR.Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}, GPIO_PIN_RESET);




	inline void Setup()
	{
		pin_rs_1.Init();
		pin_sens_1.Init();
		pin_rs_2.Init();
		pin_sens_2.Init();

		manager.AddDevice(obj1);
		manager.AddDevice(obj2);

		obj1.begin(8000000, 500000, CAN_RX);
		obj2.begin(8000000, 500000, CAN_RX);

		pin_sens_1.On();
		pin_sens_2.On();

		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		manager.Tick(current_time);
		
		/*
		static uint32_t tick10 = 0;
		if(current_time - tick10 > 10)
		{
			tick10 = current_time;

			obj1.Processing(current_time);
			obj2.Processing(current_time);
		}
		*/
		
		current_time = HAL_GetTick();
		
		return;
	}
};
