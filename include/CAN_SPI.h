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






class SteeringAngleSensor
{
	static constexpr uint16_t PACKET_ID = 0x00C2;
	static constexpr uint8_t PACKET_LENGTH = 7;
	
	public:

		SteeringAngleSensor() : _last_count(255)
		{}
		
		struct __attribute__((packed)) sensor_t
		{
			uint8_t crc;
			uint8_t _null;
			uint8_t counter : 4;
			uint8_t error : 4;
			int16_t roll;
			int16_t angle;
		};
		
		bool PutPacket(uint16_t id, uint8_t *raw, uint8_t length)
		{
			if(id != PACKET_ID) return false;
			if(length != PACKET_LENGTH) return false;
			
			memcpy(_raw_array, raw, length);
			_ReverseArray();
			_FixParams();
			
			if(_obj->counter == _last_count) return false;
			_last_count = _obj->counter;
			
			return true;
		}

		const sensor_t *data = (sensor_t *) _raw_array;

	private:
		
		void _ReverseArray()
		{
			uint8_t i = 0;
			uint8_t j = (PACKET_LENGTH - 1);
			uint8_t temp;
			while(i < j)
			{
				temp = _raw_array[i];
				_raw_array[i] = _raw_array[j];
				_raw_array[j] = temp;
				
				i++; j--;
			}
		}
		
		uint8_t _CalculateCRC()
		{
			uint8_t crc = 0xFF;
			for(uint8_t i = 0; i < (PACKET_LENGTH - 1); ++i)
			{
				crc -= _raw_array[i];
			}
			return crc;
		}

		void _FixParams()
		{
			_obj->angle -= 0x8000;
			_obj->roll -= 0x8000;
			
			return;
		}
		
		uint8_t _last_count;
		
		uint8_t _raw_array[PACKET_LENGTH];
		sensor_t *_obj = (sensor_t *) _raw_array;
};









	SteeringAngleSensor sensor1;
	SteeringAngleSensor sensor2;


	void CAN_RX(uint32_t address, uint8_t *data, uint8_t length)
	{
		bool result = sensor1.PutPacket(address, data, length);
		if(result == true)
		{
			DEBUG_LOG_TOPIC("ExCAN RX", "Addr: %04X, angle:%+05d, roll: %+05d, err: %02d\n", address, sensor1.data->angle, sensor1.data->roll, sensor1.data->error);
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
