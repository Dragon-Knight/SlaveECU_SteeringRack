#pragma once
#include <inttypes.h>
#include <EasyPinD.h>
#include <SPIManager.h>
#include <drivers/SPI_MCP2515.h>
#include <SteeringAngleSensor.h>

extern SPI_HandleTypeDef hspi1;

namespace CAN_SPI
{
	
	static constexpr EasyPinD::d_pin_t CAN1_SPI_CS = {GPIOB, GPIO_PIN_11};
	static constexpr EasyPinD::d_pin_t CAN1_RX_INT = {GPIOB, GPIO_PIN_10};
	static constexpr EasyPinD::d_pin_t CAN1_RS = {GPIOB, GPIO_PIN_2};
	static constexpr EasyPinD::d_pin_t CAN1_SENS_PWR = {GPIOA, GPIO_PIN_3};
	
	static constexpr EasyPinD::d_pin_t CAN2_SPI_CS = {GPIOB, GPIO_PIN_3};
	static constexpr EasyPinD::d_pin_t CAN2_RX_INT = {GPIOB, GPIO_PIN_4};
	static constexpr EasyPinD::d_pin_t CAN2_RS = {GPIOB, GPIO_PIN_8};
	static constexpr EasyPinD::d_pin_t CAN2_SENS_PWR = {GPIOA, GPIO_PIN_4};
	
	
	
	void OnSteeringAngleSensorError(uint8_t id, SteeringAngleSensorBase::error_t code);
	
	
	
	SteeringAngleSensor sensor1(SteeringRack::RACK_1, OnSteeringAngleSensorError);
	SteeringAngleSensor sensor2(SteeringRack::RACK_2, OnSteeringAngleSensorError);
	
	
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
	
	
	void CAN_RX(SteeringRack::rack_id_t id, uint32_t address, uint8_t *data, uint8_t length)
	{
		bool result;
		uint32_t time;
		
		switch(id)
		{
			case SteeringRack::RACK_1:
			{
				time = HAL_GetTick();
				
				result = sensor1.PutPacket(time, address, data, length);
				if(result == true)
				{
					SteeringRack::OnDataSensor( id, sensor1.data_float->angle, sensor1.data_float->roll, sensor1.data_float->dt );

					//DEBUG_LOG_TOPIC("ExCAN RX", "Port: %d, Addr: %04X, Angle: %+05d, Roll: %+05d, Err: %02d\n", id, address, sensor1.data_int->angle, sensor1.data_int->roll, sensor1.data_int->error);
				}
				
				break;
			}
			case SteeringRack::RACK_2:
			{
				time = HAL_GetTick();
				
				result = sensor2.PutPacket(time, address, data, length);
				if(result == true)
				{
					SteeringRack::OnDataSensor( id, (0.0 - sensor2.data_float->angle), sensor2.data_float->roll, sensor2.data_float->dt );
				}
				
				break;
			}
		}
		
		return;
	}
	
	void OnSteeringAngleSensorError(uint8_t id, SteeringAngleSensorBase::error_t code)
	{
		Logger.Printf("Sensor %d: error code: %d", id, code).PrintNewLine();
		SteeringRack::OnErrorSensor((SteeringRack::rack_id_t)id, code);
		
		return;
	}
	
	
	
	SPIManager<2> manager(SPI_Config, SPI_Write, SPI_Read, SPI_WriteRead);
	SPI_MCP2515 obj1(CAN1_SPI_CS, CAN1_RX_INT, SPI_BAUDRATEPRESCALER_8);
	SPI_MCP2515 obj2(CAN2_SPI_CS, CAN2_RX_INT, SPI_BAUDRATEPRESCALER_8);
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

		obj1.begin(8000000, 500000, [](uint32_t address, uint8_t *data, uint8_t length){ CAN_RX(SteeringRack::RACK_1, address, data, length); });
		obj2.begin(8000000, 500000, [](uint32_t address, uint8_t *data, uint8_t length){ CAN_RX(SteeringRack::RACK_2, address, data, length); });


		pin_sens_1.On();
		pin_sens_2.On();

		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		manager.Tick(current_time);
		sensor1.Tick(current_time);
		sensor2.Tick(current_time);

		/*
		sensor1.CheckLost(current_time, [](){
			DEBUG_LOG_TOPIC("ExCAN ERR", "Lost sensor 1\n");
			Leds::obj.SetOn(Leds::LED_RED, 20);
		});
		sensor2.CheckLost(current_time, [](){
			DEBUG_LOG_TOPIC("ExCAN ERR", "Lost sensor 2\n");
			Leds::obj.SetOn(Leds::LED_RED, 20);
		});
		*/
		
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
