#pragma once
#include <inttypes.h>
#include "ConfigData.h"

extern CRC_HandleTypeDef hcrc;

namespace Config
{
	static constexpr uint16_t EEPROM_OFFSET_MAIN = 0;
	static constexpr uint16_t DATA_SIZE = 256;
	static constexpr uint16_t DATA_H_SIZE = 9;
	static constexpr uint16_t DATA_PAGE_SIZE = SPI::eeprom.EEPROM_PAGE_SIZE;
	static constexpr uint16_t EEPROM_OFFSET_MIRROR = DATA_SIZE + EEPROM_OFFSET_MAIN;
	static constexpr uint32_t MIRROR_TIME_SYNC = 10 * 60 * 1000;
	
	static_assert(EEPROM_OFFSET_MAIN % DATA_PAGE_SIZE == 0, "EEPROM_OFFSET_MAIN must be a multiple of 32!");
	
	// Общая структура всего блока данных
	struct __attribute__((packed)) eeprom_t
	{
		// Версия формата заголовка, а так-же флаг наличия записи в блоке (если 0x00 или 0xFF, то считаем что блок не инициализирован)
		uint8_t verison = 0x02;
		
		// Счётчик записей в память
		uint32_t counter = 0x00000001;
		
		// Блок полезных данных
		eeprom_body_t body;
		
		// Заполнитель пустого места в объекте
		uint8_t _reserved[ (DATA_SIZE - DATA_H_SIZE - sizeof(eeprom_body_t)) ];
		
		// Контрольная сумма всей структуры
		uint32_t crc32;
	} obj;
	static_assert(sizeof(eeprom_t) == DATA_SIZE, "Structures should have the same size!");
	
	
	uint32_t GetCRCObj()
	{
		return HAL_CRC_Calculate(&hcrc, (uint32_t *) &obj, ((sizeof(obj) - 4) / 4));
	}
	
	void CoolSave(uint16_t eeprom_offset)
	{
		static uint8_t data[DATA_SIZE] = {};
		
		SPI::eeprom.ReadRaw(eeprom_offset, data);

		// Обновляем метаданные
		obj.counter++;
		obj.crc32 = GetCRCObj();
		
		for(uint8_t page = 0; page < (DATA_SIZE / DATA_PAGE_SIZE); ++page)
		{
			uint16_t offset = page * DATA_PAGE_SIZE;
			if( memcmp(&data[offset], &((uint8_t *) &obj)[offset], DATA_PAGE_SIZE) != 0 )
			{
				SPI::eeprom.WritePage( ((eeprom_offset / DATA_PAGE_SIZE) + page), &((uint8_t *) &obj)[offset] );
			}
		}
		
		return;
	}
	
	void CoolLoad()
	{
		uint32_t crc;
		
		// Начальная инициализацяи памяти, когда она или не инициализирована, или поменялась версия.
		if(SPI::eeprom.ReadByte(EEPROM_OFFSET_MAIN) != obj.verison)
		{
			// Считаем и заполняем CRC данных по умолчанию.
			obj.crc32 = GetCRCObj();
			
			// Используем принудительную запись, чтобы гарантировать переписать все ячейки.
			// Это удалить счётчики и прочии метаданные, но по хорошему это вызывается один раз при первом запуске платы.
			SPI::eeprom.WriteRaw(EEPROM_OFFSET_MAIN, obj);
			SPI::eeprom.WriteRaw(EEPROM_OFFSET_MIRROR, obj);
		}
		
		// Читаем основной блок и проверяем CRC
		SPI::eeprom.ReadRaw(EEPROM_OFFSET_MAIN, obj);
		crc = GetCRCObj();
		if(crc == obj.crc32) return;
		
		// Читаем запасной блок и проверяем CRC
		SPI::eeprom.ReadRaw(EEPROM_OFFSET_MIRROR, obj);
		crc = GetCRCObj();
		if(crc == obj.crc32) return;

		//Если оба блока данных повреждены, то зависаем
		Leds::obj.SetOn(Leds::LED_RED);
		Logger.Printf("EEPROM read error!").PrintNewLine();
		while(true){}
	}
	
	void Dump()
	{
		Logger.Printf("EEPROM Dump(%d): ", SPI::eeprom.EEPROM_MEM_SIZE);
		uint8_t data;
		for(uint16_t i = 0; i < SPI::eeprom.EEPROM_MEM_SIZE; ++i)
		{
			if(i % 16 == 0)
			{
				Logger.Printf("\n %04X | ", i);
			}
			
			if(i % 16 == 8)
			{
				Logger.Print(" ");
			}
			
			data = SPI::eeprom.ReadByte(i);
			Logger.Printf("%02X ", data);
		}
		Logger.PrintNewLine();
		
		return;
	}
	
	
	inline void Setup()
	{
		#warning fix it!
		CoolLoad();
		
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		static uint32_t last_save_mirror_time = 0;
		if(current_time - last_save_mirror_time > MIRROR_TIME_SYNC)
		{
			last_save_mirror_time = current_time;

			CoolSave(EEPROM_OFFSET_MIRROR);
		}

		current_time = HAL_GetTick();

		static uint32_t last_save_time = 0;
		if(current_time - last_save_time > 30000)
		{
			last_save_time = current_time;
			
			CoolSave(EEPROM_OFFSET_MAIN);
		}
		
		current_time = HAL_GetTick();
		
		return;
	}
};
