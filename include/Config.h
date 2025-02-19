#pragma once
#include <inttypes.h>
#include <CUtils.h>
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












	struct __attribute__((packed)) eeprom_page_t
	{
		// 1 байт - Версия
		uint8_t version = 0x03;

		// 3 байта - Счётчик записей
		uint32_t counter : 24;

		// 26 байт - Полезная нагрузка
		uint8_t payload[26];

		// 2 байта - Контрольная сумма
		uint16_t crc;
	};
	static_assert(sizeof(eeprom_page_t) == DATA_PAGE_SIZE, "The structure must be of size DATA_PAGE_SIZE!");


	static constexpr uint16_t PAYLOAD_SIZE = sizeof(eeprom_page_t::payload);
	static constexpr uint16_t CRC_LENGTH = sizeof(eeprom_page_t) - 2;
	
	
	
	bool LoadPage2(uint8_t idx, const eeprom_page_t &page)
	{
		bool result = false;
		
		SPI::eeprom.ReadPage(idx, ((uint8_t *) &page));
		if(page.version > 0x00 && page.version < 0xFF)
		{
			if(CRC16_XModem( ((uint8_t *) &page), CRC_LENGTH ) == page.crc)
			{
				result = true;
			}
		}
		
		return result;
	}
	
	bool SavePage2(uint8_t idx, const eeprom_page_t &page)
	{
		bool result = false;

		SPI::eeprom.WritePage(idx, ((uint8_t *) &page));
		result = true;
		
		return result;
	}
	
	bool UpdatePage2(uint8_t idx, const eeprom_page_t &page)
	{
		bool result = false;
		
		eeprom_page_t page_in_ee;
		if(LoadPage2(idx, page_in_ee) == true)
		{
			if( memcmp( ((uint8_t *) &page_in_ee.payload), ((uint8_t *) &page.payload), PAYLOAD_SIZE ) != 0 )
			{
				memcpy( ((uint8_t *) &page_in_ee.payload), ((uint8_t *) &page.payload), PAYLOAD_SIZE );
				page_in_ee.counter++;
				page_in_ee.crc = CRC16_XModem( ((uint8_t *) &page_in_ee), CRC_LENGTH );
				
				result = SavePage2(idx, page);
			}
		}
		
		return result;
	}

	void LoadConfig2()
	{
		uint16_t payload_length = sizeof(eeprom_body_t);
		uint8_t  payload_count = (sizeof(eeprom_body_t) + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE;
		uint16_t payload_offset = 0;
		
		eeprom_page_t page;
		for(uint8_t idx = 0; idx < payload_count; ++idx)
		{
			if(LoadPage2(idx, page) == true)
			{
				memcpy( ((uint8_t *) &config) + payload_offset, ((uint8_t *) &page.payload), PAYLOAD_SIZE );
			}
			else
			{
				DEBUG_LOG_TOPIC("EELoad", "Load error, page: %d, data:\n", idx);
				DEBUG_LOG_ARRAY_HEX("EE", ((uint8_t *) &page), sizeof(page));
				DEBUG_LOG_NEW_LINE();
			}
			
			payload_offset += PAYLOAD_SIZE;
		}
	}
	
	void SaveConfig2()
	{
		uint16_t payload_length = sizeof(eeprom_body_t);
		uint8_t  payload_count = (sizeof(eeprom_body_t) + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE;
		uint16_t payload_offset = 0;
		
		eeprom_page_t page;
		for(uint8_t idx = 0; idx < payload_count; ++idx)
		{
			/*
			if(LoadPage2(idx, page) == true)
			{
				if( memcmp( ((uint8_t *) &config) + payload_offset, ((uint8_t *) &page.payload), PAYLOAD_SIZE ) != 0 )
				{
					memcpy( ((uint8_t *) &page.payload), ((uint8_t *) &config) + payload_offset, PAYLOAD_SIZE );
					page.counter++;
					page.crc = CRC16_XModem( ((uint8_t *) &page), CRC_LENGTH );
					
					SavePage2(idx, page);
				}
			}
			*/
			
			memcpy( ((uint8_t *) &page.payload), ((uint8_t *) &config) + payload_offset, PAYLOAD_SIZE );
			if(UpdatePage2(idx, page) == true)
			{

			}
			else
			{
				DEBUG_LOG_TOPIC("EESave", "Load error, page: %d, data:\n", idx);
				DEBUG_LOG_ARRAY_HEX("EE", ((uint8_t *) &page), sizeof(page));
				DEBUG_LOG_NEW_LINE();
			}
			
			payload_offset += PAYLOAD_SIZE;
		}
	}













	
	
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
