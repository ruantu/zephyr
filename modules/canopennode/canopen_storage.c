/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/eeprom.h>

#define LOG_LEVEL CONFIG_CANOPEN_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(CO_eeprom);

#include "storage/CO_eeprom.h"
#include "301/crc16-ccitt.h"

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE

#define EEPROM_PAGESIZE 8

/*
 * The first half of Eeprom is used for automatic storage, and the
 * second half is used for protected storage.
 */
static size_t eepromAddrNextAuto = 0;
static size_t eepromAddrNextProt = 0;
static size_t eeprom_size = 0;

bool_t CO_eeprom_init(void *storageModule)
{
	if (!device_is_ready((const struct device *)storageModule))
	{
		LOG_ERR("Error: Device \"%s\" is not ready; "
				"check the driver initialization logs for errors.",
				((const struct device *)storageModule)->name);
		return false;
	}

	eeprom_size = eeprom_get_size((const struct device *)storageModule);
	eepromAddrNextAuto = 0;
	eepromAddrNextProt = eeprom_size / 2;

	return true;
}

size_t CO_eeprom_getAddr(void *storageModule, bool_t isAuto, size_t len,
						 bool_t *overflow)
{
	size_t addr;

	if (isAuto)
	{
		/* auto storage is processed byte by byte, no alignment necessary */
		addr = eepromAddrNextAuto;
		eepromAddrNextAuto += len;
		if (eepromAddrNextAuto > (eeprom_size / 2))
		{
			*overflow = true;
		}
	}
	else
	{
		/* addresses for storage on command must be page aligned */
		addr = eepromAddrNextProt;
		size_t lenAligned = len & (~(EEPROM_PAGESIZE - 1));
		if (lenAligned < len)
		{
			lenAligned += EEPROM_PAGESIZE;
		}
		eepromAddrNextProt += lenAligned;

		if (eepromAddrNextProt > eeprom_size)
		{
			*overflow = true;
		}
	}

	return addr;
}

void CO_eeprom_readBlock(void *storageModule, uint8_t *data, size_t eepromAddr,
						 size_t len)
{
	int ret;
	ret = eeprom_read((const struct device *)storageModule, eepromAddr, data, len);
	if (ret < 0)
	{
		LOG_ERR("Error: Couldn't read eeprom: ret: %d.", ret);
	}
}

bool_t CO_eeprom_writeBlock(void *storageModule, uint8_t *data, size_t eepromAddr,
							size_t len)
{
	int ret;

	ret = eeprom_write((const struct device *)storageModule, eepromAddr, data, len);
	if (ret < 0)
	{
		LOG_ERR("Error: Couldn't write eeprom: ret: %d.", ret);
		return false;
	}
	return true;
}

uint16_t CO_eeprom_getCrcBlock(void *storageModule, size_t eepromAddr, size_t len)
{
	uint16_t crc = 0;
	uint8_t buf[EEPROM_PAGESIZE];
	int ret;

	while (len > 0)
	{
		uint8_t subLen = len <= EEPROM_PAGESIZE ? (uint8_t)len : EEPROM_PAGESIZE;

		/* update crc from data part */
		ret = eeprom_read((const struct device *)storageModule, eepromAddr, buf, subLen);
		if (ret < 0)
		{
			LOG_ERR("Error: Couldn't read eeprom: ret: %d.", ret);
		}

		crc = crc16_ccitt(buf, subLen, crc);
		len -= subLen;
		eepromAddr += subLen;
	}

	return crc;
}

bool_t CO_eeprom_updateByte(void *storageModule, uint8_t data,
							size_t eepromAddr)
{
	int ret;
	uint8_t buf;

	ret = eeprom_read((const struct device *)storageModule, eepromAddr, &buf, sizeof(buf));
	if (ret < 0)
	{
		LOG_ERR("Error: Couldn't read eeprom: ret: %d.", ret);
		return false;
	}
	if (buf != data)
	{
		ret = eeprom_write((const struct device *)storageModule, eepromAddr, &data, sizeof(data));
		if (ret < 0)
		{
			LOG_ERR("Error: Couldn't write eeprom: ret: %d.", ret);
			return false;
		}
	}

	return true;
}

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */
