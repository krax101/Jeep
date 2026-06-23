#include "storage.h"
#include "tables.h"
#include <EEPROM.h>

#define EEPROM_BASE_ADDR  0

static uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

bool storage_load(ECUConfig& cfg) {
    if (EEPROM.length() < sizeof(ECUConfig)) return false;

    uint8_t* p = (uint8_t*)&cfg;
    for (size_t i = 0; i < sizeof(ECUConfig); i++)
        p[i] = EEPROM.read(EEPROM_BASE_ADDR + i);

    if (cfg.magic != CFG_MAGIC) return false;
    if (cfg.version != CFG_VER)  return false;

    uint16_t stored_crc = cfg.checksum;
    cfg.checksum = 0;
    uint16_t calc_crc = crc16((uint8_t*)&cfg, sizeof(ECUConfig));
    cfg.checksum = stored_crc;

    return (stored_crc == calc_crc);
}

void storage_save(const ECUConfig& cfg) {
    ECUConfig tmp = cfg;
    tmp.checksum  = 0;
    tmp.checksum  = crc16((uint8_t*)&tmp, sizeof(ECUConfig));

    const uint8_t* p = (const uint8_t*)&tmp;
    for (size_t i = 0; i < sizeof(ECUConfig); i++)
        EEPROM.update(EEPROM_BASE_ADDR + i, p[i]);
}

void storage_factory_reset(ECUConfig& cfg) {
    tables_load_defaults(cfg);
    storage_save(cfg);
}
