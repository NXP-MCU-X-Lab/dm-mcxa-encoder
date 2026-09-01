#ifndef APP_ENCODER_STORAGE_H_
#define APP_ENCODER_STORAGE_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_encoder.h"
#include "app_tformat.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_STORAGE_FLASH_BASE         (0x0003C000UL)
#define ENCODER_STORAGE_FLASH_SIZE         (0x00004000UL)
#define ENCODER_STORAGE_SECTOR_SIZE        (0x00002000UL)
#define ENCODER_STORAGE_RECORD_SIZE        (1024U)
#define ENCODER_STORAGE_EEPROM_SIZE        TFORMAT_EEPROM_SIZE
#define ENCODER_STORAGE_RECORD_COUNT       (ENCODER_STORAGE_FLASH_SIZE / ENCODER_STORAGE_RECORD_SIZE)
#define ENCODER_STORAGE_RECORDS_PER_SECTOR (ENCODER_STORAGE_SECTOR_SIZE / ENCODER_STORAGE_RECORD_SIZE)

#define ENCODER_STORAGE_MAGIC   (0x324C4345UL)
#define ENCODER_STORAGE_VERSION (3U)

typedef struct _encoder_persistent_config
{
    encoder_calibration_t calibration;
    encoder_cal_quality_t quality;
    uint8_t eeprom[ENCODER_STORAGE_EEPROM_SIZE];
} encoder_persistent_config_t;

typedef struct _encoder_storage_record
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    float cal_values[12];
    uint32_t sample_count;
    uint32_t status;
    float mag16;
    float mag15;
    uint8_t eeprom[ENCODER_STORAGE_EEPROM_SIZE];
    uint8_t reserved[182];
    uint32_t crc32;
} encoder_storage_record_t;

void EncoderStorage_PackRecord(encoder_storage_record_t *record,
                               const encoder_persistent_config_t *config,
                               uint32_t sequence);
bool EncoderStorage_ValidateRecord(const encoder_storage_record_t *record);
bool EncoderStorage_UnpackRecord(const encoder_storage_record_t *record,
                                 encoder_persistent_config_t *config,
                                 uint32_t *sequence);
bool EncoderStorage_SelectLatestRecord(const encoder_storage_record_t *records,
                                       uint32_t count,
                                       encoder_storage_record_t *latest,
                                       uint32_t *latest_index);

bool EncoderStorage_Load(encoder_persistent_config_t *config, uint32_t *sequence);
bool EncoderStorage_Save(const encoder_persistent_config_t *config);
bool EncoderStorage_Erase(void);

#ifdef ENCODER_STORAGE_HOST_TEST
void EncoderStorage_HostReset(void);
void EncoderStorage_HostFailAfter(int32_t operation_count);
#endif

#ifdef __cplusplus
}
#endif

#endif /* APP_ENCODER_STORAGE_H_ */
