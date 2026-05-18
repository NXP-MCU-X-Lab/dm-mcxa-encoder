#ifndef APP_ENCODER_STORAGE_H_
#define APP_ENCODER_STORAGE_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_STORAGE_FLASH_BASE (0x0001E000UL)
#define ENCODER_STORAGE_FLASH_SIZE (0x00002000UL)
#define ENCODER_STORAGE_BLOCK_SIZE (128U)
#define ENCODER_STORAGE_SLOT_COUNT (ENCODER_STORAGE_FLASH_SIZE / ENCODER_STORAGE_BLOCK_SIZE)

#define ENCODER_STORAGE_MAGIC (0x314C4345UL)
#define ENCODER_STORAGE_VERSION (1U)

typedef struct _encoder_storage_block
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    uint32_t flags;
    float cal_values[12];
    uint32_t sample_count;
    uint32_t status;
    float mag16;
    float mag15;
    float reserved_quality[8];
    uint32_t crc32;
    uint8_t reserved_tail[12];
} encoder_storage_block_t;

void EncoderStorage_PackBlock(encoder_storage_block_t *block,
                              const encoder_calibration_t *calibration,
                              const encoder_cal_quality_t *quality,
                              uint32_t sequence,
                              uint32_t flags);
bool EncoderStorage_ValidateBlock(const encoder_storage_block_t *block);
bool EncoderStorage_UnpackBlock(const encoder_storage_block_t *block,
                                encoder_calibration_t *calibration,
                                encoder_cal_quality_t *quality,
                                uint32_t *sequence,
                                uint32_t *flags);
bool EncoderStorage_SelectLatestBlock(const encoder_storage_block_t *blocks,
                                      uint32_t count,
                                      encoder_storage_block_t *latest);

bool EncoderStorage_Load(encoder_calibration_t *calibration,
                         encoder_cal_quality_t *quality,
                         uint32_t *sequence);
bool EncoderStorage_SaveFactoryCalibration(const encoder_calibration_t *calibration,
                                           const encoder_cal_quality_t *quality);
bool EncoderStorage_EraseFactoryCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ENCODER_STORAGE_H_ */
