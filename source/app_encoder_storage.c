#include "app_encoder_storage.h"

#include <stddef.h>
#include <string.h>

#ifndef ENCODER_STORAGE_HOST_TEST
#include "fsl_common.h"
#include "fsl_romapi.h"
#endif

typedef char encoder_storage_block_must_be_128_bytes
    [(sizeof(encoder_storage_block_t) == ENCODER_STORAGE_BLOCK_SIZE) ? 1 : -1];

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    uint32_t i;

    while (length > 0U)
    {
        crc ^= (uint32_t)(*data);
        for (i = 0U; i < 8U; i++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
        data++;
        length--;
    }

    return crc;
}

static uint32_t block_crc32(const encoder_storage_block_t *block)
{
    return crc32_update(0xFFFFFFFFUL,
                        (const uint8_t *)block,
                        (uint32_t)offsetof(encoder_storage_block_t, crc32)) ^
           0xFFFFFFFFUL;
}

static void calibration_to_flat(const encoder_calibration_t *calibration, float values[12])
{
    values[0] = calibration->a1.center_sin;
    values[1] = calibration->a1.center_cos;
    values[2] = calibration->a1.t00;
    values[3] = calibration->a1.t10;
    values[4] = calibration->a1.t11;
    values[5] = calibration->a2.center_sin;
    values[6] = calibration->a2.center_cos;
    values[7] = calibration->a2.t00;
    values[8] = calibration->a2.t10;
    values[9] = calibration->a2.t11;
    values[10] = calibration->phase_a1_zero_deg;
    values[11] = calibration->phase_a2_zero_deg;
}

static void flat_to_calibration(const float values[12], encoder_calibration_t *calibration)
{
    calibration->a1.center_sin = values[0];
    calibration->a1.center_cos = values[1];
    calibration->a1.t00 = values[2];
    calibration->a1.t10 = values[3];
    calibration->a1.t11 = values[4];
    calibration->a2.center_sin = values[5];
    calibration->a2.center_cos = values[6];
    calibration->a2.t00 = values[7];
    calibration->a2.t10 = values[8];
    calibration->a2.t11 = values[9];
    calibration->phase_a1_zero_deg = values[10];
    calibration->phase_a2_zero_deg = values[11];
    calibration->valid = true;
}

static bool block_is_erased(const encoder_storage_block_t *block)
{
    const uint8_t *data = (const uint8_t *)block;
    uint32_t i;

    for (i = 0U; i < sizeof(*block); i++)
    {
        if (data[i] != 0xFFU)
        {
            return false;
        }
    }

    return true;
}

void EncoderStorage_PackBlock(encoder_storage_block_t *block,
                              const encoder_calibration_t *calibration,
                              const encoder_cal_quality_t *quality,
                              uint32_t sequence,
                              uint32_t flags)
{
    if ((block == NULL) || (calibration == NULL))
    {
        return;
    }

    memset(block, 0xFF, sizeof(*block));
    block->magic = ENCODER_STORAGE_MAGIC;
    block->version = ENCODER_STORAGE_VERSION;
    block->size = (uint16_t)sizeof(*block);
    block->sequence = sequence;
    block->flags = flags;
    calibration_to_flat(calibration, block->cal_values);

    if (quality != NULL)
    {
        block->sample_count = quality->sample_count;
        block->status = quality->status;
        block->mag16 = quality->mag16;
        block->mag15 = quality->mag15;
    }
    else
    {
        block->sample_count = 0U;
        block->status = ENCODER_STATUS_OK;
        block->mag16 = 0.0f;
        block->mag15 = 0.0f;
    }

    block->crc32 = block_crc32(block);
}

bool EncoderStorage_ValidateBlock(const encoder_storage_block_t *block)
{
    if ((block == NULL) || block_is_erased(block))
    {
        return false;
    }

    if ((block->magic != ENCODER_STORAGE_MAGIC) ||
        (block->version != ENCODER_STORAGE_VERSION) ||
        (block->size != sizeof(*block)))
    {
        return false;
    }

    return block->crc32 == block_crc32(block);
}

bool EncoderStorage_UnpackBlock(const encoder_storage_block_t *block,
                                encoder_calibration_t *calibration,
                                encoder_cal_quality_t *quality,
                                uint32_t *sequence,
                                uint32_t *flags)
{
    if (!EncoderStorage_ValidateBlock(block))
    {
        return false;
    }

    if (calibration != NULL)
    {
        flat_to_calibration(block->cal_values, calibration);
    }

    if (quality != NULL)
    {
        quality->sample_count = block->sample_count;
        quality->status = block->status;
        quality->mag16 = block->mag16;
        quality->mag15 = block->mag15;
    }

    if (sequence != NULL)
    {
        *sequence = block->sequence;
    }

    if (flags != NULL)
    {
        *flags = block->flags;
    }

    return true;
}

bool EncoderStorage_SelectLatestBlock(const encoder_storage_block_t *blocks,
                                      uint32_t count,
                                      encoder_storage_block_t *latest)
{
    uint32_t i;
    bool found = false;
    uint32_t best_sequence = 0U;

    if ((blocks == NULL) || (latest == NULL))
    {
        return false;
    }

    for (i = 0U; i < count; i++)
    {
        if (!EncoderStorage_ValidateBlock(&blocks[i]))
        {
            continue;
        }

        if (!found || (blocks[i].sequence > best_sequence))
        {
            *latest = blocks[i];
            best_sequence = blocks[i].sequence;
            found = true;
        }
    }

    return found;
}

#ifdef ENCODER_STORAGE_HOST_TEST

bool EncoderStorage_Load(encoder_calibration_t *calibration,
                         encoder_cal_quality_t *quality,
                         uint32_t *sequence)
{
    (void)calibration;
    (void)quality;
    (void)sequence;
    return false;
}

bool EncoderStorage_SaveFactoryCalibration(const encoder_calibration_t *calibration,
                                           const encoder_cal_quality_t *quality)
{
    (void)calibration;
    (void)quality;
    return false;
}

bool EncoderStorage_EraseFactoryCalibration(void)
{
    return false;
}

#else

static const encoder_storage_block_t *storage_blocks(void)
{
    return (const encoder_storage_block_t *)ENCODER_STORAGE_FLASH_BASE;
}

static int32_t find_first_erased_slot(const encoder_storage_block_t *blocks)
{
    uint32_t i;

    for (i = 0U; i < ENCODER_STORAGE_SLOT_COUNT; i++)
    {
        if (block_is_erased(&blocks[i]))
        {
            return (int32_t)i;
        }
    }

    return -1;
}

bool EncoderStorage_Load(encoder_calibration_t *calibration,
                         encoder_cal_quality_t *quality,
                         uint32_t *sequence)
{
    encoder_storage_block_t latest;

    if (!EncoderStorage_SelectLatestBlock(storage_blocks(), ENCODER_STORAGE_SLOT_COUNT, &latest))
    {
        return false;
    }

    return EncoderStorage_UnpackBlock(&latest, calibration, quality, sequence, NULL);
}

bool EncoderStorage_SaveFactoryCalibration(const encoder_calibration_t *calibration,
                                           const encoder_cal_quality_t *quality)
{
    flash_config_t flash_config;
    encoder_storage_block_t block;
    encoder_storage_block_t latest;
    const encoder_storage_block_t *blocks = storage_blocks();
    int32_t slot;
    uint32_t next_sequence = 1U;
    uint32_t failed_address = 0U;
    uint32_t failed_data = 0U;
    status_t status;
    uint32_t irq_mask;

    if ((calibration == NULL) || !calibration->valid)
    {
        return false;
    }

    if (EncoderStorage_SelectLatestBlock(blocks, ENCODER_STORAGE_SLOT_COUNT, &latest))
    {
        next_sequence = latest.sequence + 1U;
        if (next_sequence == 0U)
        {
            next_sequence = 1U;
        }
    }

    slot = find_first_erased_slot(blocks);
    EncoderStorage_PackBlock(&block, calibration, quality, next_sequence, 0U);

    status = FLASH_Init(&flash_config);
    if (status != kStatus_FLASH_Success)
    {
        return false;
    }

    irq_mask = DisableGlobalIRQ();
    if (slot < 0)
    {
        status = FLASH_EraseSector(&flash_config,
                                   ENCODER_STORAGE_FLASH_BASE,
                                   ENCODER_STORAGE_FLASH_SIZE,
                                   kFLASH_ApiEraseKey);
        slot = 0;
    }

    if (status == kStatus_FLASH_Success)
    {
        status = FLASH_ProgramPage(&flash_config,
                                   ENCODER_STORAGE_FLASH_BASE +
                                       ((uint32_t)slot * ENCODER_STORAGE_BLOCK_SIZE),
                                   (uint8_t *)&block,
                                   sizeof(block));
    }

    if (status == kStatus_FLASH_Success)
    {
        status = FLASH_VerifyProgram(&flash_config,
                                     ENCODER_STORAGE_FLASH_BASE +
                                         ((uint32_t)slot * ENCODER_STORAGE_BLOCK_SIZE),
                                     sizeof(block),
                                     (const uint8_t *)&block,
                                     &failed_address,
                                     &failed_data);
    }
    EnableGlobalIRQ(irq_mask);

    return status == kStatus_FLASH_Success;
}

bool EncoderStorage_EraseFactoryCalibration(void)
{
    flash_config_t flash_config;
    status_t status;
    uint32_t irq_mask;

    status = FLASH_Init(&flash_config);
    if (status != kStatus_FLASH_Success)
    {
        return false;
    }

    irq_mask = DisableGlobalIRQ();
    status = FLASH_EraseSector(&flash_config,
                               ENCODER_STORAGE_FLASH_BASE,
                               ENCODER_STORAGE_FLASH_SIZE,
                               kFLASH_ApiEraseKey);
    EnableGlobalIRQ(irq_mask);

    return status == kStatus_FLASH_Success;
}

#endif
