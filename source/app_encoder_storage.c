#include "app_encoder_storage.h"

#include <stddef.h>
#include <string.h>

#ifndef ENCODER_STORAGE_HOST_TEST
#include "fsl_common.h"
#include "fsl_romapi.h"
#endif

typedef char encoder_storage_record_must_match_configured_size
    [(sizeof(encoder_storage_record_t) == ENCODER_STORAGE_RECORD_SIZE) ? 1 : -1];

static encoder_storage_record_t s_latest_record;
static encoder_storage_record_t s_write_record;

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    uint32_t i;

    while (length-- > 0U)
    {
        crc ^= *data++;
        for (i = 0U; i < 8U; i++)
        {
            crc = ((crc & 1U) != 0U) ? (crc >> 1U) ^ 0xEDB88320UL : crc >> 1U;
        }
    }

    return crc;
}

static uint32_t record_crc32(const encoder_storage_record_t *record)
{
    return crc32_update(0xFFFFFFFFUL,
                        (const uint8_t *)record,
                        (uint32_t)offsetof(encoder_storage_record_t, crc32)) ^
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

static bool record_is_erased(const encoder_storage_record_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t i;

    for (i = 0U; i < sizeof(*record); i++)
    {
        if (data[i] != 0xFFU)
        {
            return false;
        }
    }

    return true;
}

static bool sequence_is_newer(uint32_t candidate, uint32_t reference)
{
    return (int32_t)(candidate - reference) > 0;
}

void EncoderStorage_PackRecord(encoder_storage_record_t *record,
                               const encoder_persistent_config_t *config,
                               uint32_t sequence)
{
    memset(record, 0xFF, sizeof(*record));
    record->magic = ENCODER_STORAGE_MAGIC;
    record->version = ENCODER_STORAGE_VERSION;
    record->size = (uint16_t)sizeof(*record);
    record->sequence = sequence;
    calibration_to_flat(&config->calibration, record->cal_values);
    record->sample_count = config->quality.sample_count;
    record->status = config->quality.status;
    record->mag16 = config->quality.mag16;
    record->mag15 = config->quality.mag15;
    memcpy(record->eeprom, config->eeprom, sizeof(record->eeprom));
    record->crc32 = record_crc32(record);
}

bool EncoderStorage_ValidateRecord(const encoder_storage_record_t *record)
{
    return !record_is_erased(record) &&
           (record->magic == ENCODER_STORAGE_MAGIC) &&
           (record->version == ENCODER_STORAGE_VERSION) &&
           (record->size == sizeof(*record)) &&
           (record->crc32 == record_crc32(record));
}

bool EncoderStorage_UnpackRecord(const encoder_storage_record_t *record,
                                 encoder_persistent_config_t *config,
                                 uint32_t *sequence)
{
    if (!EncoderStorage_ValidateRecord(record))
    {
        return false;
    }

    flat_to_calibration(record->cal_values, &config->calibration);
    config->quality.sample_count = record->sample_count;
    config->quality.status = record->status;
    config->quality.mag16 = record->mag16;
    config->quality.mag15 = record->mag15;
    memcpy(config->eeprom, record->eeprom, sizeof(config->eeprom));
    if (sequence != NULL)
    {
        *sequence = record->sequence;
    }

    return true;
}

bool EncoderStorage_SelectLatestRecord(const encoder_storage_record_t *records,
                                       uint32_t count,
                                       encoder_storage_record_t *latest,
                                       uint32_t *latest_index)
{
    uint32_t i;
    bool found = false;

    for (i = 0U; i < count; i++)
    {
        if (!EncoderStorage_ValidateRecord(&records[i]))
        {
            continue;
        }

        if (!found || sequence_is_newer(records[i].sequence, latest->sequence))
        {
            *latest = records[i];
            if (latest_index != NULL)
            {
                *latest_index = i;
            }
            found = true;
        }
    }

    return found;
}

#ifdef ENCODER_STORAGE_HOST_TEST

static uint8_t s_host_flash[ENCODER_STORAGE_FLASH_SIZE];
static int32_t s_host_fail_after = -1;

static bool host_operation_allowed(void)
{
    if (s_host_fail_after < 0)
    {
        return true;
    }
    if (s_host_fail_after == 0)
    {
        return false;
    }
    s_host_fail_after--;
    return true;
}

void EncoderStorage_HostReset(void)
{
    memset(s_host_flash, 0xFF, sizeof(s_host_flash));
    s_host_fail_after = -1;
}

void EncoderStorage_HostFailAfter(int32_t operation_count)
{
    s_host_fail_after = operation_count;
}

static const encoder_storage_record_t *storage_records(void)
{
    return (const encoder_storage_record_t *)s_host_flash;
}

static bool storage_erase(uint32_t address)
{
    if (!host_operation_allowed())
    {
        return false;
    }
    memset(&s_host_flash[address - ENCODER_STORAGE_FLASH_BASE], 0xFF, ENCODER_STORAGE_SECTOR_SIZE);
    return true;
}

static bool storage_program(uint32_t address, const uint8_t *data)
{
    uint8_t *destination;
    uint32_t i;

    if (!host_operation_allowed())
    {
        return false;
    }

    destination = &s_host_flash[address - ENCODER_STORAGE_FLASH_BASE];
    for (i = 0U; i < 128U; i++)
    {
        destination[i] &= data[i];
    }
    return true;
}

static bool storage_verify(uint32_t address, const uint8_t *data, uint32_t size)
{
    return memcmp(&s_host_flash[address - ENCODER_STORAGE_FLASH_BASE], data, size) == 0;
}

#else

static flash_config_t s_flash_config;

static const encoder_storage_record_t *storage_records(void)
{
    return (const encoder_storage_record_t *)ENCODER_STORAGE_FLASH_BASE;
}

static bool storage_erase(uint32_t address)
{
    return FLASH_EraseSector(&s_flash_config,
                             address,
                             ENCODER_STORAGE_SECTOR_SIZE,
                             kFLASH_ApiEraseKey) == kStatus_FLASH_Success;
}

static bool storage_program(uint32_t address, const uint8_t *data)
{
    return FLASH_ProgramPage(&s_flash_config, address, (uint8_t *)data, 128U) ==
           kStatus_FLASH_Success;
}

static bool storage_verify(uint32_t address, const uint8_t *data, uint32_t size)
{
    uint32_t failed_address;
    uint32_t failed_data;

    return FLASH_VerifyProgram(&s_flash_config,
                               address,
                               size,
                               data,
                               &failed_address,
                               &failed_data) == kStatus_FLASH_Success;
}

#endif

static int32_t find_erased_record(uint32_t sector)
{
    const encoder_storage_record_t *records = storage_records();
    const uint32_t first = sector * ENCODER_STORAGE_RECORDS_PER_SECTOR;
    uint32_t i;

    for (i = 0U; i < ENCODER_STORAGE_RECORDS_PER_SECTOR; i++)
    {
        if (record_is_erased(&records[first + i]))
        {
            return (int32_t)(first + i);
        }
    }

    return -1;
}

static bool program_record(uint32_t index, const encoder_storage_record_t *record)
{
    const uint32_t address = ENCODER_STORAGE_FLASH_BASE + (index * ENCODER_STORAGE_RECORD_SIZE);
    const uint8_t *data = (const uint8_t *)record;
    uint32_t offset;

    for (offset = 0U; offset < sizeof(*record); offset += 128U)
    {
        if (!storage_program(address + offset, data + offset))
        {
            return false;
        }
    }
    return storage_verify(address, data, sizeof(*record));
}

bool EncoderStorage_Load(encoder_persistent_config_t *config, uint32_t *sequence)
{
    if (!EncoderStorage_SelectLatestRecord(storage_records(),
                                           ENCODER_STORAGE_RECORD_COUNT,
                                           &s_latest_record,
                                           NULL))
    {
        return false;
    }

    return EncoderStorage_UnpackRecord(&s_latest_record, config, sequence);
}

bool EncoderStorage_Save(const encoder_persistent_config_t *config)
{
    uint32_t latest_index = 0U;
    uint32_t active_sector = 0U;
    uint32_t target_sector;
    uint32_t next_sequence = 1U;
    int32_t slot;
    bool found;
    bool saved;
#ifndef ENCODER_STORAGE_HOST_TEST
    uint32_t irq_mask;
#endif

    if (!config->calibration.valid)
    {
        return false;
    }

    found = EncoderStorage_SelectLatestRecord(storage_records(),
                                              ENCODER_STORAGE_RECORD_COUNT,
                                              &s_latest_record,
                                              &latest_index);
    if (found)
    {
        active_sector = latest_index / ENCODER_STORAGE_RECORDS_PER_SECTOR;
        next_sequence = s_latest_record.sequence + 1U;
    }

    EncoderStorage_PackRecord(&s_write_record, config, next_sequence);

#ifndef ENCODER_STORAGE_HOST_TEST
    if (FLASH_Init(&s_flash_config) != kStatus_FLASH_Success)
    {
        return false;
    }
    irq_mask = DisableGlobalIRQ();
#endif

    slot = find_erased_record(active_sector);
    if (slot >= 0)
    {
        saved = program_record((uint32_t)slot, &s_write_record);
    }
    else
    {
        target_sector = active_sector ^ 1U;
        saved = storage_erase(ENCODER_STORAGE_FLASH_BASE +
                              (target_sector * ENCODER_STORAGE_SECTOR_SIZE)) &&
                program_record(target_sector * ENCODER_STORAGE_RECORDS_PER_SECTOR,
                               &s_write_record);
        if (saved)
        {
            (void)storage_erase(ENCODER_STORAGE_FLASH_BASE +
                                (active_sector * ENCODER_STORAGE_SECTOR_SIZE));
        }
    }

#ifndef ENCODER_STORAGE_HOST_TEST
    EnableGlobalIRQ(irq_mask);
#endif
    return saved;
}

bool EncoderStorage_Erase(void)
{
    bool erased;
#ifndef ENCODER_STORAGE_HOST_TEST
    uint32_t irq_mask;

    if (FLASH_Init(&s_flash_config) != kStatus_FLASH_Success)
    {
        return false;
    }
    irq_mask = DisableGlobalIRQ();
#endif

    erased = storage_erase(ENCODER_STORAGE_FLASH_BASE) &&
             storage_erase(ENCODER_STORAGE_FLASH_BASE + ENCODER_STORAGE_SECTOR_SIZE);

#ifndef ENCODER_STORAGE_HOST_TEST
    EnableGlobalIRQ(irq_mask);
#endif
    return erased;
}
