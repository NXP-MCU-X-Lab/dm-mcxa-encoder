#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_encoder_storage.h"

static int s_failures;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            s_failures++;                                                       \
        }                                                                       \
    } while (0)

static encoder_persistent_config_t make_config(uint32_t marker)
{
    encoder_persistent_config_t config;
    uint32_t i;

    memset(&config, 0, sizeof(config));
    config.calibration.a1.center_sin = 1000.0f + (float)marker;
    config.calibration.a1.center_cos = 2000.0f;
    config.calibration.a1.t00 = 0.001f;
    config.calibration.a1.t10 = 0.0001f;
    config.calibration.a1.t11 = 0.0011f;
    config.calibration.a2.center_sin = 3000.0f;
    config.calibration.a2.center_cos = 4000.0f;
    config.calibration.a2.t00 = 0.0012f;
    config.calibration.a2.t10 = -0.0001f;
    config.calibration.a2.t11 = 0.0013f;
    config.calibration.phase_a1_zero_deg = (float)marker;
    config.calibration.phase_a2_zero_deg = (float)marker + 1.0f;
    config.calibration.valid = true;
    config.quality.sample_count = marker;
    config.quality.status = marker ^ 0x55AA55AAUL;
    config.quality.mag16 = 1.0f;
    config.quality.mag15 = 1.0f;
    for (i = 0U; i < ENCODER_STORAGE_EEPROM_SIZE; i++)
    {
        config.eeprom[i] = (uint8_t)(marker + i);
    }
    return config;
}

static void test_record_vectors(void)
{
    encoder_persistent_config_t input = make_config(0x12U);
    encoder_persistent_config_t output;
    encoder_storage_record_t records[4];
    encoder_storage_record_t latest;
    uint32_t sequence;
    uint32_t index;

    CHECK(sizeof(encoder_storage_record_t) == ENCODER_STORAGE_RECORD_SIZE);
    EncoderStorage_PackRecord(&records[0], &input, 0xFFFFFFFEUL);
    EncoderStorage_PackRecord(&records[1], &input, 0xFFFFFFFFUL);
    EncoderStorage_PackRecord(&records[2], &input, 0U);
    EncoderStorage_PackRecord(&records[3], &input, 1U);
    CHECK(EncoderStorage_SelectLatestRecord(records, 4U, &latest, &index));
    CHECK(index == 3U);
    CHECK(EncoderStorage_UnpackRecord(&latest, &output, &sequence));
    CHECK(sequence == 1U);
    CHECK(output.calibration.phase_a1_zero_deg == input.calibration.phase_a1_zero_deg);
    CHECK(memcmp(output.eeprom, input.eeprom, sizeof(input.eeprom)) == 0);

    records[3].eeprom[17] ^= 0x01U;
    CHECK(!EncoderStorage_ValidateRecord(&records[3]));

    EncoderStorage_PackRecord(&records[3], &input, 2U);
    records[3].version = ENCODER_STORAGE_VERSION - 1U;
    CHECK(!EncoderStorage_ValidateRecord(&records[3]));
}

static void test_persistence(void)
{
    encoder_persistent_config_t input = make_config(7U);
    encoder_persistent_config_t output;
    uint32_t sequence;

    EncoderStorage_HostReset();
    CHECK(!EncoderStorage_Load(&output, &sequence));
    CHECK(EncoderStorage_Save(&input));
    CHECK(EncoderStorage_Load(&output, &sequence));
    CHECK(sequence == 1U);
    CHECK(output.calibration.phase_a1_zero_deg == 7.0f);
    CHECK(memcmp(output.eeprom, input.eeprom, sizeof(input.eeprom)) == 0);
}

static void test_append_power_loss(void)
{
    int32_t failure_point;

    for (failure_point = 0; failure_point <= 8; failure_point++)
    {
        const encoder_persistent_config_t previous = make_config(1U);
        const encoder_persistent_config_t next = make_config(2U);
        encoder_persistent_config_t output;
        uint32_t sequence;
        bool saved;

        EncoderStorage_HostReset();
        CHECK(EncoderStorage_Save(&previous));
        EncoderStorage_HostFailAfter(failure_point);
        saved = EncoderStorage_Save(&next);
        EncoderStorage_HostFailAfter(-1);

        CHECK(EncoderStorage_Load(&output, &sequence));
        CHECK(sequence == (saved ? 2U : 1U));
        CHECK(output.calibration.phase_a1_zero_deg == (saved ? 2.0f : 1.0f));
    }
}

static void test_rollover_power_loss(void)
{
    int32_t failure_point;

    for (failure_point = 0; failure_point <= 10; failure_point++)
    {
        encoder_persistent_config_t output;
        encoder_persistent_config_t next;
        uint32_t sequence;
        uint32_t i;
        bool saved;

        EncoderStorage_HostReset();
        for (i = 1U; i <= ENCODER_STORAGE_RECORDS_PER_SECTOR; i++)
        {
            encoder_persistent_config_t current = make_config(i);
            CHECK(EncoderStorage_Save(&current));
        }

        next = make_config(100U + (uint32_t)failure_point);
        EncoderStorage_HostFailAfter(failure_point);
        saved = EncoderStorage_Save(&next);
        EncoderStorage_HostFailAfter(-1);

        CHECK(EncoderStorage_Load(&output, &sequence));
        CHECK((sequence == ENCODER_STORAGE_RECORDS_PER_SECTOR) ||
              (sequence == ENCODER_STORAGE_RECORDS_PER_SECTOR + 1U));
        if (saved)
        {
            CHECK(sequence == ENCODER_STORAGE_RECORDS_PER_SECTOR + 1U);
            CHECK(output.calibration.phase_a1_zero_deg ==
                  next.calibration.phase_a1_zero_deg);
        }
        else
        {
            CHECK(sequence == ENCODER_STORAGE_RECORDS_PER_SECTOR);
            CHECK(output.calibration.phase_a1_zero_deg ==
                  (float)ENCODER_STORAGE_RECORDS_PER_SECTOR);
        }
    }
}

int main(void)
{
    test_record_vectors();
    test_persistence();
    test_append_power_loss();
    test_rollover_power_loss();

    if (s_failures != 0)
    {
        fprintf(stderr, "%d storage checks failed\n", s_failures);
        return 1;
    }
    puts("storage checks passed");
    return 0;
}
