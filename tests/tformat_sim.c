/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "../source/app_tformat.c"

#define GAP       4000U
#define RESET_GAP 8000U

static int s_failures;

static void check(int condition, const char *what)
{
    if (!condition)
    {
        printf("  FAIL %s\n", what);
        s_failures++;
    }
}

static void check_crc(uint32_t length, const char *what)
{
    check((length > 0U) &&
              (tformat_crc(g_stub_tx, length - 1U) == g_stub_tx[length - 1U]),
          what);
}

static void publish_state(uint32_t counts,
                          float velocity_dps,
                          uint32_t status,
                          bool ready,
                          bool stationary)
{
    encoder_result_t result;

    memset(&result, 0, sizeof(result));
    result.angle_counts = counts;
    result.angular_velocity_dps = velocity_dps;
    result.status = status;
    TFormat_Publish(&result, ready, stationary);
}

static void publish(uint32_t counts, float velocity_dps, uint32_t status, bool ready)
{
    publish_state(counts, velocity_dps, status, ready, true);
}

static uint32_t request(const uint8_t *bytes, uint32_t count, uint32_t age_cycles)
{
    uint32_t i;

    g_stub_tx_len = 0U;
    DWT->CYCCNT += age_cycles;
    for (i = 0U; i < count; i++)
    {
        receive_byte(bytes[i]);
    }

    if (s_tx_busy)
    {
        tformat_tx_callback(LPUART2, &s_lpuart_edma_handle, kStatus_LPUART_TxIdle, NULL);
    }

    return g_stub_tx_len;
}

static uint32_t request_cf(uint8_t cf, uint32_t age_cycles)
{
    return request(&cf, 1U, age_cycles);
}

static void send_reset_sequence_state(uint8_t cf, uint32_t count, bool stationary)
{
    uint32_t i;

    for (i = 0U; i < count; i++)
    {
        publish_state(0x1000U, 0.0f, ENCODER_STATUS_OK, true, stationary);
        check(request_cf(cf, RESET_GAP) == 6U, "reset response length");
        check(g_stub_tx[0] == cf, "reset response CF");
        check_crc(6U, "reset response CRC");
    }
}

static void send_reset_sequence(uint8_t cf, uint32_t count)
{
    send_reset_sequence_state(cf, count, true);
}

static void clear_errors(void)
{
    publish(0x1000U, 0.0f, ENCODER_STATUS_OK, true);
    (void)TFormat_TakeResetRequests();
    send_reset_sequence(0xBAU, TFORMAT_RESET_COUNT);
    check((TFormat_TakeResetRequests() & TFORMAT_RESET_ERROR) != 0U,
          "ID7 reset sequence completes");
}

static uint32_t frame_position(const uint8_t *frame)
{
    return (uint32_t)frame[2] | ((uint32_t)frame[3] << 8U) |
           ((uint32_t)frame[4] << 16U);
}

static void test_standard_frames(void)
{
    uint32_t length;

    printf("standard frames\n");
    publish(0x1234U, 0.0f, ENCODER_STATUS_OK, true);

    length = request_cf(0x02U, GAP);
    check(length == 6U, "ID0 length");
    check(memcmp(g_stub_tx, "\x02\x00\x34\x12\x00\x24", 6U) == 0,
          "ID0 fixed vector");

    length = request_cf(0x8AU, GAP);
    check(length == 6U, "ID1 length");
    check(g_stub_tx[2] == 0U && g_stub_tx[3] == 0U && g_stub_tx[4] == 0U,
          "ID1 reports single-turn ABM=0");
    check(g_stub_tx[0] == 0x8AU, "ID1 response CF");
    check_crc(length, "ID1 CRC");

    length = request_cf(0x92U, GAP);
    check(length == 4U, "ID2 length");
    check(g_stub_tx[2] == 0x10U, "ID2 ENID");
    check(g_stub_tx[0] == 0x92U, "ID2 response CF");
    check_crc(length, "ID2 CRC");

    length = request_cf(0x1AU, GAP);
    check(length == 11U, "ID3 length");
    check(frame_position(g_stub_tx) == 0x1234U, "ID3 ABS");
    check(g_stub_tx[5] == 0x10U, "ID3 ENID");
    check(g_stub_tx[6] == 0U && g_stub_tx[7] == 0U && g_stub_tx[8] == 0U,
          "ID3 reports single-turn ABM=0");
    check(g_stub_tx[9] == 0U, "ID3 ALMC clear");
    check(g_stub_tx[0] == 0x1AU, "ID3 response CF");
    check_crc(length, "ID3 CRC");
}

static void test_request_delimiting(void)
{
    uint8_t read_request[3] = {0xEAU, 0x02U, 0U};
    uint32_t before;

    printf("request delimiting\n");
    read_request[2] = tformat_crc(read_request, 2U);

    g_stub_tx_len = 0U;
    DWT->CYCCNT += GAP;
    receive_byte(read_request[0]);
    receive_byte(read_request[1]);
    check(g_stub_tx_len == 0U, "CF inside request is data");
    receive_byte(read_request[2]);
    if (s_tx_busy)
    {
        tformat_tx_callback(LPUART2, &s_lpuart_edma_handle, kStatus_LPUART_TxIdle, NULL);
    }
    check(g_stub_tx_len == 4U, "complete EEPROM read answers");
    check(g_stub_tx[0] == 0xEAU, "ID D response CF");
    check_crc(g_stub_tx_len, "ID D response CRC");

    before = tformat_unsupported_count;
    check(request_cf(0x55U, GAP) == 0U, "unknown CF is silent");
    check(tformat_unsupported_count == before + 1U, "unknown CF counted");

    g_stub_tx_len = 0U;
    DWT->CYCCNT += GAP;
    receive_byte(0x55U);
    receive_byte(0x02U);
    check(g_stub_tx_len == 0U, "unknown frame stays discarded until gap");

    read_request[2] ^= 0xFFU;
    before = tformat_crc_error_count;
    check(request(read_request, 3U, GAP) == 0U, "bad CRC is silent");
    check(tformat_crc_error_count == before + 1U, "bad CRC counted");
}

static void test_eeprom(void)
{
    uint8_t select_page[4] = {0x32U, 0x7FU, 0x01U, 0U};
    uint8_t write_request[4] = {0x32U, 0x20U, 0xA5U, 0U};
    uint8_t second_write[4] = {0x32U, 0x21U, 0x5AU, 0U};
    uint8_t read_request[3] = {0xEAU, 0x20U, 0U};
    uint8_t image[TFORMAT_EEPROM_SIZE];
    uint32_t length;
    const uint32_t page1_base = TFORMAT_EEPROM_PAGE_SIZE;

    printf("EEPROM\n");
    select_page[3] = tformat_crc(select_page, 3U);
    write_request[3] = tformat_crc(write_request, 3U);
    second_write[3] = tformat_crc(second_write, 3U);
    read_request[2] = tformat_crc(read_request, 2U);

    length = request(select_page, 4U, GAP);
    check(length == 4U && g_stub_tx[1] == 0x7FU && g_stub_tx[2] == 0x01U,
          "ADF 0x7F selects Page1 without busy");
    check_crc(length, "page select response CRC");

    length = request(write_request, 4U, GAP);
    check(length == 4U, "ID6 response length");
    check(g_stub_tx[0] == 0x32U && g_stub_tx[1] == 0xA0U && g_stub_tx[2] == 0xA5U,
          "ID6 response has ADF busy and EDF");
    check_crc(length, "ID6 response CRC");
    check(TFormat_EepromWritePending(), "ID6 queues persistence");

    (void)request(second_write, 4U, GAP);
    TFormat_CopyEeprom(image);
    check(image[page1_base + 0x20U] == 0xA5U &&
              image[page1_base + 0x21U] == 0xFFU,
          "busy EEPROM rejects a second write");
    check((g_stub_tx[1] & 0x80U) != 0U && g_stub_tx[2] == 0U,
          "busy write reports EDF=0");

    length = request(read_request, 3U, GAP);
    check(length == 4U && (g_stub_tx[1] & 0x80U) != 0U && g_stub_tx[2] == 0U,
          "busy read reports EDF=0");
    check_crc(length, "busy ID D response CRC");

    TFormat_CompleteEepromWrite(true);
    length = request(read_request, 3U, GAP);
    check(length == 4U && g_stub_tx[1] == 0x20U && g_stub_tx[2] == 0xA5U,
          "ID D clears busy after persistence");

    write_request[2] = 0x33U;
    write_request[3] = tformat_crc(write_request, 3U);
    (void)request(write_request, 4U, GAP);
    TFormat_CompleteEepromWrite(false);
    (void)request(read_request, 3U, GAP);
    check(g_stub_tx[2] == 0xA5U, "failed persistence restores previous value");

    select_page[2] = 5U;
    select_page[3] = tformat_crc(select_page, 3U);
    (void)request(select_page, 4U, GAP);
    write_request[1] = 0x7EU;
    write_request[2] = 0x5AU;
    write_request[3] = tformat_crc(write_request, 3U);
    (void)request(write_request, 4U, GAP);
    TFormat_CompleteEepromWrite(true);
    read_request[1] = 0x7EU;
    read_request[2] = tformat_crc(read_request, 2U);
    (void)request(read_request, 3U, GAP);
    check(g_stub_tx[2] == 0x5AU, "Page5 address 0x7E is writable");
    TFormat_CopyEeprom(image);
    check(image[(5U * TFORMAT_EEPROM_PAGE_SIZE) + 0x7EU] == 0x5AU,
          "Page5 upper boundary maps into persistent image");

    select_page[2] = 6U;
    select_page[3] = tformat_crc(select_page, 3U);
    (void)request(select_page, 4U, GAP);
    (void)request(read_request, 3U, GAP);
    check(g_stub_tx[2] == 0xFFU, "Page6 reads as 0xFF");
    (void)request(write_request, 4U, GAP);
    check(!TFormat_EepromWritePending() && g_stub_tx[2] == 0xFFU,
          "Page6 ignores writes");

    select_page[2] = 7U;
    select_page[3] = tformat_crc(select_page, 3U);
    (void)request(select_page, 4U, GAP);
    (void)request(read_request, 3U, GAP);
    check(g_stub_tx[2] == 0xFFU, "Page7 reads as 0xFF");

    select_page[2] = 0U;
    select_page[3] = tformat_crc(select_page, 3U);
    (void)request(select_page, 4U, GAP);
}

static void test_status(void)
{
    uint32_t length;

    printf("status\n");
    clear_errors();
    publish(0x1000U, 0.0f, ENCODER_STATUS_OK, false);
    length = request_cf(0x1AU, GAP);
    check(length == 11U, "invalid ID3 still answers");
    check(g_stub_tx[1] == 0x10U, "counting error maps to SF ea0");
    check((g_stub_tx[9] & 0x04U) != 0U, "counting error maps to ALMC CE");

    clear_errors();
    publish(0x1000U, TFORMAT_OVERSPEED_DPS * 1.1f, ENCODER_STATUS_OK, true);
    (void)request_cf(0x1AU, GAP);
    check(g_stub_tx[1] == 0U, "overspeed alone does not set SF encoder error");
    check((g_stub_tx[9] & 0x01U) != 0U, "overspeed maps to ALMC OS");

    publish(0x1000U, 0.0f, ENCODER_STATUS_HOLD_LAST, true);
    (void)request_cf(0x1AU, GAP);
    check(g_stub_tx[1] == 0x10U && (g_stub_tx[9] & 0x04U) != 0U,
          "held sample raises counting error");

    publish(0x1000U, 36000.0f, ENCODER_STATUS_HOLD_LAST, true);
    (void)request_cf(0x02U, 18000U);
    check(frame_position(g_stub_tx) == 0x1000U,
          "held sample does not extrapolate old velocity");

    clear_errors();
    check(request_cf(0x82U, GAP) == 0U, "CF parity error is silent");
    (void)request_cf(0x1AU, GAP);
    check((g_stub_tx[1] & TFORMAT_SF_PARITY_ERROR) != 0U,
          "CF parity error maps to SF ca0");

    clear_errors();
    g_stub_uart_status = kLPUART_FramingErrorFlag;
    LPUART2_IRQHandler();
    (void)request_cf(0x1AU, GAP);
    check((g_stub_tx[1] & TFORMAT_SF_DELIMITER_ERROR) != 0U,
          "UART framing error maps to SF ca1");
}

static void test_staleness(void)
{
    uint32_t before;

    printf("staleness\n");
    clear_errors();
    publish(0x2000U, 0.0f, ENCODER_STATUS_OK, true);
    (void)request_cf(0x1AU, GAP);
    check(g_stub_tx[9] == 0U, "fresh sample is valid");

    before = tformat_stale_count;
    (void)request_cf(0x1AU, s_stale_cycles + 1000U);
    check((g_stub_tx[9] & 0x04U) != 0U, "stale sample raises counting error");
    check(tformat_stale_count > before, "stale request counted");

    DWT->CYCCNT = s_snapshot[s_active_snapshot].cycle + GAP;
    s_rx_last_cycle = DWT->CYCCNT - (s_gap_cycles + 1U);
    (void)request_cf(0x1AU, 1U);
    check((g_stub_tx[9] & 0x04U) != 0U, "stale latch survives cycle wrap alias");

    publish(0x2000U, 0.0f, ENCODER_STATUS_OK, true);
    (void)request_cf(0x1AU, GAP);
    check((g_stub_tx[9] & TFORMAT_ALMC_COUNT_ERROR) != 0U,
          "fresh publish does not clear latched stale alarm");
    clear_errors();
    (void)request_cf(0x1AU, GAP);
    check(g_stub_tx[9] == 0U, "ID7 clears stale latch when input is valid");
}

static void test_extrapolation(void)
{
    uint32_t position;

    printf("fixed-point extrapolation\n");
    clear_errors();
    publish(1000U, 36000.0f, ENCODER_STATUS_OK, true);
    (void)request_cf(0x02U, 18000U);
    position = frame_position(g_stub_tx);
    check(position > 1550U && position < 1760U, "100 us extrapolation is about 655 counts");

    publish(ENCODER_COUNTS_PER_REV - 100U, 36000.0f, ENCODER_STATUS_OK, true);
    (void)request_cf(0x1AU, 18000U);
    check(frame_position(g_stub_tx) < 1000U, "positive extrapolation wraps single-turn ABS");
    check(g_stub_tx[6] == 0U && g_stub_tx[7] == 0U && g_stub_tx[8] == 0U,
          "extrapolation never invents multi-turn data");

    publish(50U, -36000.0f, ENCODER_STATUS_OK, true);
    (void)request_cf(0x02U, 18000U);
    check(frame_position(g_stub_tx) > ENCODER_COUNTS_PER_REV - 1000U,
          "negative extrapolation wraps single-turn ABS");
}

static void test_resets(void)
{
    uint32_t requests;
    uint32_t i;

    printf("reset semantics\n");
    publish(0x1234U, 0.0f, ENCODER_STATUS_OK, true);
    (void)TFormat_TakeResetRequests();

    send_reset_sequence(0xBAU, TFORMAT_RESET_COUNT - 1U);
    check(TFormat_TakeResetRequests() == 0U, "nine ID7 frames do not reset");
    send_reset_sequence(0xBAU, 1U);
    check((TFormat_TakeResetRequests() & TFORMAT_RESET_ERROR) != 0U,
          "tenth ID7 clears errors once");

    send_reset_sequence_state(0xC2U, TFORMAT_RESET_COUNT, false);
    check(TFormat_TakeResetRequests() == 0U,
          "moving ID8 sequence does not request a delayed reset");
    check((g_stub_tx[1] & TFORMAT_SF_COUNTING_ERROR) != 0U,
          "moving ID8 sequence reports Counting Error");
    publish(0x1000U, 0.0f, ENCODER_STATUS_OK, true);
    check(TFormat_TakeResetRequests() == 0U,
          "stationary state does not revive rejected ID8");
    clear_errors();

    send_reset_sequence(0xC2U, 4U);
    (void)request_cf(0xC2U, GAP);
    send_reset_sequence(0xC2U, TFORMAT_RESET_COUNT - 1U);
    check(TFormat_TakeResetRequests() == 0U,
          "short reset interval clears the sequence");
    send_reset_sequence(0xC2U, 1U);
    check((TFormat_TakeResetRequests() & TFORMAT_RESET_POSITION) != 0U,
          "tenth valid ID8 requests one position reset");

    send_reset_sequence(0x62U, 4U);
    (void)request_cf(0x02U, RESET_GAP);
    for (i = 0U; i < TFORMAT_RESET_COUNT - 1U; i++)
    {
        (void)request_cf(0x62U, RESET_GAP);
    }
    check(TFormat_TakeResetRequests() == 0U,
          "non-reset command clears the reset sequence");
    (void)request_cf(0x62U, RESET_GAP);

    requests = TFormat_TakeResetRequests();
    check((requests & TFORMAT_RESET_MULTITURN) != 0U,
          "tenth valid ID C acknowledges multi-turn reset");
    check(TFormat_TakeResetRequests() == 0U, "reset latch clears atomically");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    TFormat_Init();
    DWT->CYCCNT = 1000000U;

    test_standard_frames();
    test_request_delimiting();
    test_eeprom();
    test_status();
    test_staleness();
    test_extrapolation();
    test_resets();

    if (s_failures != 0)
    {
        printf("%d check(s) failed\n", s_failures);
        return 1;
    }

    printf("all checks passed\n");
    return 0;
}
