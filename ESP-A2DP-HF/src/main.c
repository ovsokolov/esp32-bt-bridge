#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/rmt_tx.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_check.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/gpio_sig_map.h"

static const char *TAG = "ESP_A2DP_HF";

#ifndef BT_DEVICE_NAME
#define BT_DEVICE_NAME "ESP-A2DP-HF"
#endif

#ifndef BT_REMOTE_NAME
#define BT_REMOTE_NAME ""
#endif

#ifndef RGB_LED_GPIO
#define RGB_LED_GPIO 0
#endif

#ifndef RGB_LED_POWER_GPIO
#define RGB_LED_POWER_GPIO 2
#endif

#ifndef SAMPLE_RATE
#define SAMPLE_RATE 44100
#endif

#ifndef I2S_BCLK_GPIO
#define I2S_BCLK_GPIO 5
#endif

#ifndef I2S_WS_GPIO
#define I2S_WS_GPIO 25
#endif

#ifndef I2S_DOUT_GPIO
#define I2S_DOUT_GPIO 19
#endif

#ifndef SCO_PCM_CLK_GPIO
#define SCO_PCM_CLK_GPIO I2S_BCLK_GPIO
#endif

#ifndef SCO_PCM_FSYNC_GPIO
#define SCO_PCM_FSYNC_GPIO I2S_WS_GPIO
#endif

#ifndef SCO_PCM_DOUT_GPIO
#define SCO_PCM_DOUT_GPIO I2S_DOUT_GPIO
#endif

#ifndef SCO_PCM_DIN_GPIO
#define SCO_PCM_DIN_GPIO 21
#endif

#define UART_PORT UART_NUM_0
#define UART_RX_BUF_SIZE 512
#define UART_LINE_SIZE 192
#define RGB_RMT_RESOLUTION_HZ 10000000U
#define NVS_NS "bt_hf"
#define NVS_LAST_BDA "last_bda"
#define AUTO_RECONNECT_INITIAL_MS 3000
#define AUTO_RECONNECT_PERIOD_MS 5000
#define AVRCP_SNAPSHOT_MIN_INTERVAL_MS 1500
#define AVRCP_TRACK_CHANGE_SNAPSHOT_DELAY_MS 700
#define PCM_RINGBUF_SIZE (24 * 1024)
#define HF_BUILD_TAG "hf-hfp-forward-ring-20260526"
#define A2DP_I2S_GATE_DURING_SCO 0

static esp_bd_addr_t remote_bda;
static esp_bd_addr_t saved_bda;
static bool have_remote_bda;
static bool have_saved_bda;
static bool a2dp_connected;
static bool a2dp_connecting;
static bool avrcp_connected;
static bool hf_slc_connected;
static bool hf_connecting;
static bool hf_audio_connected;
static bool a2dp_ready;
static bool hf_ready;
static volatile bool a2dp_audio_started;
static volatile bool hf_clcc_task_running;
static volatile bool hf_clcc_batch_pending;
static esp_avrc_rn_evt_cap_mask_t avrcp_peer_rn_caps;
static bool avrcp_have_rn_caps;
static uint8_t avrcp_tl;
static TickType_t avrcp_snapshot_not_before;
static volatile bool avrcp_delayed_snapshot_running;
static uint32_t avrcp_last_song_length;
static uint32_t avrcp_last_song_position;
static uint8_t avrcp_last_play_status;
static bool avrcp_have_last_status;
static uint8_t avrcp_last_track_id[8];
static bool avrcp_have_last_track_id;
static char avrcp_last_metadata[8][128];
static uint16_t avrcp_last_metadata_len[8];
static bool avrcp_have_last_metadata[8];
static char a2dp_rate_status[192] = "A2DP_AUDIO_RATE:codec=none sample_rate=0 channels=unknown";
static char hfp_rate_status[96] = "HFP_HF_AUDIO_RATE:codec=none sample_rate=0 channels=mono frame=0";
static uint32_t a2dp_rx_bytes;
static uint32_t i2s_tx_bytes;
static uint32_t i2s_tx_short_count;
static uint32_t i2s_tx_report_next = 32768;
static uint32_t pcm_ring_drop_bytes;
static uint32_t pcm_ring_drop_count;
static uint16_t i2s_tx_peak;
static i2s_chan_handle_t i2s_tx;
static RingbufHandle_t pcm_ringbuf;
static bool sco_pcm_gpio_active;
static bool a2dp_i2s_enabled;
static rmt_channel_handle_t rgb_chan;
static rmt_encoder_handle_t rgb_encoder;
static uint8_t rgb_pixel[3];

static void a2dp_i2s_disable_for_sco(void)
{
    if (!a2dp_i2s_enabled || i2s_tx == NULL) {
        return;
    }
    esp_err_t err = i2s_channel_disable(i2s_tx);
    if (err == ESP_OK) {
        a2dp_i2s_enabled = false;
        ESP_LOGI(TAG, "A2DP_I2S:disabled_for_sco");
    } else {
        ESP_LOGW(TAG, "A2DP_I2S_DISABLE_FAILED:0x%x", err);
    }
}

static void a2dp_i2s_enable_after_sco(void)
{
    if (a2dp_i2s_enabled || i2s_tx == NULL) {
        return;
    }
    esp_err_t err = i2s_channel_enable(i2s_tx);
    if (err == ESP_OK) {
        a2dp_i2s_enabled = true;
        ESP_LOGI(TAG, "A2DP_I2S:enabled_after_sco");
    } else {
        ESP_LOGW(TAG, "A2DP_I2S_ENABLE_FAILED:0x%x", err);
    }
}

static void sco_pcm_gpio_config(void)
{
#if A2DP_I2S_GATE_DURING_SCO
    a2dp_i2s_disable_for_sco();
#else
    ESP_LOGI(TAG, "A2DP_I2S:kept_enabled_for_sco");
#endif

    const uint64_t output_pins =
        (1ULL << SCO_PCM_DOUT_GPIO) |
#ifndef CONFIG_BTDM_CTRL_PCM_ROLE_SLAVE
        (1ULL << SCO_PCM_CLK_GPIO) |
        (1ULL << SCO_PCM_FSYNC_GPIO) |
#endif
        0;
    const uint64_t input_pins =
        (1ULL << SCO_PCM_DIN_GPIO) |
#ifdef CONFIG_BTDM_CTRL_PCM_ROLE_SLAVE
        (1ULL << SCO_PCM_CLK_GPIO) |
        (1ULL << SCO_PCM_FSYNC_GPIO) |
#endif
        0;

    gpio_config_t in_conf = {
        .pin_bit_mask = input_pins,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_conf));

    if (output_pins != 0) {
        gpio_config_t out_conf = {
            .pin_bit_mask = output_pins,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&out_conf));
    }

#ifdef CONFIG_BTDM_CTRL_PCM_ROLE_SLAVE
    esp_rom_gpio_connect_in_signal(SCO_PCM_FSYNC_GPIO, PCMFSYNC_IN_IDX, false);
    esp_rom_gpio_connect_in_signal(SCO_PCM_CLK_GPIO, PCMCLK_IN_IDX, false);
#else
    esp_rom_gpio_connect_out_signal(SCO_PCM_FSYNC_GPIO, PCMFSYNC_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(SCO_PCM_CLK_GPIO, PCMCLK_OUT_IDX, false, false);
#endif
    esp_rom_gpio_connect_out_signal(SCO_PCM_DOUT_GPIO, PCMDOUT_IDX, false, false);
    esp_rom_gpio_connect_in_signal(SCO_PCM_DIN_GPIO, PCMDIN_IDX, false);
    sco_pcm_gpio_active = true;
    ESP_LOGI(TAG, "HFP_SCO_PCM_GPIO:active clk=%d fsync=%d dout=%d din=%d",
             SCO_PCM_CLK_GPIO, SCO_PCM_FSYNC_GPIO, SCO_PCM_DOUT_GPIO, SCO_PCM_DIN_GPIO);
}

static void a2dp_i2s_gpio_config(void)
{
    const uint64_t output_pins =
        (1ULL << I2S_BCLK_GPIO) |
        (1ULL << I2S_WS_GPIO) |
        (1ULL << I2S_DOUT_GPIO);

    gpio_config_t out_conf = {
        .pin_bit_mask = output_pins,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_conf));

    esp_rom_gpio_connect_out_signal(I2S_BCLK_GPIO, I2S0O_BCK_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(I2S_WS_GPIO, I2S0O_WS_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(I2S_DOUT_GPIO, I2S0O_DATA_OUT23_IDX, false, false);
    sco_pcm_gpio_active = false;
    ESP_LOGI(TAG, "A2DP_I2S_GPIO:active bclk=%d ws=%d dout=%d",
             I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DOUT_GPIO);
#if A2DP_I2S_GATE_DURING_SCO
    a2dp_i2s_enable_after_sco();
#endif
}

static uint32_t a2dp_sbc_sample_rate(uint8_t samp_freq)
{
    if (samp_freq & ESP_A2D_SBC_CIE_SF_48K) {
        return 48000;
    }
    if (samp_freq & ESP_A2D_SBC_CIE_SF_44K) {
        return 44100;
    }
    if (samp_freq & ESP_A2D_SBC_CIE_SF_32K) {
        return 32000;
    }
    if (samp_freq & ESP_A2D_SBC_CIE_SF_16K) {
        return 16000;
    }
    return 0;
}

static const char *a2dp_sbc_channel_mode(uint8_t ch_mode)
{
    if (ch_mode & ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO) {
        return "joint_stereo";
    }
    if (ch_mode & ESP_A2D_SBC_CIE_CH_MODE_STEREO) {
        return "stereo";
    }
    if (ch_mode & ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL) {
        return "dual";
    }
    if (ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO) {
        return "mono";
    }
    return "unknown";
}

static const char *a2dp_sbc_alloc_method(uint8_t alloc_mthd)
{
    if (alloc_mthd & ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS) {
        return "loudness";
    }
    if (alloc_mthd & ESP_A2D_SBC_CIE_ALLOC_MTHD_SNR) {
        return "snr";
    }
    return "unknown";
}

static uint8_t a2dp_sbc_block_len(uint8_t block_len)
{
    if (block_len & ESP_A2D_SBC_CIE_BLOCK_LEN_16) {
        return 16;
    }
    if (block_len & ESP_A2D_SBC_CIE_BLOCK_LEN_12) {
        return 12;
    }
    if (block_len & ESP_A2D_SBC_CIE_BLOCK_LEN_8) {
        return 8;
    }
    if (block_len & ESP_A2D_SBC_CIE_BLOCK_LEN_4) {
        return 4;
    }
    return 0;
}

static uint8_t a2dp_sbc_subbands(uint8_t num_subbands)
{
    if (num_subbands & ESP_A2D_SBC_CIE_NUM_SUBBANDS_8) {
        return 8;
    }
    if (num_subbands & ESP_A2D_SBC_CIE_NUM_SUBBANDS_4) {
        return 4;
    }
    return 0;
}

static const rmt_symbol_word_t ws2812_zero = {
    .level0 = 1,
    .duration0 = 0.3 * RGB_RMT_RESOLUTION_HZ / 1000000U,
    .level1 = 0,
    .duration1 = 0.9 * RGB_RMT_RESOLUTION_HZ / 1000000U,
};

static const rmt_symbol_word_t ws2812_one = {
    .level0 = 1,
    .duration0 = 0.9 * RGB_RMT_RESOLUTION_HZ / 1000000U,
    .level1 = 0,
    .duration1 = 0.3 * RGB_RMT_RESOLUTION_HZ / 1000000U,
};

static const rmt_symbol_word_t ws2812_reset = {
    .level0 = 0,
    .duration0 = RGB_RMT_RESOLUTION_HZ / 1000000U * 50U / 2U,
    .level1 = 0,
    .duration1 = RGB_RMT_RESOLUTION_HZ / 1000000U * 50U / 2U,
};

static size_t ws2812_encoder_callback(const void *data, size_t data_size,
                                      size_t symbols_written, size_t symbols_free,
                                      rmt_symbol_word_t *symbols, bool *done,
                                      void *arg)
{
    (void)arg;

    if (symbols_free < 8) {
        return 0;
    }

    const size_t data_pos = symbols_written / 8;
    const uint8_t *data_bytes = (const uint8_t *)data;
    if (data_pos < data_size) {
        size_t symbol_pos = 0;
        for (uint8_t bitmask = 0x80; bitmask != 0; bitmask >>= 1) {
            symbols[symbol_pos++] = (data_bytes[data_pos] & bitmask) ? ws2812_one : ws2812_zero;
        }
        return symbol_pos;
    }

    symbols[0] = ws2812_reset;
    *done = true;
    return 1;
}

static esp_err_t rgb_led_init(void)
{
    gpio_config_t power_cfg = {
        .pin_bit_mask = BIT64(RGB_LED_POWER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&power_cfg), TAG, "rgb power gpio");
    gpio_set_level(RGB_LED_POWER_GPIO, 1);

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = RGB_LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = RGB_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &rgb_chan), TAG, "rmt channel");

    rmt_simple_encoder_config_t encoder_config = {
        .callback = ws2812_encoder_callback,
    };
    ESP_RETURN_ON_ERROR(rmt_new_simple_encoder(&encoder_config, &rgb_encoder), TAG, "rmt encoder");
    ESP_RETURN_ON_ERROR(rmt_enable(rgb_chan), TAG, "rmt enable");
    ESP_LOGI(TAG, "RGB_LED_READY:data_gpio=%d power_gpio=%d", RGB_LED_GPIO, RGB_LED_POWER_GPIO);
    return ESP_OK;
}

static void rgb_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!rgb_chan || !rgb_encoder) {
        return;
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    rgb_pixel[0] = green;
    rgb_pixel[1] = red;
    rgb_pixel[2] = blue;

    if (rmt_transmit(rgb_chan, rgb_encoder, rgb_pixel, sizeof(rgb_pixel), &tx_config) == ESP_OK) {
        rmt_tx_wait_all_done(rgb_chan, portMAX_DELAY);
    }
}

static void led_task(void *arg)
{
    (void)arg;

    while (true) {
        if (a2dp_connected || hf_slc_connected) {
            rgb_led_set(0, 0, 32);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            rgb_led_set(0, 32, 0);
            vTaskDelay(pdMS_TO_TICKS(250));
            rgb_led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(750));
        }
    }
}

static const char *bda_to_str(const esp_bd_addr_t bda, char *out, size_t out_len)
{
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return out;
}

static bool parse_bda(const char *text, esp_bd_addr_t out)
{
    unsigned int b[6];
    if (sscanf(text, "%02x:%02x:%02x:%02x:%02x:%02x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        if (b[i] > 0xff) {
            return false;
        }
        out[i] = (uint8_t)b[i];
    }
    return true;
}

static char *trim(char *line)
{
    while (isspace((unsigned char)*line)) {
        line++;
    }
    char *end = line + strlen(line);
    while (end > line && isspace((unsigned char)end[-1])) {
        *--end = '\0';
    }
    return line;
}

static uint8_t next_tl(void)
{
    uint8_t tl = avrcp_tl++ & ESP_AVRC_TRANS_LABEL_MAX;
    return tl;
}

static const char *avrcp_event_name(uint8_t event_id)
{
    switch (event_id) {
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
        return "PLAY_STATUS_CHANGE";
    case ESP_AVRC_RN_TRACK_CHANGE:
        return "TRACK_CHANGE";
    case ESP_AVRC_RN_TRACK_REACHED_END:
        return "TRACK_REACHED_END";
    case ESP_AVRC_RN_TRACK_REACHED_START:
        return "TRACK_REACHED_START";
    case ESP_AVRC_RN_PLAY_POS_CHANGED:
        return "PLAY_POS_CHANGED";
    case ESP_AVRC_RN_BATTERY_STATUS_CHANGE:
        return "BATTERY_STATUS_CHANGE";
    case ESP_AVRC_RN_SYSTEM_STATUS_CHANGE:
        return "SYSTEM_STATUS_CHANGE";
    case ESP_AVRC_RN_APP_SETTING_CHANGE:
        return "APP_SETTING_CHANGE";
    case ESP_AVRC_RN_NOW_PLAYING_CHANGE:
        return "NOW_PLAYING_CHANGE";
    case ESP_AVRC_RN_AVAILABLE_PLAYERS_CHANGE:
        return "AVAILABLE_PLAYERS_CHANGE";
    case ESP_AVRC_RN_ADDRESSED_PLAYER_CHANGE:
        return "ADDRESSED_PLAYER_CHANGE";
    case ESP_AVRC_RN_UIDS_CHANGE:
        return "UIDS_CHANGE";
    case ESP_AVRC_RN_VOLUME_CHANGE:
        return "VOLUME_CHANGE";
    default:
        return "UNKNOWN";
    }
}

static uint32_t avrcp_event_parameter(uint8_t event_id)
{
    return event_id == ESP_AVRC_RN_PLAY_POS_CHANGED ? 1 : 0;
}

static uint8_t avrcp_metadata_mask(void)
{
    return ESP_AVRC_MD_ATTR_TITLE |
           ESP_AVRC_MD_ATTR_ARTIST |
           ESP_AVRC_MD_ATTR_ALBUM |
           ESP_AVRC_MD_ATTR_TRACK_NUM |
           ESP_AVRC_MD_ATTR_NUM_TRACKS |
           ESP_AVRC_MD_ATTR_GENRE |
           ESP_AVRC_MD_ATTR_PLAYING_TIME;
}

static int avrcp_metadata_slot(uint8_t attr_id)
{
    switch (attr_id) {
    case ESP_AVRC_MD_ATTR_TITLE:
        return 0;
    case ESP_AVRC_MD_ATTR_ARTIST:
        return 1;
    case ESP_AVRC_MD_ATTR_ALBUM:
        return 2;
    case ESP_AVRC_MD_ATTR_TRACK_NUM:
        return 3;
    case ESP_AVRC_MD_ATTR_NUM_TRACKS:
        return 4;
    case ESP_AVRC_MD_ATTR_GENRE:
        return 5;
    case ESP_AVRC_MD_ATTR_PLAYING_TIME:
        return 6;
    default:
        return -1;
    }
}

static bool avrcp_metadata_is_duplicate(uint8_t attr_id, const uint8_t *text, uint16_t len)
{
    int slot = avrcp_metadata_slot(attr_id);
    if (slot < 0) {
        return false;
    }

    uint16_t copy_len = len;
    if (copy_len >= sizeof(avrcp_last_metadata[slot])) {
        copy_len = sizeof(avrcp_last_metadata[slot]) - 1;
    }

    if (avrcp_have_last_metadata[slot] &&
        avrcp_last_metadata_len[slot] == copy_len &&
        memcmp(avrcp_last_metadata[slot], text, copy_len) == 0) {
        return true;
    }

    memcpy(avrcp_last_metadata[slot], text, copy_len);
    avrcp_last_metadata[slot][copy_len] = '\0';
    avrcp_last_metadata_len[slot] = copy_len;
    avrcp_have_last_metadata[slot] = true;
    return false;
}

static void avrcp_clear_snapshot_cache(const char *reason)
{
    avrcp_have_last_status = false;
    avrcp_have_last_track_id = false;
    memset(avrcp_have_last_metadata, 0, sizeof(avrcp_have_last_metadata));
    memset(avrcp_last_metadata_len, 0, sizeof(avrcp_last_metadata_len));
    ESP_LOGI(TAG, "AVRCP_MONITOR_CACHE_CLEAR:%s", reason ? reason : "unknown");
}

static void avrcp_request_snapshot_internal(const char *reason, bool force)
{
    if (!avrcp_connected) {
        ESP_LOGI(TAG, "AVRCP_MONITOR_SNAPSHOT_SKIPPED:%s not_connected", reason);
        return;
    }

    TickType_t now = xTaskGetTickCount();
    if (!force && avrcp_snapshot_not_before != 0 &&
        (int32_t)(now - avrcp_snapshot_not_before) < 0) {
        ESP_LOGI(TAG, "AVRCP_MONITOR_SNAPSHOT_SKIPPED:%s throttled", reason);
        return;
    }
    avrcp_snapshot_not_before = now + pdMS_TO_TICKS(AVRCP_SNAPSHOT_MIN_INTERVAL_MS);

    esp_err_t meta_err = esp_avrc_ct_send_metadata_cmd(next_tl(), avrcp_metadata_mask());
    esp_err_t status_err = esp_avrc_ct_send_get_play_status_cmd(next_tl());
    ESP_LOGI(TAG, "AVRCP_MONITOR_SNAPSHOT:%s metadata=0x%x status=0x%x",
             reason, meta_err, status_err);
}

static void avrcp_request_snapshot(const char *reason)
{
    avrcp_request_snapshot_internal(reason, false);
}

static void avrcp_force_snapshot(const char *reason)
{
    avrcp_clear_snapshot_cache(reason);
    avrcp_request_snapshot_internal(reason, true);
}

static void avrcp_delayed_snapshot_task(void *arg)
{
    const char *reason = (const char *)arg;
    vTaskDelay(pdMS_TO_TICKS(AVRCP_TRACK_CHANGE_SNAPSHOT_DELAY_MS));
    avrcp_force_snapshot(reason);
    avrcp_delayed_snapshot_running = false;
    vTaskDelete(NULL);
}

static void avrcp_schedule_delayed_snapshot(const char *reason)
{
    if (avrcp_delayed_snapshot_running) {
        ESP_LOGI(TAG, "AVRCP_MONITOR_SNAPSHOT_DELAY_SKIPPED:%s pending", reason);
        return;
    }

    avrcp_delayed_snapshot_running = true;
    BaseType_t ok = xTaskCreate(avrcp_delayed_snapshot_task, "avrcp_snap", 3072,
                                (void *)reason, 5, NULL);
    if (ok != pdPASS) {
        avrcp_delayed_snapshot_running = false;
        ESP_LOGW(TAG, "AVRCP_MONITOR_SNAPSHOT_DELAY_FAILED:%s", reason);
    } else {
        ESP_LOGI(TAG, "AVRCP_MONITOR_SNAPSHOT_DELAY:%s %dms",
                 reason, AVRCP_TRACK_CHANGE_SNAPSHOT_DELAY_MS);
    }
}

static void avrcp_register_event(uint8_t event_id, const char *reason)
{
    if (!avrcp_connected) {
        return;
    }

    esp_err_t err = esp_avrc_ct_send_register_notification_cmd(
        next_tl(), event_id, avrcp_event_parameter(event_id));
    ESP_LOGI(TAG, "AVRCP_MONITOR_REGISTER:%s event=0x%02x(%s) err=0x%x",
             reason, event_id, avrcp_event_name(event_id), err);
}

static void avrcp_register_supported_events(const char *reason)
{
    if (!avrcp_have_rn_caps) {
        ESP_LOGI(TAG, "AVRCP_MONITOR_REGISTER_SKIPPED:%s no_caps", reason);
        return;
    }

    for (uint8_t event_id = ESP_AVRC_RN_PLAY_STATUS_CHANGE;
         event_id < ESP_AVRC_RN_MAX_EVT; event_id++) {
        if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST,
                                               &avrcp_peer_rn_caps,
                                               event_id)) {
            avrcp_register_event(event_id, reason);
        }
    }
}

static bool ensure_remote(void)
{
    if (!have_remote_bda) {
        ESP_LOGW(TAG, "REMOTE_MISSING: use CONNECT:aa:bb:cc:dd:ee:ff first");
        return false;
    }
    return true;
}

static void send_avrcp_passthrough(uint8_t key_code)
{
    if (!avrcp_connected) {
        ESP_LOGW(TAG, "AVRCP_NOT_CONNECTED:key=0x%02x", key_code);
        return;
    }
    uint8_t tl = next_tl();
    esp_err_t err = esp_avrc_ct_send_passthrough_cmd(tl, key_code, ESP_AVRC_PT_CMD_STATE_PRESSED);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AVRCP_PRESS_FAILED:key=0x%02x err=0x%x", key_code, err);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(60));
    err = esp_avrc_ct_send_passthrough_cmd(tl, key_code, ESP_AVRC_PT_CMD_STATE_RELEASED);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AVRCP_RELEASE_FAILED:key=0x%02x err=0x%x", key_code, err);
    } else {
        ESP_LOGI(TAG, "AVRCP_TX:key=0x%02x", key_code);
    }
}

static bool parse_chld(const char *value, esp_hf_chld_type_t *type, int *index)
{
    *index = 0;
    if (strcmp(value, "0") == 0) {
        *type = ESP_HF_CHLD_TYPE_REL;
    } else if (strcmp(value, "1") == 0) {
        *type = ESP_HF_CHLD_TYPE_REL_ACC;
    } else if (strcmp(value, "2") == 0) {
        *type = ESP_HF_CHLD_TYPE_HOLD_ACC;
    } else if (strcmp(value, "3") == 0) {
        *type = ESP_HF_CHLD_TYPE_MERGE;
    } else if (strcmp(value, "4") == 0) {
        *type = ESP_HF_CHLD_TYPE_MERGE_DETACH;
    } else if (value[0] == '1' && isdigit((unsigned char)value[1])) {
        *type = ESP_HF_CHLD_TYPE_REL_X;
        *index = atoi(value + 1);
    } else if (value[0] == '2' && isdigit((unsigned char)value[1])) {
        *type = ESP_HF_CHLD_TYPE_PRIV_X;
        *index = atoi(value + 1);
    } else {
        return false;
    }
    return true;
}

static void log_status(void)
{
    char bda[18] = "none";
    char saved[18] = "none";
    if (have_remote_bda) {
        bda_to_str(remote_bda, bda, sizeof(bda));
    }
    if (have_saved_bda) {
        bda_to_str(saved_bda, saved, sizeof(saved));
    }
    ESP_LOGI(TAG, "STATUS:remote=%s saved=%s a2dp=%d/%d avrcp=%d hf=%d/%d hf_audio=%d pcm=%" PRIu32 " i2s_tx=%" PRIu32 " short=%" PRIu32 " peak=%u ring_drop=%" PRIu32 "/%" PRIu32,
             bda, saved, a2dp_connected, a2dp_connecting, avrcp_connected,
             hf_slc_connected, hf_connecting, hf_audio_connected,
             a2dp_rx_bytes, i2s_tx_bytes, i2s_tx_short_count, i2s_tx_peak,
             pcm_ring_drop_count, pcm_ring_drop_bytes);
    ESP_LOGI(TAG, "%s", a2dp_rate_status);
    ESP_LOGI(TAG, "%s", hfp_rate_status);
}

static void log_board_id(void)
{
    ESP_LOGI(TAG, "BOARD_ID:role=HF name=%s build=%s profiles=A2DP_SINK,HFP_HF,AVRCP_CT",
             BT_DEVICE_NAME, HF_BUILD_TAG);
}

static void save_last_bda(const esp_bd_addr_t bda)
{
    if (have_saved_bda && memcmp(saved_bda, bda, sizeof(saved_bda)) == 0) {
        return;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BT_LAST_BDA_SAVE_OPEN_FAILED:0x%x", err);
        return;
    }

    err = nvs_set_blob(nvs, NVS_LAST_BDA, bda, ESP_BD_ADDR_LEN);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);

    char text[18];
    if (err == ESP_OK) {
        memcpy(saved_bda, bda, sizeof(saved_bda));
        have_saved_bda = true;
        ESP_LOGI(TAG, "BT_LAST_BDA_SAVED:%s", bda_to_str(saved_bda, text, sizeof(text)));
    } else {
        ESP_LOGW(TAG, "BT_LAST_BDA_SAVE_FAILED:%s err=0x%x",
                 bda_to_str(bda, text, sizeof(text)), err);
    }
}

static void load_last_bda(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "BT_LAST_BDA:none open=0x%x", err);
        return;
    }

    size_t len = ESP_BD_ADDR_LEN;
    err = nvs_get_blob(nvs, NVS_LAST_BDA, saved_bda, &len);
    nvs_close(nvs);

    if (err == ESP_OK && len == ESP_BD_ADDR_LEN) {
        have_saved_bda = true;
        memcpy(remote_bda, saved_bda, sizeof(remote_bda));
        have_remote_bda = true;
        char text[18];
        ESP_LOGI(TAG, "BT_LAST_BDA_LOADED:%s", bda_to_str(saved_bda, text, sizeof(text)));
    } else {
        ESP_LOGI(TAG, "BT_LAST_BDA:none err=0x%x len=%u", err, (unsigned int)len);
    }
}

static void clear_last_bda(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_erase_key(nvs, NVS_LAST_BDA);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    have_saved_bda = false;
    have_remote_bda = false;
    memset(saved_bda, 0, sizeof(saved_bda));
    memset(remote_bda, 0, sizeof(remote_bda));
    ESP_LOGI(TAG, "BT_LAST_BDA_CLEARED");
}

static void clear_bonded_devices(void)
{
    int count = esp_bt_gap_get_bond_device_num();
    if (count <= 0) {
        ESP_LOGI(TAG, "BT_BOND_CLEAR:none count=%d", count);
        return;
    }

    esp_bd_addr_t *devices = calloc(count, sizeof(esp_bd_addr_t));
    if (!devices) {
        ESP_LOGW(TAG, "BT_BOND_CLEAR:no_mem count=%d", count);
        return;
    }

    int requested = count;
    esp_err_t err = esp_bt_gap_get_bond_device_list(&requested, devices);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BT_BOND_CLEAR:list_failed err=0x%x", err);
        free(devices);
        return;
    }

    for (int i = 0; i < requested; i++) {
        char bda[18];
        ESP_LOGI(TAG, "BT_BOND_CLEAR:remove:%s", bda_to_str(devices[i], bda, sizeof(bda)));
        esp_bt_gap_remove_bond_device(devices[i]);
    }
    free(devices);
    clear_last_bda();
}

static void configure_car_audio_identity(void)
{
    esp_bt_cod_t cod = {
        .reserved_2 = 0,
        .minor = 0x08,
        .major = ESP_BT_COD_MAJOR_DEV_AV,
        .service = ESP_BT_COD_SRVC_RENDERING |
                   ESP_BT_COD_SRVC_AUDIO |
                   ESP_BT_COD_SRVC_TELEPHONY,
        .reserved_8 = 0,
    };
    esp_err_t err = esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BT_COD:CAR_AUDIO service=0x%03x major=0x%02x minor=0x%02x",
                 cod.service, cod.major, cod.minor);
    } else {
        ESP_LOGW(TAG, "BT_COD_FAILED:0x%x", err);
    }
}

static void configure_discoverable_identity(const char *reason)
{
    esp_err_t name_err = esp_bt_gap_set_device_name(BT_DEVICE_NAME);
    configure_car_audio_identity();

    esp_bt_eir_data_t eir_data = {
        .fec_required = false,
        .include_txpower = true,
        .include_uuid = false,
        .include_name = true,
        .flag = ESP_BT_EIR_FLAG_GEN_DISC,
    };
    esp_err_t eir_err = esp_bt_gap_config_eir_data(&eir_data);
    esp_err_t scan_err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    ESP_LOGI(TAG, "BT_DISCOVERABLE:%s name_err=0x%x eir_err=0x%x scan_err=0x%x name=%s",
             reason, name_err, eir_err, scan_err, BT_DEVICE_NAME);
}

static void connect_profiles(esp_bd_addr_t bda)
{
    memcpy(remote_bda, bda, sizeof(remote_bda));
    have_remote_bda = true;
    save_last_bda(remote_bda);

    char text[18];
    ESP_LOGI(TAG, "CONNECTING:%s", bda_to_str(remote_bda, text, sizeof(text)));
    esp_bt_gap_cancel_discovery();
    if (a2dp_ready && !a2dp_connected && !a2dp_connecting) {
        esp_err_t err = esp_a2d_sink_connect(remote_bda);
        a2dp_connecting = err == ESP_OK;
        ESP_LOGI(TAG, "A2DP_CONNECT_TX:err=0x%x", err);
    }
    if (hf_ready && !hf_slc_connected && !hf_connecting) {
        esp_err_t err = esp_hf_client_connect(remote_bda);
        hf_connecting = err == ESP_OK;
        ESP_LOGI(TAG, "HFP_CONNECT_TX:err=0x%x", err);
    }
}

static void auto_reconnect_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(AUTO_RECONNECT_INITIAL_MS));

    while (true) {
        if (have_saved_bda && a2dp_ready && hf_ready && (!a2dp_connected || !hf_slc_connected)) {
            char text[18];
            ESP_LOGI(TAG, "AUTO_RECONNECT_TRY:%s a2dp=%d/%d hf=%d/%d",
                     bda_to_str(saved_bda, text, sizeof(text)),
                     a2dp_connected, a2dp_connecting, hf_slc_connected, hf_connecting);
            connect_profiles(saved_bda);
        }
        vTaskDelay(pdMS_TO_TICKS(AUTO_RECONNECT_PERIOD_MS));
    }
}

static void hf_clcc_query_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(250));
    if (hf_slc_connected) {
        hf_clcc_batch_pending = true;
        ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CLCC_BEGIN");
        esp_err_t err = esp_hf_client_query_current_calls();
        ESP_LOGI(TAG, "HFP_HF_CLCC_AUTO_QUERY:err=0x%x", err);
        if (err != ESP_OK) {
            hf_clcc_batch_pending = false;
            ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CLCC_END");
        }
    }
    hf_clcc_task_running = false;
    vTaskDelete(NULL);
}

static void schedule_hf_clcc_query(void)
{
    if (!hf_slc_connected || hf_clcc_task_running) {
        return;
    }
    hf_clcc_task_running = true;
    BaseType_t ok = xTaskCreate(hf_clcc_query_task, "hf_clcc_query", 3072, NULL, 5, NULL);
    if (ok != pdPASS) {
        hf_clcc_task_running = false;
        ESP_LOGW(TAG, "HFP_HF_CLCC_AUTO_QUERY_TASK_FAILED");
    }
}

static void handle_uart_command(char *line)
{
    line = trim(line);
    if (!*line) {
        return;
    }
    ESP_LOGI(TAG, "UART_RX:%s", line);

    if (strcmp(line, "BOARD_ID") == 0 || strcmp(line, "IDENT") == 0) {
        log_board_id();
    } else if (strcmp(line, "HELP") == 0) {
        log_board_id();
        ESP_LOGI(TAG, "CMDS:DISCOVERABLE CONNECTABLE HIDDEN CLEAR_PAIRING SCAN STOP_SCAN CONNECT:<bda> DISCONNECT STATUS");
        ESP_LOGI(TAG, "CMDS:AVRCP_PLAY AVRCP_PAUSE AVRCP_STOP AVRCP_NEXT AVRCP_PREVIOUS AVRCP_STATUS AVRCP_METADATA AVRCP_RN");
        ESP_LOGI(TAG, "CMDS:HF_ANSWER HF_HANGUP HF_CLCC HF_COPS HF_CNUM HF_AUDIO_CONNECT HF_AUDIO_DISCONNECT HF_CHLD:<0|1|2|3|4|1x|2x> HF_VOL:<0-15> HF_DTMF:<char> HF_NREC");
    } else if (strcmp(line, "SCAN") == 0) {
        esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
        ESP_LOGI(TAG, "SCAN_START:err=0x%x", err);
    } else if (strcmp(line, "STOP_SCAN") == 0) {
        esp_err_t err = esp_bt_gap_cancel_discovery();
        ESP_LOGI(TAG, "SCAN_STOP:err=0x%x", err);
    } else if (strcmp(line, "DISCOVERABLE") == 0 || strcmp(line, "PAIRABLE") == 0) {
        configure_discoverable_identity("uart");
    } else if (strcmp(line, "CLEAR_PAIRING") == 0) {
        clear_bonded_devices();
        clear_last_bda();
        configure_discoverable_identity("clear_pairing");
    } else if (strcmp(line, "CONNECTABLE") == 0) {
        esp_err_t err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        ESP_LOGI(TAG, "BT_SCAN_MODE:CONNECTABLE err=0x%x", err);
    } else if (strcmp(line, "HIDDEN") == 0 || strcmp(line, "PRIVATE") == 0) {
        esp_err_t err = esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        ESP_LOGI(TAG, "BT_SCAN_MODE:HIDDEN err=0x%x", err);
    } else if (strncmp(line, "CONNECT:", 8) == 0) {
        esp_bd_addr_t bda;
        if (parse_bda(line + 8, bda)) {
            connect_profiles(bda);
        } else {
            ESP_LOGW(TAG, "CONNECT_BAD_BDA:%s", line + 8);
        }
    } else if (strcmp(line, "DISCONNECT") == 0) {
        if (ensure_remote()) {
            esp_hf_client_disconnect(remote_bda);
            esp_a2d_sink_disconnect(remote_bda);
        }
    } else if (strcmp(line, "STATUS") == 0) {
        log_status();
    } else if (strcmp(line, "AVRCP_PLAY") == 0) {
        send_avrcp_passthrough(ESP_AVRC_PT_CMD_PLAY);
    } else if (strcmp(line, "AVRCP_PAUSE") == 0) {
        send_avrcp_passthrough(ESP_AVRC_PT_CMD_PAUSE);
    } else if (strcmp(line, "AVRCP_STOP") == 0) {
        send_avrcp_passthrough(ESP_AVRC_PT_CMD_STOP);
    } else if (strcmp(line, "AVRCP_NEXT") == 0) {
        send_avrcp_passthrough(ESP_AVRC_PT_CMD_FORWARD);
        avrcp_schedule_delayed_snapshot("bridge_next_delayed");
    } else if (strcmp(line, "AVRCP_PREVIOUS") == 0) {
        send_avrcp_passthrough(ESP_AVRC_PT_CMD_BACKWARD);
        avrcp_schedule_delayed_snapshot("bridge_previous_delayed");
    } else if (strcmp(line, "AVRCP_STATUS") == 0) {
        esp_avrc_ct_send_get_play_status_cmd(next_tl());
    } else if (strcmp(line, "AVRCP_METADATA") == 0) {
        esp_avrc_ct_send_metadata_cmd(next_tl(), avrcp_metadata_mask());
    } else if (strcmp(line, "AVRCP_RN") == 0) {
        esp_err_t err = esp_avrc_ct_send_get_rn_capabilities_cmd(next_tl());
        ESP_LOGI(TAG, "AVRCP_MONITOR_CAPS_REQUEST:uart err=0x%x", err);
        avrcp_register_supported_events("uart");
        avrcp_request_snapshot("uart");
    } else if (strcmp(line, "AVRCP_SNAPSHOT") == 0) {
        avrcp_force_snapshot("bridge");
    } else if (strcmp(line, "HF_ANSWER") == 0) {
        esp_hf_client_answer_call();
    } else if (strcmp(line, "HF_HANGUP") == 0 || strcmp(line, "HF_REJECT") == 0) {
        esp_hf_client_reject_call();
    } else if (strcmp(line, "HF_CLCC") == 0) {
        hf_clcc_batch_pending = true;
        ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CLCC_BEGIN");
        esp_err_t err = esp_hf_client_query_current_calls();
        if (err != ESP_OK) {
            hf_clcc_batch_pending = false;
            ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CLCC_END");
        }
    } else if (strcmp(line, "HF_COPS") == 0) {
        esp_hf_client_query_current_operator_name();
    } else if (strcmp(line, "HF_CNUM") == 0) {
        esp_hf_client_retrieve_subscriber_info();
    } else if (strcmp(line, "HF_AUDIO_CONNECT") == 0) {
        if (ensure_remote()) {
            esp_hf_client_connect_audio(remote_bda);
        }
    } else if (strcmp(line, "HF_AUDIO_DISCONNECT") == 0) {
        if (ensure_remote()) {
            esp_hf_client_disconnect_audio(remote_bda);
        }
    } else if (strncmp(line, "HF_CHLD:", 8) == 0) {
        esp_hf_chld_type_t type;
        int index;
        if (parse_chld(line + 8, &type, &index)) {
            esp_err_t err = esp_hf_client_send_chld_cmd(type, index);
            ESP_LOGI(TAG, "HF_CHLD_TX:%s type=%d idx=%d err=0x%x", line + 8, type, index, err);
        } else {
            ESP_LOGW(TAG, "HF_CHLD_BAD:%s", line + 8);
        }
    } else if (strncmp(line, "HF_VOL:", 7) == 0) {
        int vol = atoi(line + 7);
        esp_hf_client_volume_update(ESP_HF_VOLUME_CONTROL_TARGET_SPK, vol);
    } else if (strncmp(line, "HF_DTMF:", 8) == 0) {
        esp_hf_client_send_dtmf(line[8]);
    } else if (strcmp(line, "HF_NREC") == 0) {
        esp_hf_client_send_nrec();
    } else if (strncmp(line, "BRIDGE_RX:", 10) == 0) {
        char bridged[UART_LINE_SIZE];
        snprintf(bridged, sizeof(bridged), "%s", line + 10);
        ESP_LOGI(TAG, "BRIDGE_RX:%s", line + 10);
        handle_uart_command(bridged);
    } else {
        ESP_LOGW(TAG, "UNKNOWN_CMD:%s", line);
    }
}

static void uart_task(void *arg)
{
    (void)arg;
    uint8_t byte;
    char line[UART_LINE_SIZE];
    size_t len = 0;

    while (true) {
        int n = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0) {
            continue;
        }
        if (byte == '\n' || byte == '\r') {
            if (len > 0) {
                line[len] = '\0';
                handle_uart_command(line);
                len = 0;
            }
        } else if (len + 1 < sizeof(line)) {
            line[len++] = (char)byte;
        } else {
            len = 0;
            ESP_LOGW(TAG, "UART_LINE_TOO_LONG");
        }
    }
}

static esp_err_t i2s_output_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 12;
    chan_cfg.dma_frame_num = 256;
    chan_cfg.auto_clear = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx, NULL);
    if (err != ESP_OK) {
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(i2s_tx, &std_cfg);
    if (err != ESP_OK) {
        return err;
    }
    err = i2s_channel_enable(i2s_tx);
    if (err == ESP_OK) {
        a2dp_i2s_enabled = true;
    }
    return err;
}

static bool pcm_ringbuf_send_latest(const uint8_t *data, uint32_t len)
{
    if (pcm_ringbuf == NULL || data == NULL || len == 0) {
        return false;
    }

    if (xRingbufferSend(pcm_ringbuf, data, len, 0) == pdTRUE) {
        return true;
    }

    size_t drop_len = 0;
    void *drop = xRingbufferReceiveUpTo(pcm_ringbuf, &drop_len, 0, len);
    if (drop != NULL) {
        pcm_ring_drop_bytes += drop_len;
        pcm_ring_drop_count++;
        vRingbufferReturnItem(pcm_ringbuf, drop);
    } else {
        pcm_ring_drop_bytes += len;
        pcm_ring_drop_count++;
        return false;
    }

    if (xRingbufferSend(pcm_ringbuf, data, len, 0) == pdTRUE) {
        return true;
    }

    pcm_ring_drop_bytes += len;
    pcm_ring_drop_count++;
    return false;
}

static void pcm_ringbuf_drain(void)
{
    if (pcm_ringbuf == NULL) {
        return;
    }

    for (;;) {
        size_t len = 0;
        void *item = xRingbufferReceive(pcm_ringbuf, &len, 0);
        if (item == NULL) {
            return;
        }
        vRingbufferReturnItem(pcm_ringbuf, item);
    }
}

static void i2s_tx_task(void *arg)
{
    (void)arg;

    for (;;) {
        size_t len = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceive(pcm_ringbuf, &len, portMAX_DELAY);
        if (data == NULL || len == 0) {
            continue;
        }
        if (!a2dp_audio_started) {
            vRingbufferReturnItem(pcm_ringbuf, data);
            continue;
        }

        size_t offset = 0;
        while (offset < len && a2dp_audio_started) {
            size_t bytes_written = 0;
            esp_err_t err = i2s_channel_write(i2s_tx, data + offset, len - offset,
                                              &bytes_written, portMAX_DELAY);
            i2s_tx_bytes += bytes_written;
            offset += bytes_written;

            if (err != ESP_OK || bytes_written == 0) {
                i2s_tx_short_count++;
                if (i2s_tx_short_count <= 8 || (i2s_tx_short_count % 128) == 0) {
                    ESP_LOGW(TAG, "I2S_TX_SHORT:err=0x%x wrote=%u left=%u",
                             err, (unsigned int)bytes_written,
                             (unsigned int)(len - offset));
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        if (i2s_tx_bytes >= i2s_tx_report_next) {
            ESP_LOGI(TAG,
                     "I2S_TX:bytes=%" PRIu32 " short=%" PRIu32
                     " rx_pcm=%" PRIu32 " peak=%u ring_drop=%" PRIu32 "/%" PRIu32,
                     i2s_tx_bytes, i2s_tx_short_count, a2dp_rx_bytes, i2s_tx_peak,
                     pcm_ring_drop_count, pcm_ring_drop_bytes);
            i2s_tx_report_next = i2s_tx_bytes + 32768;
        }

        vRingbufferReturnItem(pcm_ringbuf, data);
    }
}

static void a2dp_sink_data_cb(const uint8_t *data, uint32_t len)
{
    if (data && len > 0) {
        if (!a2dp_audio_started) {
            return;
        }
        a2dp_rx_bytes += len;
        uint16_t peak = 0;
        const int16_t *samples = (const int16_t *)data;
        const uint32_t sample_count = len / sizeof(int16_t);
        for (uint32_t i = 0; i < sample_count; i++) {
            int32_t sample = samples[i];
            uint16_t abs_sample = (uint16_t)(sample < 0 ? -sample : sample);
            if (abs_sample > peak) {
                peak = abs_sample;
            }
        }
        i2s_tx_peak = peak;

        if (!pcm_ringbuf_send_latest(data, len)) {
            ESP_LOGW(TAG, "PCM_RING_DROP:count=%" PRIu32 " bytes=%" PRIu32,
                     pcm_ring_drop_count, pcm_ring_drop_bytes);
        }
    }
}

static void a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    char bda[18];
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        a2dp_connected = param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED;
        a2dp_connecting = param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING;
        if (!a2dp_connected) {
            a2dp_audio_started = false;
            pcm_ringbuf_drain();
        }
        if (a2dp_connected) {
            memcpy(remote_bda, param->conn_stat.remote_bda, sizeof(remote_bda));
            have_remote_bda = true;
            save_last_bda(remote_bda);
        }
        ESP_LOGI(TAG, "A2DP_CONNECTION:state=%d bda=%s handle=%u mtu=%u reason=%d",
                 param->conn_stat.state,
                 bda_to_str(param->conn_stat.remote_bda, bda, sizeof(bda)),
                 param->conn_stat.conn_hdl, param->conn_stat.audio_mtu,
                 param->conn_stat.disc_rsn);
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        a2dp_audio_started = param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED;
        if (!a2dp_audio_started) {
            pcm_ringbuf_drain();
        }
        ESP_LOGI(TAG, "A2DP_AUDIO:state=%d handle=%u", param->audio_stat.state, param->audio_stat.conn_hdl);
        ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_NOTIFY_PLAYBACK:%d",
                 param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED ?
                 ESP_AVRC_PLAYBACK_PLAYING : ESP_AVRC_PLAYBACK_PAUSED);
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(TAG, "A2DP_AUDIO_CFG:handle=%u codec=%u", param->audio_cfg.conn_hdl, param->audio_cfg.mcc.type);
        if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            const esp_a2d_cie_sbc_t *sbc = &param->audio_cfg.mcc.cie.sbc_info;
            snprintf(a2dp_rate_status, sizeof(a2dp_rate_status),
                     "A2DP_AUDIO_RATE:codec=SBC sample_rate=%" PRIu32
                     " channels=%s bitpool=%u-%u blocks=%u subbands=%u alloc=%s",
                     a2dp_sbc_sample_rate(sbc->samp_freq), a2dp_sbc_channel_mode(sbc->ch_mode),
                     sbc->min_bitpool, sbc->max_bitpool, a2dp_sbc_block_len(sbc->block_len),
                     a2dp_sbc_subbands(sbc->num_subbands), a2dp_sbc_alloc_method(sbc->alloc_mthd));
            ESP_LOGI(TAG, "%s", a2dp_rate_status);
        } else {
            snprintf(a2dp_rate_status, sizeof(a2dp_rate_status),
                     "A2DP_AUDIO_RATE:codec=%u sample_rate=0 channels=unknown",
                     param->audio_cfg.mcc.type);
            ESP_LOGI(TAG, "%s", a2dp_rate_status);
        }
        break;
    case ESP_A2D_PROF_STATE_EVT:
        ESP_LOGI(TAG, "A2DP_PROFILE:init_state=%d", param->a2d_prof_stat.init_state);
        if (param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS) {
            a2dp_ready = true;
            configure_discoverable_identity("a2dp_ready");
        }
        break;
    default:
        ESP_LOGI(TAG, "A2DP_EVENT:%d", event);
        break;
    }
}

static void avrcp_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    char bda[18];
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        avrcp_connected = param->conn_stat.connected;
        if (!avrcp_connected) {
            avrcp_have_rn_caps = false;
            avrcp_have_last_status = false;
            avrcp_have_last_track_id = false;
            memset(avrcp_have_last_metadata, 0, sizeof(avrcp_have_last_metadata));
            memset(&avrcp_peer_rn_caps, 0, sizeof(avrcp_peer_rn_caps));
        }
        ESP_LOGI(TAG, "AVRCP_CT_CONNECTION:connected=%d bda=%s",
                 param->conn_stat.connected,
                 bda_to_str(param->conn_stat.remote_bda, bda, sizeof(bda)));
        if (avrcp_connected) {
            esp_err_t caps_err = esp_avrc_ct_send_get_rn_capabilities_cmd(next_tl());
            ESP_LOGI(TAG, "AVRCP_MONITOR_CAPS_REQUEST:connect err=0x%x", caps_err);
            avrcp_request_snapshot("connect");
        }
        break;
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
        ESP_LOGI(TAG, "AVRCP_CT_FEATURES:bda=%s feat=0x%" PRIx32 " tg_flag=0x%x",
                 bda_to_str(param->rmt_feats.remote_bda, bda, sizeof(bda)),
                 param->rmt_feats.feat_mask, param->rmt_feats.tg_feat_flag);
        break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
        ESP_LOGI(TAG, "AVRCP_CT_RSP:key=0x%02x state=%u rsp=%u tl=%u",
                 param->psth_rsp.key_code, param->psth_rsp.key_state,
                 param->psth_rsp.rsp_code, param->psth_rsp.tl);
        break;
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        if (avrcp_metadata_is_duplicate(param->meta_rsp.attr_id,
                                        param->meta_rsp.attr_text,
                                        param->meta_rsp.attr_length)) {
            break;
        }
        ESP_LOGI(TAG, "AVRCP_METADATA:attr=%u len=%d text=\"%.*s\"",
                 param->meta_rsp.attr_id, param->meta_rsp.attr_length,
                 param->meta_rsp.attr_length, (const char *)param->meta_rsp.attr_text);
        ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_METADATA:attr=%u len=%d text=\"%.*s\"",
                 param->meta_rsp.attr_id, param->meta_rsp.attr_length,
                 param->meta_rsp.attr_length, (const char *)param->meta_rsp.attr_text);
        break;
    case ESP_AVRC_CT_PLAY_STATUS_RSP_EVT:
        if (avrcp_have_last_status &&
            avrcp_last_song_length == param->play_status_rsp.song_length &&
            avrcp_last_song_position == param->play_status_rsp.song_position &&
            avrcp_last_play_status == param->play_status_rsp.play_status) {
            break;
        }
        avrcp_last_song_length = param->play_status_rsp.song_length;
        avrcp_last_song_position = param->play_status_rsp.song_position;
        avrcp_last_play_status = param->play_status_rsp.play_status;
        avrcp_have_last_status = true;
        ESP_LOGI(TAG, "AVRCP_STATUS:len=%" PRIu32 " pos=%" PRIu32 " status=%d",
                 param->play_status_rsp.song_length,
                 param->play_status_rsp.song_position,
                 param->play_status_rsp.play_status);
        ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_STATUS:len=%" PRIu32 " pos=%" PRIu32 " status=%d",
                 param->play_status_rsp.song_length,
                 param->play_status_rsp.song_position,
                 param->play_status_rsp.play_status);
        break;
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
        avrcp_peer_rn_caps = param->get_rn_caps_rsp.evt_set;
        avrcp_have_rn_caps = true;
        ESP_LOGI(TAG, "AVRCP_RN_CAPS:count=%u bits=0x%x",
                 param->get_rn_caps_rsp.cap_count, param->get_rn_caps_rsp.evt_set.bits);
        avrcp_register_supported_events("caps");
        break;
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        switch (param->change_ntf.event_id) {
        case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
            ESP_LOGI(TAG, "AVRCP_NOTIFY:event=0x%02x(%s) playback=%d",
                     param->change_ntf.event_id,
                     avrcp_event_name(param->change_ntf.event_id),
                     param->change_ntf.event_parameter.playback);
            ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_NOTIFY_PLAYBACK:%d",
                     param->change_ntf.event_parameter.playback);
            avrcp_request_snapshot("play_status_change");
            if (param->change_ntf.event_parameter.playback == ESP_AVRC_PLAYBACK_PLAYING) {
                avrcp_schedule_delayed_snapshot("play_started_delayed");
            }
            break;
        case ESP_AVRC_RN_TRACK_CHANGE:
            if (avrcp_have_last_track_id &&
                memcmp(avrcp_last_track_id,
                       param->change_ntf.event_parameter.elm_id,
                       sizeof(avrcp_last_track_id)) == 0) {
                break;
            }
            memcpy(avrcp_last_track_id,
                   param->change_ntf.event_parameter.elm_id,
                   sizeof(avrcp_last_track_id));
            avrcp_have_last_track_id = true;
            ESP_LOGI(TAG, "AVRCP_NOTIFY:event=0x%02x(%s) track=%02x%02x%02x%02x%02x%02x%02x%02x",
                     param->change_ntf.event_id,
                     avrcp_event_name(param->change_ntf.event_id),
                     param->change_ntf.event_parameter.elm_id[0],
                     param->change_ntf.event_parameter.elm_id[1],
                     param->change_ntf.event_parameter.elm_id[2],
                     param->change_ntf.event_parameter.elm_id[3],
                     param->change_ntf.event_parameter.elm_id[4],
                     param->change_ntf.event_parameter.elm_id[5],
                     param->change_ntf.event_parameter.elm_id[6],
                     param->change_ntf.event_parameter.elm_id[7]);
            avrcp_request_snapshot("track_change");
            avrcp_schedule_delayed_snapshot("track_change_delayed");
            break;
        case ESP_AVRC_RN_PLAY_POS_CHANGED:
            if (avrcp_have_last_status &&
                avrcp_last_song_position == param->change_ntf.event_parameter.play_pos) {
                break;
            }
            if (avrcp_have_last_status &&
                param->change_ntf.event_parameter.play_pos + 1000 < avrcp_last_song_position) {
                avrcp_schedule_delayed_snapshot("pos_reset_delayed");
            }
            avrcp_last_song_position = param->change_ntf.event_parameter.play_pos;
            avrcp_have_last_status = true;
            ESP_LOGI(TAG, "AVRCP_NOTIFY:event=0x%02x(%s) pos=%" PRIu32,
                     param->change_ntf.event_id,
                     avrcp_event_name(param->change_ntf.event_id),
                     param->change_ntf.event_parameter.play_pos);
            ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_POS:%" PRIu32,
                     param->change_ntf.event_parameter.play_pos);
            break;
        case ESP_AVRC_RN_BATTERY_STATUS_CHANGE:
            ESP_LOGI(TAG, "AVRCP_NOTIFY:event=0x%02x(%s) batt=%d",
                     param->change_ntf.event_id,
                     avrcp_event_name(param->change_ntf.event_id),
                     param->change_ntf.event_parameter.batt);
            break;
        case ESP_AVRC_RN_VOLUME_CHANGE:
            ESP_LOGI(TAG, "AVRCP_NOTIFY:event=0x%02x(%s) volume=%u",
                     param->change_ntf.event_id,
                     avrcp_event_name(param->change_ntf.event_id),
                     param->change_ntf.event_parameter.volume);
            break;
        default:
            ESP_LOGI(TAG, "AVRCP_NOTIFY:event=0x%02x(%s)",
                     param->change_ntf.event_id,
                     avrcp_event_name(param->change_ntf.event_id));
            break;
        }
        avrcp_register_event(param->change_ntf.event_id, "notify");
        break;
    case ESP_AVRC_CT_PROF_STATE_EVT:
        ESP_LOGI(TAG, "AVRCP_CT_PROFILE:state=%d", param->avrc_ct_init_stat.state);
        break;
    default:
        ESP_LOGI(TAG, "AVRCP_CT_EVENT:%d", event);
        break;
    }
}

static void hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    char bda[18];
    switch (event) {
    case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
        hf_slc_connected = param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED;
        hf_connecting = param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING ||
                        param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED;
        if (hf_slc_connected) {
            memcpy(remote_bda, param->conn_stat.remote_bda, sizeof(remote_bda));
            have_remote_bda = true;
            save_last_bda(remote_bda);
        }
        ESP_LOGI(TAG, "HFP_HF_CONNECTION:state=%d bda=%s peer=0x%" PRIx32 " chld=0x%" PRIx32,
                 param->conn_stat.state,
                 bda_to_str(param->conn_stat.remote_bda, bda, sizeof(bda)),
                 param->conn_stat.peer_feat, param->conn_stat.chld_feat);
        if (hf_slc_connected) {
            schedule_hf_clcc_query();
        }
        break;
    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        hf_audio_connected = param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                             param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC;
        if (hf_audio_connected) {
            sco_pcm_gpio_config();
        } else if (sco_pcm_gpio_active) {
            a2dp_i2s_gpio_config();
        }
        ESP_LOGI(TAG, "HFP_HF_AUDIO:state=%d bda=%s handle=%d frame=%u",
                 param->audio_stat.state,
                 bda_to_str(param->audio_stat.remote_bda, bda, sizeof(bda)),
                 param->audio_stat.sync_conn_handle,
                 param->audio_stat.preferred_frame_size);
        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
            snprintf(hfp_rate_status, sizeof(hfp_rate_status),
                     "HFP_HF_AUDIO_RATE:codec=mSBC sample_rate=16000 channels=mono frame=%u",
                     param->audio_stat.preferred_frame_size);
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED) {
            snprintf(hfp_rate_status, sizeof(hfp_rate_status),
                     "HFP_HF_AUDIO_RATE:codec=CVSD sample_rate=8000 channels=mono frame=%u",
                     param->audio_stat.preferred_frame_size);
        } else {
            snprintf(hfp_rate_status, sizeof(hfp_rate_status),
                     "HFP_HF_AUDIO_RATE:codec=none sample_rate=0 channels=mono frame=%u",
                     param->audio_stat.preferred_frame_size);
        }
        ESP_LOGI(TAG, "%s", hfp_rate_status);
        break;
    case ESP_HF_CLIENT_CIND_CALL_EVT:
        ESP_LOGI(TAG, "HFP_HF_CIND_CALL:%d", param->call.status);
        ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CIND_CALL:%d", param->call.status);
        schedule_hf_clcc_query();
        break;
    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        ESP_LOGI(TAG, "HFP_HF_CIND_CALLSETUP:%d", param->call_setup.status);
        ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CIND_CALLSETUP:%d", param->call_setup.status);
        schedule_hf_clcc_query();
        break;
    case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        ESP_LOGI(TAG, "HFP_HF_CIND_CALLHELD:%d", param->call_held.status);
        ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CIND_CALLHELD:%d", param->call_held.status);
        schedule_hf_clcc_query();
        break;
    case ESP_HF_CLIENT_CLIP_EVT:
        ESP_LOGI(TAG, "HFP_HF_CLIP:number=\"%s\"", param->clip.number ? param->clip.number : "");
        ESP_LOGI(TAG, "BRIDGE_TX:CALL_IN:%s", param->clip.number ? param->clip.number : "");
        schedule_hf_clcc_query();
        break;
    case ESP_HF_CLIENT_CCWA_EVT:
        ESP_LOGI(TAG, "HFP_HF_CCWA:number=\"%s\"", param->ccwa.number ? param->ccwa.number : "");
        ESP_LOGI(TAG, "BRIDGE_TX:CALL_IN:%s", param->ccwa.number ? param->ccwa.number : "");
        schedule_hf_clcc_query();
        break;
    case ESP_HF_CLIENT_CLCC_EVT:
        ESP_LOGI(TAG, "HFP_HF_CLCC:idx=%d dir=%d status=%d mpty=%d num=\"%s\"",
                 param->clcc.idx, param->clcc.dir, param->clcc.status,
                 param->clcc.mpty, param->clcc.number ? param->clcc.number : "");
        ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CLCC:idx=%d dir=%d status=%d mpty=%d num=\"%s\"",
                 param->clcc.idx, param->clcc.dir, param->clcc.status,
                 param->clcc.mpty, param->clcc.number ? param->clcc.number : "");
        break;
    case ESP_HF_CLIENT_RING_IND_EVT:
        ESP_LOGI(TAG, "HFP_HF_RING");
        ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_RING");
        schedule_hf_clcc_query();
        break;
    case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "HFP_HF_VOLUME:type=%d volume=%d",
                 param->volume_control.type, param->volume_control.volume);
        break;
    case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        ESP_LOGI(TAG, "HFP_HF_AT_RESPONSE:code=%d cme=%d",
                 param->at_response.code, param->at_response.cme);
        if (hf_clcc_batch_pending) {
            hf_clcc_batch_pending = false;
            ESP_LOGI(TAG, "BRIDGE_TX:HFP_HF_CLCC_END");
        }
        break;
    case ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT:
        ESP_LOGI(TAG, "HFP_HF_COPS:name=\"%s\"", param->cops.name ? param->cops.name : "");
        break;
    case ESP_HF_CLIENT_CNUM_EVT:
        ESP_LOGI(TAG, "HFP_HF_CNUM:number=\"%s\" type=%d",
                 param->cnum.number ? param->cnum.number : "", param->cnum.type);
        break;
    case ESP_HF_CLIENT_BSIR_EVT:
        ESP_LOGI(TAG, "HFP_HF_BSIR:%d", param->bsir.state);
        break;
    case ESP_HF_CLIENT_PROF_STATE_EVT:
        ESP_LOGI(TAG, "HFP_HF_PROFILE:state=%d", param->prof_stat.state);
        if (param->prof_stat.state == ESP_HF_INIT_SUCCESS) {
            hf_ready = true;
        }
        break;
    default:
        ESP_LOGI(TAG, "HFP_HF_EVENT:%d", event);
        break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        char bda[18];
        char name[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = "";
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
            if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME) {
                size_t len = prop->len < ESP_BT_GAP_MAX_BDNAME_LEN ? prop->len : ESP_BT_GAP_MAX_BDNAME_LEN;
                memcpy(name, prop->val, len);
                name[len] = '\0';
            } else if (prop->type == ESP_BT_GAP_DEV_PROP_EIR) {
                uint8_t len = 0;
                uint8_t *eir_name = esp_bt_gap_resolve_eir_data(prop->val, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
                if (!eir_name) {
                    eir_name = esp_bt_gap_resolve_eir_data(prop->val, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
                }
                if (eir_name && len > 0) {
                    size_t copy_len = len < ESP_BT_GAP_MAX_BDNAME_LEN ? len : ESP_BT_GAP_MAX_BDNAME_LEN;
                    memcpy(name, eir_name, copy_len);
                    name[copy_len] = '\0';
                }
            }
        }
        ESP_LOGI(TAG, "DISCOVERED:bda=%s name=\"%s\"",
                 bda_to_str(param->disc_res.bda, bda, sizeof(bda)), name);
        if (strlen(BT_REMOTE_NAME) > 0 && strcmp(name, BT_REMOTE_NAME) == 0) {
            connect_profiles(param->disc_res.bda);
        }
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(TAG, "DISCOVERY_STATE:%d", param->disc_st_chg.state);
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(TAG, "SCAN_STARTED");
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(TAG, "SCAN_DONE:0");
        }
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        ESP_LOGI(TAG, "AUTH:%s bda=%02x:%02x:%02x:%02x:%02x:%02x name=\"%s\"",
                 param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS ? "OK" : "FAIL",
                 param->auth_cmpl.bda[0], param->auth_cmpl.bda[1], param->auth_cmpl.bda[2],
                 param->auth_cmpl.bda[3], param->auth_cmpl.bda[4], param->auth_cmpl.bda[5],
                 param->auth_cmpl.device_name);
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            memcpy(remote_bda, param->auth_cmpl.bda, sizeof(remote_bda));
            have_remote_bda = true;
            save_last_bda(remote_bda);
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        char bda[18];
        esp_bt_pin_code_t pin_code = {'0', '0', '0', '0'};
        memcpy(remote_bda, param->pin_req.bda, sizeof(remote_bda));
        have_remote_bda = true;
        ESP_LOGW(TAG, "PIN_REQUIRED:%s:min16=%d:auto=0000",
                 bda_to_str(param->pin_req.bda, bda, sizeof(bda)),
                 param->pin_req.min_16_digit);
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        break;
    }
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP_CONFIRM:%06" PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
        ESP_LOGI(TAG, "BT_EIR_CONFIG:status=0x%x types=%u",
                 param->config_eir_data.stat,
                 (unsigned int)param->config_eir_data.eir_type_num);
        break;
    case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT: {
        char bda[18];
        ESP_LOGI(TAG, "BT_BOND_REMOVED:%s status=0x%x",
                 bda_to_str(param->remove_bond_dev_cmpl.bda, bda, sizeof(bda)),
                 param->remove_bond_dev_cmpl.status);
        break;
    }
    default:
        ESP_LOGI(TAG, "GAP_EVENT:%d", event);
        break;
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }
    load_last_bda();

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(rgb_led_init());
    ESP_ERROR_CHECK(i2s_output_init());
    pcm_ringbuf = xRingbufferCreate(PCM_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    ESP_ERROR_CHECK(pcm_ringbuf == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    BaseType_t task_ok = xTaskCreate(i2s_tx_task, "i2s_tx", 4096, NULL, 18, NULL);
    ESP_ERROR_CHECK(task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "I2S_TX_READY:rate=%d bclk=%d ws=%d dout=%d role=master dma=12x256 ring=%u",
             SAMPLE_RATE, I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DOUT_GPIO,
             (unsigned int)PCM_RINGBUF_SIZE);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bredr_sco_datapath_set(ESP_SCO_DATA_PATH_PCM));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(BT_DEVICE_NAME));

    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(avrcp_ct_cb));
    ESP_ERROR_CHECK(esp_a2d_register_callback(a2dp_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(a2dp_sink_data_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_init());
    ESP_ERROR_CHECK(esp_hf_client_register_callback(hf_client_cb));
    ESP_ERROR_CHECK(esp_hf_client_init());

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap)));

    esp_bt_pin_code_t pin_code = {0};
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, pin_code));

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "rgb_led", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(auto_reconnect_task, "auto_reconnect", 4096, NULL, 4, NULL);

    const uint8_t *addr = esp_bt_dev_get_address();
    ESP_LOGI(TAG, "BT_READY:name=%s bda=%02x:%02x:%02x:%02x:%02x:%02x",
             BT_DEVICE_NAME, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    log_board_id();
    ESP_LOGI(TAG, "BT_BOND_COUNT:boot:%d", esp_bt_gap_get_bond_device_num());
    ESP_LOGI(TAG, "ROLE:A2DP_SINK HFP_HF AVRCP_CT");
#ifdef CONFIG_BT_HFP_WBS_ENABLE
    ESP_LOGW(TAG, "HFP_AUDIO_POLICY:WBS_ENABLED mSBC may be advertised");
#else
    ESP_LOGI(TAG, "HFP_AUDIO_POLICY:CVSD_ONLY sample_rate=8000 wbs=0 sco_path=PCM");
#endif
#ifdef CONFIG_BTDM_CTRL_PCM_ROLE_SLAVE
    ESP_LOGI(TAG, "HFP_SCO_PCM:role=slave polar=falling fsync=stereo");
#else
    ESP_LOGI(TAG, "HFP_SCO_PCM:role=master polar=falling fsync=stereo");
#endif
    ESP_LOGI(TAG, "HFP_SCO_PCM_GPIO_PLAN:clk=%d fsync=%d dout=%d din=%d",
             SCO_PCM_CLK_GPIO, SCO_PCM_FSYNC_GPIO, SCO_PCM_DOUT_GPIO, SCO_PCM_DIN_GPIO);
    ESP_LOGI(TAG, "UART:HELP for commands");

    if (strlen(BT_REMOTE_NAME) > 0) {
        ESP_LOGI(TAG, "AUTO_SCAN_FOR_NAME:%s", BT_REMOTE_NAME);
        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    }
}
