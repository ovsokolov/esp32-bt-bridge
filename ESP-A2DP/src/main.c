#include <ctype.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt.h"
#include "esp_check.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_ag_api.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/gpio_sig_map.h"
#include "voice_samples.h"

static const char *TAG = "ESP_A2DP";

#ifndef SAMPLE_RATE
#define SAMPLE_RATE 44100
#endif

#ifndef I2S_BCLK_GPIO
#define I2S_BCLK_GPIO 5
#endif

#ifndef I2S_WS_GPIO
#define I2S_WS_GPIO 25
#endif

#ifndef I2S_DIN_GPIO
#define I2S_DIN_GPIO 21
#endif

#ifndef SCO_PCM_CLK_GPIO
#define SCO_PCM_CLK_GPIO I2S_BCLK_GPIO
#endif

#ifndef SCO_PCM_FSYNC_GPIO
#define SCO_PCM_FSYNC_GPIO I2S_WS_GPIO
#endif

#ifndef SCO_PCM_DOUT_GPIO
#define SCO_PCM_DOUT_GPIO 19
#endif

#ifndef SCO_PCM_DIN_GPIO
#define SCO_PCM_DIN_GPIO I2S_DIN_GPIO
#endif

#ifndef RGB_LED_GPIO
#define RGB_LED_GPIO 0
#endif

#ifndef RGB_LED_POWER_GPIO
#define RGB_LED_POWER_GPIO 2
#endif

#ifndef BT_DEVICE_NAME
#define BT_DEVICE_NAME "ESP-A2DP-Source"
#endif

#define AG_BUILD_TAG "ag-hfp-force-outband-ring-20260526"

#ifndef BT_REMOTE_NAME
#define BT_REMOTE_NAME ""
#endif

#define NVS_NS "a2dp"
#define NVS_LAST_BDA "last_bda"
#define AUDIO_CHANNELS 2
#define I2S_READ_TIMEOUT_MS 50
#define I2S_EMPTY_READ_DELAY_TICKS 1
#define I2S_RX_SHORT_REPORT_PERIOD 32768
#define PCM_RINGBUF_SIZE (24 * 1024)
#define I2S_RX_TASK_CHUNK_BYTES 1024
#define RECONNECT_PERIOD_MS 3000
#define POST_DISCOVERY_CONNECT_DELAY_MS 200
#define BOOT_ACTIVE_RECONNECT_DELAY_MS 2000
#define AUTO_RECONNECT_MAX_ATTEMPTS 40
#define AUTO_RECONNECT_BACKOFF_STEP_MS 5000
#define AUTO_RECONNECT_BACKOFF_MAX_MS 60000
#define PASSIVE_ACL_RECONNECT_DELAY_MS 5000
#define UART_CMD_BUF_LEN 512
#define MAX_DISCOVERED_DEVICES 16
#define HFP_MAX_CALLS 6
#define RGB_RMT_RESOLUTION_HZ 10000000U
#define SIMULATED_TRACK_COUNT 3
#define SIMULATED_TRACK_LEN_MS 15000
#define SIMULATED_TRACK_GAP_FRAMES SAMPLE_RATE
#define HFP_CALL_CONTROL_TIMEOUT_MS 1000
#define HFP_CHLD2_DEFER_MS 300
#define HFP_POST_HANGUP_SUPPRESS_MS 8000
#define HFP_OUTBAND_RING_REPEAT_MS 2000
#define BRIDGE_SNAPSHOT_MIN_INTERVAL_MS 1500
#define A2DP_I2S_GATE_DURING_SCO 0
#define BTA_AG_HANDLE_ALL_COMPAT 0xFFFF
#define BTA_AG_IN_CALL_RES_COMPAT 11
#define BTA_AG_AT_MAX_LEN_COMPAT 256
#define BTA_AG_CLIP_TYPE_DEFAULT_COMPAT 129

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

typedef enum {
    AUDIO_STOPPED,
    AUDIO_BRIDGE_SILENCE,
    AUDIO_I2S,
    AUDIO_SIMULATE,
} audio_mode_t;

typedef enum {
    CALL_IDLE,
    CALL_INCOMING,
    CALL_OUTGOING,
    CALL_ACTIVE,
    CALL_HELD,
    CALL_MERGED,
} call_mode_t;

typedef struct {
    bool in_use;
    call_mode_t mode;
    esp_hf_current_call_direction_t direction;
    char number[32];
} hfp_call_t;

typedef struct {
    bool pending;
    char chld[8];
    TickType_t started_tick;
    hfp_call_t snapshot[HFP_MAX_CALLS];
} hfp_call_control_t;

typedef struct {
    uint16_t type;
    uint16_t value;
} bta_ag_ind_compat_t;

typedef struct {
    char str[BTA_AG_AT_MAX_LEN_COMPAT + 1];
    bta_ag_ind_compat_t ind;
    uint16_t num;
    uint16_t audio_handle;
    uint16_t errcode;
    uint8_t ok_flag;
    uint8_t state;
} bta_ag_res_data_compat_t;

extern void BTA_AgResult(uint16_t handle, uint8_t result, bta_ag_res_data_compat_t *data);

typedef struct {
    esp_bd_addr_t bda;
    char name[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
    uint32_t cod;
    int8_t rssi;
    bool have_cod;
    bool have_rssi;
} discovered_device_t;

static i2s_chan_handle_t i2s_rx;
static rmt_channel_handle_t rgb_chan;
static rmt_encoder_handle_t rgb_encoder;
static volatile audio_mode_t audio_mode = AUDIO_STOPPED;
static volatile esp_avrc_playback_stat_t avrc_play_status = ESP_AVRC_PLAYBACK_STOPPED;
static volatile bool a2dp_connected;
static volatile bool a2dp_connecting;
static volatile bool a2dp_profile_ready;
static volatile bool bt_paired;
static esp_bd_addr_t remote_bda;
static esp_bd_addr_t saved_bda;
static bool have_remote_bda;
static bool have_saved_bda;
static bool discovery_in_progress;
static bool discovery_cancel_pending;
static bool connect_pending;
static bool delayed_connect_scheduled;
static bool bt_pairable;
static bool auto_reconnect_sdp_pending;
static bool auto_reconnect_paused;
static bool passive_acl_seen;
static uint32_t auto_reconnect_attempts;
static TickType_t auto_reconnect_not_before;
static esp_bd_addr_t pending_connect_bda;
static discovered_device_t discovered_devices[MAX_DISCOVERED_DEVICES];
static size_t discovered_device_count;
static uint32_t simulate_track;
static uint64_t simulate_phase;
static uint32_t simulate_silence_frames;
static uint32_t bridge_keepalive_lfsr = 0x12345678;
static audio_mode_t paused_audio_mode = AUDIO_STOPPED;
static uint32_t i2s_rx_bytes;
static uint32_t i2s_rx_short_count;
static uint32_t i2s_rx_report_next = 32768;
static uint32_t pcm_ring_overrun_count;
static uint32_t pcm_ring_overrun_bytes;
static uint32_t pcm_ring_underflow_count;
static uint32_t pcm_ring_underflow_bytes;
static uint16_t i2s_rx_peak;
static RingbufHandle_t pcm_ringbuf;
static int16_t pcm_underflow_frame[AUDIO_CHANNELS];
static bool pcm_underflow_frame_valid;
static bool avrc_tg_connected;
static bool avrc_rn_play_status_registered;
static bool avrc_rn_track_registered;
static bool avrc_rn_play_pos_registered;
static char bridge_media_title[256];
static char bridge_media_artist[128];
static uint32_t bridge_media_len_ms;
static uint32_t bridge_media_pos_ms;
static uint8_t bridge_track_index;
static bool bridge_track_change_pending;
static TickType_t bridge_snapshot_not_before;
static char a2dp_rate_status[192] = "A2DP_AUDIO_RATE:codec=none sample_rate=0 channels=unknown";
static char hfp_rate_status[96] = "HFP_HF_AUDIO_RATE:codec=none sample_rate=0 channels=mono frame=0";
static bool hfp_ag_connected;
static hfp_call_t hfp_calls[HFP_MAX_CALLS];
static hfp_call_control_t hfp_call_control;
static bool hfp_phone_clcc_batch_active;
static bool hfp_phone_clcc_seen[HFP_MAX_CALLS];
static bool hfp_phone_indicator_waiting_for_clcc;
static TickType_t hfp_phone_sync_suppress_until;
static volatile bool hfp_chld2_defer_pending;
static bool hfp_media_suspended;
static audio_mode_t hfp_media_resume_mode = AUDIO_STOPPED;
static esp_avrc_playback_stat_t hfp_media_resume_status = ESP_AVRC_PLAYBACK_STOPPED;
static bool hfp_media_resume_in_progress;
static bool hfp_ag_audio_connected;
static volatile bool hfp_outband_ring_task_running;
static uint32_t hfp_outband_ring_ticks;
static bool sco_pcm_gpio_active;
static bool a2dp_i2s_enabled;
static uint8_t rgb_pixel[3];

static void a2dp_i2s_disable_for_sco(void)
{
    if (!a2dp_i2s_enabled || i2s_rx == NULL) {
        return;
    }
    esp_err_t err = i2s_channel_disable(i2s_rx);
    if (err == ESP_OK) {
        a2dp_i2s_enabled = false;
        ESP_LOGI(TAG, "A2DP_I2S:disabled_for_sco");
    } else {
        ESP_LOGW(TAG, "A2DP_I2S_DISABLE_FAILED:0x%x", err);
    }
}

static void a2dp_i2s_enable_after_sco(void)
{
    if (a2dp_i2s_enabled || i2s_rx == NULL) {
        return;
    }
    esp_err_t err = i2s_channel_enable(i2s_rx);
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
    const uint64_t input_pins =
        (1ULL << I2S_BCLK_GPIO) |
        (1ULL << I2S_WS_GPIO) |
        (1ULL << I2S_DIN_GPIO);

    gpio_config_t in_conf = {
        .pin_bit_mask = input_pins,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_conf));

    esp_rom_gpio_connect_in_signal(I2S_BCLK_GPIO, I2S0I_BCK_IN_IDX, false);
    esp_rom_gpio_connect_in_signal(I2S_WS_GPIO, I2S0I_WS_IN_IDX, false);
    esp_rom_gpio_connect_in_signal(I2S_DIN_GPIO, I2S0I_DATA_IN15_IDX, false);
    sco_pcm_gpio_active = false;
    ESP_LOGI(TAG, "A2DP_I2S_GPIO:active bclk=%d ws=%d din=%d",
             I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DIN_GPIO);
#if A2DP_I2S_GATE_DURING_SCO
    a2dp_i2s_enable_after_sco();
#endif
}

static const int16_t *const simulated_track_samples[SIMULATED_TRACK_COUNT] = {
    voice_left_samples,
    voice_right_samples,
    voice_both_samples,
};

static const size_t simulated_track_counts[SIMULATED_TRACK_COUNT] = {
    voice_left_sample_count,
    voice_right_sample_count,
    voice_both_sample_count,
};

static void begin_a2dp_connect(const esp_bd_addr_t bda);
static bool hfp_media_should_block(void);

static void set_peer_reconnect_mode(void)
{
    esp_err_t err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    if (err == ESP_OK) {
        bt_pairable = true;
        ESP_LOGI(TAG, "BT_SCAN_MODE:CONNECTABLE_DISCOVERABLE pairable=1");
    } else {
        ESP_LOGW(TAG, "BT_SCAN_MODE_FAILED:connectable:0x%x", err);
    }
}

static void set_bonded_listen_mode(void)
{
    esp_err_t err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (err == ESP_OK) {
        bt_pairable = true;
        ESP_LOGI(TAG, "BT_SCAN_MODE:CONNECTABLE_NON_DISCOVERABLE pairable=1");
    } else {
        ESP_LOGW(TAG, "BT_SCAN_MODE_FAILED:bonded_listen:0x%x", err);
    }
}

static void set_private_mode(void)
{
    esp_err_t err = esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    if (err == ESP_OK) {
        bt_pairable = false;
        ESP_LOGI(TAG, "BT_SCAN_MODE:PRIVATE pairable=0");
    } else {
        ESP_LOGW(TAG, "BT_SCAN_MODE_FAILED:private:0x%x", err);
    }
}

static void schedule_pending_connect_after_gap_idle(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "CONNECT_DELAY:%d", POST_DISCOVERY_CONNECT_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(POST_DISCOVERY_CONNECT_DELAY_MS));
    delayed_connect_scheduled = false;
    if (connect_pending && !discovery_in_progress && !discovery_cancel_pending &&
        !a2dp_connecting && !a2dp_connected) {
        connect_pending = false;
        begin_a2dp_connect(pending_connect_bda);
    } else if (connect_pending) {
        ESP_LOGW(TAG,
                 "CONNECT_DELAY_BLOCKED:disc=%d cancel=%d connecting=%d connected=%d",
                 discovery_in_progress, discovery_cancel_pending,
                 a2dp_connecting, a2dp_connected);
    }
    vTaskDelete(NULL);
}

static void schedule_delayed_pending_connect(void)
{
    if (delayed_connect_scheduled) {
        return;
    }
    delayed_connect_scheduled = true;
    xTaskCreate(schedule_pending_connect_after_gap_idle, "gap_idle_connect",
                3072, NULL, tskIDLE_PRIORITY + 2, NULL);
}

static void schedule_auto_reconnect_backoff(const char *reason)
{
    uint32_t delay_ms = auto_reconnect_attempts * AUTO_RECONNECT_BACKOFF_STEP_MS;
    if (delay_ms > AUTO_RECONNECT_BACKOFF_MAX_MS) {
        delay_ms = AUTO_RECONNECT_BACKOFF_MAX_MS;
    }
    if (delay_ms == 0) {
        delay_ms = AUTO_RECONNECT_BACKOFF_STEP_MS;
    }

    auto_reconnect_not_before = xTaskGetTickCount() + pdMS_TO_TICKS(delay_ms);
    ESP_LOGI(TAG, "AUTO_RECONNECT_BACKOFF:%s:%" PRIu32 "ms", reason, delay_ms);
}

static void configure_phone_media_identity(void)
{
    esp_bt_cod_t cod = {
        .reserved_2 = 0,
        .minor = 0x03,
        .major = ESP_BT_COD_MAJOR_DEV_PHONE,
        .service = ESP_BT_COD_SRVC_CAPTURING |
                   ESP_BT_COD_SRVC_OBJ_TRANSFER |
                   ESP_BT_COD_SRVC_AUDIO |
                   ESP_BT_COD_SRVC_TELEPHONY,
        .reserved_8 = 0,
    };
    esp_err_t err = esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BT_COD:PHONE_MEDIA service=0x%03x major=0x%02x minor=0x%02x",
                 cod.service, cod.major, cod.minor);
    } else {
        ESP_LOGW(TAG, "BT_COD_FAILED:0x%x", err);
    }
}

static const char *simulated_track_title(uint32_t track)
{
    static const char *titles[SIMULATED_TRACK_COUNT] = {
        "Left",
        "Right",
        "Both",
    };
    return titles[track % SIMULATED_TRACK_COUNT];
}

static const char *audio_mode_name(audio_mode_t mode)
{
    switch (mode) {
    case AUDIO_BRIDGE_SILENCE:
        return "BRIDGE_SILENCE";
    case AUDIO_I2S:
        return "I2S";
    case AUDIO_SIMULATE:
        return "SIMULATE";
    case AUDIO_STOPPED:
    default:
        return "STOPPED";
    }
}

static uint32_t simulated_track_len_ms(uint32_t track)
{
    (void)track;
    return SIMULATED_TRACK_LEN_MS;
}

static size_t simulated_track_source_frames(void)
{
    return (size_t)((VOICE_SAMPLE_RATE_HZ * (uint64_t)SIMULATED_TRACK_LEN_MS) /
                    1000ULL);
}

static esp_avrc_playback_stat_t current_avrc_playback_status(void)
{
    return avrc_play_status;
}

uint8_t esp_avrc_tg_app_play_status(void)
{
    return (uint8_t)current_avrc_playback_status();
}

uint32_t esp_avrc_tg_app_song_len_ms(void)
{
    if (bridge_media_len_ms > 0) {
        return bridge_media_len_ms;
    }
    return simulated_track_len_ms(simulate_track);
}

uint32_t esp_avrc_tg_app_song_pos_ms(void)
{
    if (bridge_media_len_ms > 0) {
        return bridge_media_pos_ms;
    }
    if (avrc_play_status == ESP_AVRC_PLAYBACK_STOPPED) {
        return 0;
    }
    if (simulate_silence_frames > 0) {
        return SIMULATED_TRACK_LEN_MS;
    }
    size_t source_index = (size_t)(simulate_phase >> 32);
    size_t count = simulated_track_source_frames();
    if (source_index > count) {
        source_index = count;
    }
    return (uint32_t)((source_index * 1000ULL) / VOICE_SAMPLE_RATE_HZ);
}

uint8_t esp_avrc_tg_app_track_index(void)
{
    if (bridge_media_title[0] != '\0') {
        return bridge_track_index;
    }
    return (uint8_t)(simulate_track % SIMULATED_TRACK_COUNT);
}

const char *esp_avrc_tg_app_track_title(void)
{
    if (bridge_media_title[0] != '\0') {
        return bridge_media_title;
    }
    return simulated_track_title(simulate_track);
}

const char *esp_avrc_tg_app_track_artist(void)
{
    if (bridge_media_artist[0] != '\0') {
        return bridge_media_artist;
    }
    return "ESP32 A2DP";
}

static void notify_avrc_play_status_changed(void)
{
    if (!avrc_tg_connected || !avrc_rn_play_status_registered) {
        ESP_LOGI(TAG, "AVRCP_PLAY_STATUS_NOTIFY_SKIPPED:connected=%d registered=%d status=%d",
                 avrc_tg_connected, avrc_rn_play_status_registered,
                 current_avrc_playback_status());
        return;
    }

    esp_avrc_rn_param_t rn_param = {0};
    rn_param.playback = current_avrc_playback_status();
    if (esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_STATUS_CHANGE,
                                ESP_AVRC_RN_RSP_CHANGED,
                                &rn_param) == ESP_OK) {
        avrc_rn_play_status_registered = false;
        ESP_LOGI(TAG, "AVRCP_PLAY_STATUS_CHANGED:%d", rn_param.playback);
    } else {
        ESP_LOGW(TAG, "AVRCP_PLAY_STATUS_CHANGED_FAILED:%d", rn_param.playback);
    }
}

static void notify_avrc_track_changed(void)
{
    if (!avrc_tg_connected || !avrc_rn_track_registered) {
        return;
    }

    esp_avrc_rn_param_t rn_param = {0};
    rn_param.elm_id[7] = (uint8_t)(esp_avrc_tg_app_track_index() + 1);
    if (esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_TRACK_CHANGE,
                                ESP_AVRC_RN_RSP_CHANGED,
                                &rn_param) == ESP_OK) {
        avrc_rn_track_registered = false;
    }
}

static void notify_avrc_play_pos_changed(void)
{
    if (!avrc_tg_connected || !avrc_rn_play_pos_registered) {
        return;
    }

    esp_avrc_rn_param_t rn_param = {0};
    rn_param.play_pos = esp_avrc_tg_app_song_pos_ms();
    if (esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_POS_CHANGED,
                                ESP_AVRC_RN_RSP_CHANGED,
                                &rn_param) == ESP_OK) {
        avrc_rn_play_pos_registered = false;
        ESP_LOGI(TAG, "AVRCP_PLAY_POS_CHANGED:%" PRIu32, rn_param.play_pos);
    } else {
        ESP_LOGW(TAG, "AVRCP_PLAY_POS_CHANGED_FAILED:%" PRIu32, rn_param.play_pos);
    }
}

static void bridge_request_snapshot(const char *reason)
{
    TickType_t now = xTaskGetTickCount();
    if (bridge_snapshot_not_before != 0 &&
        (int32_t)(now - bridge_snapshot_not_before) < 0) {
        ESP_LOGI(TAG, "BRIDGE_SNAPSHOT_SKIPPED:%s", reason);
        return;
    }

    bridge_snapshot_not_before = now + pdMS_TO_TICKS(BRIDGE_SNAPSHOT_MIN_INTERVAL_MS);
    ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_SNAPSHOT");
}

static void __attribute__((unused)) select_simulated_track(uint32_t track, bool start_playback)
{
    simulate_track = track % SIMULATED_TRACK_COUNT;
    simulate_phase = 0;
    simulate_silence_frames = 0;
    ESP_LOGI(TAG, "SIM_TRACK:%" PRIu32 ":%s", simulate_track,
             simulated_track_title(simulate_track));
    notify_avrc_track_changed();
    if (start_playback) {
        const bool already_playing = audio_mode == AUDIO_SIMULATE &&
                                     avrc_play_status == ESP_AVRC_PLAYBACK_PLAYING;
        audio_mode = AUDIO_SIMULATE;
        avrc_play_status = ESP_AVRC_PLAYBACK_PLAYING;
        paused_audio_mode = AUDIO_STOPPED;
        if (a2dp_connected && !already_playing) {
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        }
        if (!already_playing) {
            notify_avrc_play_status_changed();
        }
        ESP_LOGI(TAG, "AUDIO_MODE:SIMULATE%s",
                 already_playing ? ":TRACK_CHANGED" : "");
    }
}

static void configure_avrcp_target_controls(void)
{
    esp_avrc_psth_bit_mask_t commands = {0};
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &commands,
                                     ESP_AVRC_PT_CMD_PLAY);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &commands,
                                     ESP_AVRC_PT_CMD_PAUSE);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &commands,
                                     ESP_AVRC_PT_CMD_STOP);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &commands,
                                     ESP_AVRC_PT_CMD_FORWARD);
    esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &commands,
                                     ESP_AVRC_PT_CMD_BACKWARD);
    esp_err_t err = esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD,
                                                    &commands);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "AVRCP_TG_COMMANDS:play,pause,stop,next,previous");
    } else {
        ESP_LOGW(TAG, "AVRCP_TG_COMMANDS_FAILED:0x%x", err);
    }

    esp_avrc_rn_evt_cap_mask_t rn_events = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &rn_events,
                                       ESP_AVRC_RN_PLAY_STATUS_CHANGE);
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &rn_events,
                                       ESP_AVRC_RN_TRACK_CHANGE);
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &rn_events,
                                       ESP_AVRC_RN_PLAY_POS_CHANGED);
    err = esp_avrc_tg_set_rn_evt_cap(&rn_events);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "AVRCP_TG_RN:play_status,track,play_pos");
    } else {
        ESP_LOGW(TAG, "AVRCP_TG_RN_FAILED:0x%x", err);
    }
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

static void print_bda(const uint8_t *bda, char *out, size_t out_len)
{
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static bool parse_bda(const char *text, esp_bd_addr_t bda)
{
    unsigned int bytes[ESP_BD_ADDR_LEN];
    if (sscanf(text, "%02x:%02x:%02x:%02x:%02x:%02x",
               &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4], &bytes[5]) != 6) {
        return false;
    }

    for (size_t i = 0; i < ESP_BD_ADDR_LEN; i++) {
        if (bytes[i] > 0xff) {
            return false;
        }
        bda[i] = (uint8_t)bytes[i];
    }
    return true;
}

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

static void save_last_bda(const esp_bd_addr_t bda)
{
    if (have_saved_bda && memcmp(saved_bda, bda, ESP_BD_ADDR_LEN) == 0) {
        return;
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        esp_err_t err = nvs_set_blob(nvs, NVS_LAST_BDA, bda, ESP_BD_ADDR_LEN);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
        if (err == ESP_OK) {
            memcpy(saved_bda, bda, ESP_BD_ADDR_LEN);
            have_saved_bda = true;
        } else {
            ESP_LOGW(TAG, "SAVED_TARGET_SAVE_FAILED:0x%x", err);
        }
    }
}

static void remove_all_bt_bonds(void)
{
    int bond_count = esp_bt_gap_get_bond_device_num();
    if (bond_count > 0) {
        esp_bd_addr_t bonds[MAX_DISCOVERED_DEVICES];
        int removable_count = bond_count > MAX_DISCOVERED_DEVICES ? MAX_DISCOVERED_DEVICES : bond_count;
        if (esp_bt_gap_get_bond_device_list(&removable_count, bonds) == ESP_OK) {
            for (int i = 0; i < removable_count; i++) {
                char bda_str[18];
                print_bda(bonds[i], bda_str, sizeof(bda_str));
                esp_err_t err = esp_bt_gap_remove_bond_device(bonds[i]);
                ESP_LOGI(TAG, "BT_BOND_REMOVE:%s:0x%x", bda_str, err);
            }
        }
    }
}

static bool log_bond_state(const char *reason)
{
    bool saved_match = false;
    int bond_count = esp_bt_gap_get_bond_device_num();
    ESP_LOGI(TAG, "BT_BOND_COUNT:%s:%d", reason, bond_count);

    if (bond_count > 0) {
        esp_bd_addr_t bonds[MAX_DISCOVERED_DEVICES];
        int listed_count = bond_count > MAX_DISCOVERED_DEVICES ? MAX_DISCOVERED_DEVICES : bond_count;
        esp_err_t err = esp_bt_gap_get_bond_device_list(&listed_count, bonds);
        if (err == ESP_OK) {
            for (int i = 0; i < listed_count; i++) {
                char bda_str[18];
                bool is_saved = have_saved_bda &&
                                memcmp(bonds[i], saved_bda, ESP_BD_ADDR_LEN) == 0;
                print_bda(bonds[i], bda_str, sizeof(bda_str));
                ESP_LOGI(TAG, "BT_BOND:%s:%d:%s saved=%d",
                         reason, i, bda_str, is_saved);
                saved_match = saved_match || is_saved;
            }
        } else {
            ESP_LOGW(TAG, "BT_BOND_LIST_FAILED:%s:0x%x", reason, err);
        }
    }

    ESP_LOGI(TAG, "BT_BOND_SAVED_MATCH:%s:%d", reason, saved_match);
    return saved_match;
}

static void clear_last_bda(bool remove_bonds)
{
    if (remove_bonds) {
        remove_all_bt_bonds();
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        esp_err_t err = nvs_erase_key(nvs, NVS_LAST_BDA);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "SAVED_TARGET_CLEAR_FAILED:0x%x", err);
        }
    }

    memset(saved_bda, 0, sizeof(saved_bda));
    memset(remote_bda, 0, sizeof(remote_bda));
    have_saved_bda = false;
    have_remote_bda = false;
    bt_paired = false;
    auto_reconnect_sdp_pending = false;
    auto_reconnect_paused = false;
    passive_acl_seen = false;
    auto_reconnect_attempts = 0;
    auto_reconnect_not_before = 0;
    ESP_LOGI(TAG, "SAVED_TARGET_CLEARED:bonds=%d", remove_bonds);
}

static void load_last_bda(void)
{
    char bda_str[18];
    nvs_handle_t nvs;
    size_t len = ESP_BD_ADDR_LEN;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        have_remote_bda = (nvs_get_blob(nvs, NVS_LAST_BDA, remote_bda, &len) == ESP_OK &&
                           len == ESP_BD_ADDR_LEN);
        if (have_remote_bda) {
            memcpy(saved_bda, remote_bda, ESP_BD_ADDR_LEN);
            have_saved_bda = true;
            print_bda(remote_bda, bda_str, sizeof(bda_str));
            ESP_LOGI(TAG, "SAVED_TARGET:%s", bda_str);
        }
        nvs_close(nvs);
    }
}

static bool refresh_a2dp_profile_ready(void)
{
    esp_a2d_profile_status_t status = {0};
    esp_err_t err = esp_a2d_get_profile_status(&status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "A2DP_PROFILE_STATUS_FAILED:0x%x", err);
        return false;
    }

    if (status.a2d_src_inited && !a2dp_profile_ready) {
        a2dp_profile_ready = true;
        ESP_LOGI(TAG, "A2DP_PROFILE:STATUS_READY conn_num=%u",
                 (unsigned int)status.conn_num);
    }
    return a2dp_profile_ready;
}

static void add_discovered_device(const esp_bd_addr_t bda, const char *name)
{
    char bda_str[18];
    print_bda(bda, bda_str, sizeof(bda_str));

    for (size_t i = 0; i < discovered_device_count; i++) {
        if (memcmp(discovered_devices[i].bda, bda, ESP_BD_ADDR_LEN) == 0) {
            if (name[0] && strcmp(discovered_devices[i].name, "unnamed") == 0) {
                strlcpy(discovered_devices[i].name, name,
                        sizeof(discovered_devices[i].name));
            }
            return;
        }
    }

    if (discovered_device_count < MAX_DISCOVERED_DEVICES) {
        memcpy(discovered_devices[discovered_device_count].bda, bda, ESP_BD_ADDR_LEN);
        strlcpy(discovered_devices[discovered_device_count].name,
                name[0] ? name : "unnamed",
                sizeof(discovered_devices[discovered_device_count].name));
        discovered_device_count++;
    }

    ESP_LOGI(TAG, "DEVICE:%s:%s", bda_str, name[0] ? name : "unnamed");
    ESP_LOGI(TAG, "DEVICE_NAME:%s|%s", name[0] ? name : "unnamed", bda_str);
}

static const discovered_device_t *find_discovered_device(const esp_bd_addr_t bda);

static bool update_discovered_device_meta(const esp_bd_addr_t bda,
                                          const char *name,
                                          uint32_t cod,
                                          bool have_cod,
                                          int8_t rssi,
                                          bool have_rssi)
{
    bool should_log = true;
    const discovered_device_t *before = find_discovered_device(bda);
    if (before != NULL) {
        bool resolves_name = name[0] && strcmp(name, "unnamed") != 0 &&
                             strcmp(before->name, "unnamed") == 0;
        should_log = resolves_name;
    }

    add_discovered_device(bda, name);

    for (size_t i = 0; i < discovered_device_count; i++) {
        if (memcmp(discovered_devices[i].bda, bda, ESP_BD_ADDR_LEN) == 0) {
            if (have_cod) {
                discovered_devices[i].cod = cod;
                discovered_devices[i].have_cod = true;
            }
            if (have_rssi &&
                (!discovered_devices[i].have_rssi || rssi > discovered_devices[i].rssi)) {
                discovered_devices[i].rssi = rssi;
                discovered_devices[i].have_rssi = true;
            }
            return should_log;
        }
    }
    return should_log;
}

static const discovered_device_t *find_discovered_device(const esp_bd_addr_t bda)
{
    for (size_t i = 0; i < discovered_device_count; i++) {
        if (memcmp(discovered_devices[i].bda, bda, ESP_BD_ADDR_LEN) == 0) {
            return &discovered_devices[i];
        }
    }
    return NULL;
}

static const char *cod_major_label(uint32_t cod)
{
    switch (esp_bt_gap_get_cod_major_dev(cod)) {
    case ESP_BT_COD_MAJOR_DEV_COMPUTER:
        return "computer";
    case ESP_BT_COD_MAJOR_DEV_PHONE:
        return "phone";
    case ESP_BT_COD_MAJOR_DEV_LAN_NAP:
        return "network";
    case ESP_BT_COD_MAJOR_DEV_AV:
        return "audio_video";
    case ESP_BT_COD_MAJOR_DEV_PERIPHERAL:
        return "peripheral";
    case ESP_BT_COD_MAJOR_DEV_IMAGING:
        return "imaging";
    case ESP_BT_COD_MAJOR_DEV_WEARABLE:
        return "wearable";
    case ESP_BT_COD_MAJOR_DEV_TOY:
        return "toy";
    case ESP_BT_COD_MAJOR_DEV_HEALTH:
        return "health";
    case ESP_BT_COD_MAJOR_DEV_UNCATEGORIZED:
        return "uncategorized";
    default:
        return "misc";
    }
}

static void copy_eir_name(uint8_t *eir, char *out, size_t out_len)
{
    uint8_t len = 0;
    uint8_t *name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
    if (name == NULL) {
        name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
    }
    if (name != NULL && out_len > 0) {
        size_t copy_len = len < (out_len - 1) ? len : (out_len - 1);
        memcpy(out, name, copy_len);
        out[copy_len] = '\0';
    }
}

static void log_discovery_result(esp_bt_gap_cb_param_t *param,
                                 const char *bd_name,
                                 const char *eir_name,
                                 uint32_t cod,
                                 bool have_cod,
                                 int8_t rssi,
                                 bool have_rssi)
{
    char bda_str[18];
    print_bda(param->disc_res.bda, bda_str, sizeof(bda_str));

    ESP_LOGI(TAG,
             "BT_FOUND:bda=%s bd_name=\"%s\" eir_name=\"%s\" cod=%s0x%06" PRIx32
             " major=%s service=0x%03x minor=0x%02x rssi=%s%d props=%d",
             bda_str,
             bd_name[0] ? bd_name : "",
             eir_name[0] ? eir_name : "",
             have_cod ? "" : "none/",
             have_cod ? cod : 0,
             have_cod ? cod_major_label(cod) : "unknown",
             have_cod ? esp_bt_gap_get_cod_srvc(cod) : 0,
             have_cod ? esp_bt_gap_get_cod_minor_dev(cod) : 0,
             have_rssi ? "" : "none/",
             have_rssi ? rssi : 0,
             param->disc_res.num_prop);
}

static void log_receiver_context(const char *prefix, const esp_bd_addr_t bda)
{
    char bda_str[18];
    print_bda(bda, bda_str, sizeof(bda_str));
    const discovered_device_t *device = find_discovered_device(bda);

    if (device == NULL) {
        ESP_LOGI(TAG, "%s:bda=%s name=\"unknown\" discovered=0", prefix, bda_str);
        return;
    }

    ESP_LOGI(TAG,
             "%s:bda=%s name=\"%s\" cod=%s0x%06" PRIx32 " major=%s rssi=%s%d",
             prefix,
             bda_str,
             device->name,
             device->have_cod ? "" : "none/",
             device->have_cod ? device->cod : 0,
             device->have_cod ? cod_major_label(device->cod) : "unknown",
             device->have_rssi ? "" : "none/",
             device->have_rssi ? device->rssi : 0);
}

static void start_discovery(bool clear_list)
{
    if (clear_list) {
        discovered_device_count = 0;
    }
    if (!discovery_in_progress) {
        esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
        if (err == ESP_OK) {
            discovery_in_progress = true;
            ESP_LOGI(TAG, "SCAN_STARTED");
        } else {
            ESP_LOGW(TAG, "SCAN_FAILED:0x%x", err);
        }
    } else {
        ESP_LOGI(TAG, "SCAN_ALREADY_ACTIVE");
    }
}

static void stop_discovery(void)
{
    if (!discovery_in_progress) {
        ESP_LOGI(TAG, "SCAN_ALREADY_STOPPED");
        return;
    }

    discovery_cancel_pending = true;
    esp_err_t err = esp_bt_gap_cancel_discovery();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SCAN_STOP_REQUESTED");
    } else {
        discovery_cancel_pending = false;
        ESP_LOGW(TAG, "SCAN_STOP_FAILED:0x%x", err);
    }
}

static void begin_a2dp_connect(const esp_bd_addr_t bda)
{
    char bda_str[18];
    print_bda(bda, bda_str, sizeof(bda_str));
    bool is_saved_peer = have_saved_bda &&
                         memcmp(bda, saved_bda, ESP_BD_ADDR_LEN) == 0;

    if (discovery_in_progress || discovery_cancel_pending) {
        memcpy(pending_connect_bda, bda, ESP_BD_ADDR_LEN);
        connect_pending = true;
        stop_discovery();
        ESP_LOGI(TAG, "CONNECT_PENDING:%s", bda_str);
        return;
    }

    log_receiver_context("CONNECT_TARGET", bda);

    memcpy(remote_bda, bda, ESP_BD_ADDR_LEN);
    have_remote_bda = true;

    if (!a2dp_profile_ready) {
        memcpy(pending_connect_bda, bda, ESP_BD_ADDR_LEN);
        connect_pending = true;
        ESP_LOGI(TAG, "CONNECT_WAIT_A2DP_READY:%s", bda_str);
        return;
    }

    if (a2dp_connected || a2dp_connecting) {
        ESP_LOGI(TAG, "CONNECT_SKIPPED:%s:already_active", bda_str);
        return;
    }

    if (is_saved_peer) {
        set_bonded_listen_mode();
    } else {
        set_private_mode();
    }
    a2dp_connecting = true;
    esp_err_t err = esp_a2d_source_connect(remote_bda);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CONNECTING:%s", bda_str);
    } else {
        a2dp_connecting = false;
        ESP_LOGW(TAG, "CONNECT_FAILED:%s:0x%x", bda_str, err);
    }
}

static esp_err_t i2s_input_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 256;
    chan_cfg.auto_clear = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &i2s_rx);
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
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(i2s_rx, &std_cfg);
    if (err != ESP_OK) {
        return err;
    }
    err = i2s_channel_enable(i2s_rx);
    if (err == ESP_OK) {
        a2dp_i2s_enabled = true;
    }
    return err;
}

static bool pcm_ringbuf_send_latest(const uint8_t *data, size_t len)
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
        pcm_ring_overrun_bytes += drop_len;
        pcm_ring_overrun_count++;
        vRingbufferReturnItem(pcm_ringbuf, drop);
    } else {
        pcm_ring_overrun_bytes += len;
        pcm_ring_overrun_count++;
        return false;
    }

    if (xRingbufferSend(pcm_ringbuf, data, len, 0) == pdTRUE) {
        return true;
    }

    pcm_ring_overrun_bytes += len;
    pcm_ring_overrun_count++;
    return false;
}

static void pcm_ringbuf_drain(void)
{
    if (pcm_ringbuf == NULL) {
        return;
    }

    pcm_underflow_frame_valid = false;
    for (;;) {
        size_t len = 0;
        void *item = xRingbufferReceive(pcm_ringbuf, &len, 0);
        if (item == NULL) {
            return;
        }
        vRingbufferReturnItem(pcm_ringbuf, item);
    }
}

static void pcm_update_underflow_frame(const uint8_t *data, size_t len)
{
    if (data == NULL || len < sizeof(pcm_underflow_frame)) {
        return;
    }

    const size_t frame_size = sizeof(pcm_underflow_frame);
    const size_t last_frame = ((len - frame_size) / frame_size) * frame_size;
    memcpy(pcm_underflow_frame, data + last_frame, frame_size);
    pcm_underflow_frame_valid = true;
}

static void pcm_fill_underflow(uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return;
    }

    if (!pcm_underflow_frame_valid) {
        memset(data, 0, len);
        return;
    }

    const size_t frame_size = sizeof(pcm_underflow_frame);
    size_t offset = 0;
    while (offset + frame_size <= len) {
        memcpy(data + offset, pcm_underflow_frame, frame_size);
        offset += frame_size;
    }
    if (offset < len) {
        memcpy(data + offset, pcm_underflow_frame, len - offset);
    }
}

static void i2s_rx_task(void *arg)
{
    (void)arg;
    uint8_t data[I2S_RX_TASK_CHUNK_BYTES];

    for (;;) {
        if (audio_mode != AUDIO_I2S) {
            pcm_ringbuf_drain();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(i2s_rx, data, sizeof(data), &bytes_read,
                                         pdMS_TO_TICKS(I2S_READ_TIMEOUT_MS));
        i2s_rx_bytes += bytes_read;

        uint16_t peak = 0;
        const int16_t *samples = (const int16_t *)data;
        const size_t sample_count = bytes_read / sizeof(int16_t);
        for (size_t i = 0; i < sample_count; i++) {
            int32_t sample = samples[i];
            uint16_t abs_sample = (uint16_t)(sample < 0 ? -sample : sample);
            if (abs_sample > peak) {
                peak = abs_sample;
            }
        }
        i2s_rx_peak = peak;

        bool report_short = false;
        if (err != ESP_OK || bytes_read < sizeof(data)) {
            i2s_rx_short_count++;
            report_short = i2s_rx_short_count <= 4 ||
                           (i2s_rx_short_count % I2S_RX_SHORT_REPORT_PERIOD) == 0;
        }

        if (bytes_read > 0 && !pcm_ringbuf_send_latest(data, bytes_read)) {
            ESP_LOGW(TAG, "PCM_RING_OVERRUN:count=%" PRIu32 " bytes=%" PRIu32,
                     pcm_ring_overrun_count, pcm_ring_overrun_bytes);
        }

        if (i2s_rx_bytes >= i2s_rx_report_next || report_short) {
            ESP_LOGI(TAG,
                     "I2S_RX:bytes=%" PRIu32 " short=%" PRIu32
                     " last=%u/%u err=0x%x peak=%u ring=%" PRIu32 "/%" PRIu32
                     " under=%" PRIu32 "/%" PRIu32,
                     i2s_rx_bytes, i2s_rx_short_count, (unsigned int)bytes_read,
                     (unsigned int)sizeof(data), err, i2s_rx_peak,
                     pcm_ring_overrun_count, pcm_ring_overrun_bytes,
                     pcm_ring_underflow_count, pcm_ring_underflow_bytes);
            i2s_rx_report_next = i2s_rx_bytes + 32768;
        }

        if (bytes_read == 0) {
            vTaskDelay(I2S_EMPTY_READ_DELAY_TICKS);
        }
    }
}

static void fill_simulated_audio(uint8_t *data, int32_t len)
{
    int16_t *samples = (int16_t *)data;
    const int32_t frames = len / (sizeof(int16_t) * AUDIO_CHANNELS);
    const uint64_t step = (((uint64_t)VOICE_SAMPLE_RATE_HZ) << 32) / SAMPLE_RATE;
    const uint32_t track = simulate_track % SIMULATED_TRACK_COUNT;
    const size_t track_frames = simulated_track_source_frames();

    for (int32_t frame = 0; frame < frames; frame++) {
        const size_t source_index = (size_t)(simulate_phase >> 32);
        int16_t sample = 0;

        if (simulate_silence_frames > 0) {
            simulate_silence_frames--;
        } else if (source_index < track_frames) {
            const size_t sample_index = source_index % simulated_track_counts[track];
            sample = simulated_track_samples[track][sample_index];
            simulate_phase += step;
        } else {
            simulate_phase = 0;
            simulate_silence_frames = SIMULATED_TRACK_GAP_FRAMES;
        }

        samples[(frame * AUDIO_CHANNELS) + 0] = sample;
        samples[(frame * AUDIO_CHANNELS) + 1] = sample;
    }
}

static void fill_bridge_keepalive_audio(uint8_t *data, int32_t len)
{
    int16_t *samples = (int16_t *)data;
    const int32_t frames = len / (sizeof(int16_t) * AUDIO_CHANNELS);

    for (int32_t frame = 0; frame < frames; frame++) {
        bridge_keepalive_lfsr = (bridge_keepalive_lfsr >> 1) ^
                                ((0u - (bridge_keepalive_lfsr & 1u)) & 0x80200003u);
        int16_t sample = (int16_t)((int32_t)(bridge_keepalive_lfsr & 0x1f) - 16);

        samples[(frame * AUDIO_CHANNELS) + 0] = sample;
        samples[(frame * AUDIO_CHANNELS) + 1] = sample;
    }
}

static int32_t a2dp_data_cb(uint8_t *data, int32_t len)
{
    if (data == NULL || len <= 0) {
        return 0;
    }

    if (audio_mode == AUDIO_STOPPED) {
        pcm_ringbuf_drain();
        memset(data, 0, len);
        return len;
    }

    if (audio_mode == AUDIO_BRIDGE_SILENCE) {
        pcm_ringbuf_drain();
        fill_bridge_keepalive_audio(data, len);
        return len;
    }

    if (audio_mode == AUDIO_SIMULATE) {
        pcm_ringbuf_drain();
        fill_simulated_audio(data, len);
        return len;
    }

    size_t copied = 0;
    while (copied < (size_t)len) {
        size_t item_len = 0;
        uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(pcm_ringbuf, &item_len, 0,
                                                          len - copied);
        if (item == NULL || item_len == 0) {
            break;
        }
        memcpy(data + copied, item, item_len);
        copied += item_len;
        vRingbufferReturnItem(pcm_ringbuf, item);
    }

    if (copied < (size_t)len) {
        const size_t missing = (size_t)len - copied;
        if (copied > 0) {
            pcm_update_underflow_frame(data, copied);
        }
        pcm_fill_underflow(data + copied, missing);
        pcm_ring_underflow_count++;
        pcm_ring_underflow_bytes += missing;
    }
    if (copied == (size_t)len) {
        pcm_update_underflow_frame(data, copied);
    }
    return len;
}

static void start_media(audio_mode_t mode)
{
    if (!hfp_media_resume_in_progress && hfp_media_should_block()) {
        ESP_LOGI(TAG, "AUDIO_MODE:%s:BLOCKED_HFP", audio_mode_name(mode));
        return;
    }

    const bool resume_simulated = mode == AUDIO_SIMULATE &&
                                  avrc_play_status == ESP_AVRC_PLAYBACK_PAUSED &&
                                  paused_audio_mode == AUDIO_SIMULATE;
    const bool already_playing = audio_mode == mode &&
                                 avrc_play_status == ESP_AVRC_PLAYBACK_PLAYING;

    if (already_playing) {
        ESP_LOGI(TAG, "AUDIO_MODE:%s:UNCHANGED", audio_mode_name(mode));
        return;
    }

    audio_mode = mode;
    avrc_play_status = ESP_AVRC_PLAYBACK_PLAYING;
    paused_audio_mode = AUDIO_STOPPED;
    if (mode == AUDIO_SIMULATE && !resume_simulated) {
        simulate_phase = 0;
        simulate_silence_frames = 0;
    }
    if (a2dp_connected) {
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
    }
    notify_avrc_play_status_changed();
    ESP_LOGI(TAG, "AUDIO_MODE:%s", audio_mode_name(mode));
}

static void set_media_paused(esp_avrc_playback_stat_t status)
{
    if (audio_mode == AUDIO_STOPPED && avrc_play_status == status) {
        ESP_LOGI(TAG, "AUDIO_MODE:%s:UNCHANGED",
                 status == ESP_AVRC_PLAYBACK_PAUSED ? "PAUSED" : "STOPPED");
        return;
    }

    if (status == ESP_AVRC_PLAYBACK_PAUSED) {
        paused_audio_mode = audio_mode;
    } else {
        paused_audio_mode = AUDIO_STOPPED;
        simulate_phase = 0;
        simulate_silence_frames = 0;
    }
    audio_mode = AUDIO_STOPPED;
    avrc_play_status = status;
    if (a2dp_connected) {
        esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
    }
    notify_avrc_play_status_changed();
    ESP_LOGI(TAG, "AUDIO_MODE:%s", status == ESP_AVRC_PLAYBACK_PAUSED ?
             "PAUSED" : "STOPPED");
}

static void __attribute__((unused)) pause_media(void)
{
    set_media_paused(ESP_AVRC_PLAYBACK_PAUSED);
}

static void __attribute__((unused)) stop_media(void)
{
    set_media_paused(ESP_AVRC_PLAYBACK_STOPPED);
}

static void set_bridge_play_status(esp_avrc_playback_stat_t status)
{
    if (status == ESP_AVRC_PLAYBACK_PLAYING && hfp_media_should_block()) {
        ESP_LOGI(TAG, "BRIDGE_PLAY_STATUS:%d:BLOCKED_HFP", status);
        return;
    }

    const audio_mode_t target_mode = status == ESP_AVRC_PLAYBACK_PLAYING ?
                                     AUDIO_I2S : AUDIO_STOPPED;

    if (audio_mode == target_mode && avrc_play_status == status) {
        return;
    }

    audio_mode = target_mode;
    paused_audio_mode = AUDIO_STOPPED;
    avrc_play_status = status;
    if (a2dp_connected) {
        esp_a2d_media_ctrl(status == ESP_AVRC_PLAYBACK_PLAYING ?
                           ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY :
                           ESP_A2D_MEDIA_CTRL_SUSPEND);
    }
    notify_avrc_play_status_changed();
    ESP_LOGI(TAG, "BRIDGE_PLAY_STATUS:%d audio=%s", status,
             audio_mode_name(audio_mode));
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        char discovered_name[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = {0};
        char eir_name[ESP_BT_GAP_MAX_BDNAME_LEN + 1] = {0};
        uint32_t cod = 0;
        int8_t rssi = 0;
        bool have_cod = false;
        bool have_rssi = false;

        for (int i = 0; i < param->disc_res.num_prop; i++) {
            esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
            if (prop->type == ESP_BT_GAP_DEV_PROP_BDNAME && prop->val != NULL) {
                size_t copy_len = prop->len < ESP_BT_GAP_MAX_BDNAME_LEN ?
                                  prop->len : ESP_BT_GAP_MAX_BDNAME_LEN;
                memcpy(discovered_name, prop->val, copy_len);
            } else if (prop->type == ESP_BT_GAP_DEV_PROP_COD && prop->val != NULL) {
                cod = *(uint32_t *)prop->val;
                have_cod = true;
            } else if (prop->type == ESP_BT_GAP_DEV_PROP_RSSI && prop->val != NULL) {
                rssi = *(int8_t *)prop->val;
                have_rssi = true;
            } else if (prop->type == ESP_BT_GAP_DEV_PROP_EIR && prop->val != NULL) {
                copy_eir_name((uint8_t *)prop->val, eir_name, sizeof(eir_name));
            }
        }

        const char *best_name = discovered_name[0] ? discovered_name : eir_name;
        bool log_result = update_discovered_device_meta(param->disc_res.bda, best_name,
                                                       cod, have_cod, rssi, have_rssi);
        if (log_result) {
            log_discovery_result(param, discovered_name, eir_name, cod, have_cod, rssi, have_rssi);
        }

        if (!have_remote_bda && strlen(BT_REMOTE_NAME) > 0 &&
            (strcmp(discovered_name, BT_REMOTE_NAME) == 0 || strcmp(eir_name, BT_REMOTE_NAME) == 0)) {
            memcpy(remote_bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
            have_remote_bda = true;
            ESP_LOGI(TAG, "DISCOVERED:%s",
                     discovered_name[0] ? discovered_name :
                     eir_name[0] ? eir_name : "unnamed");
            memcpy(pending_connect_bda, remote_bda, ESP_BD_ADDR_LEN);
            connect_pending = true;
            stop_discovery();
        }
        break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        discovery_in_progress = (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED);
        if (!discovery_in_progress) {
            discovery_cancel_pending = false;
        }
        ESP_LOGI(TAG, "SCAN:%s", discovery_in_progress ? "STARTED" : "STOPPED");
        if (!discovery_in_progress) {
            ESP_LOGI(TAG, "SCAN_DONE:%u", (unsigned int)discovered_device_count);
            if (connect_pending) {
                schedule_delayed_pending_connect();
            }
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT: {
        char bda_str[18];
        print_bda(param->pin_req.bda, bda_str, sizeof(bda_str));
        ESP_LOGW(TAG, "PIN_REQUIRED:%s:min16=%d", bda_str, param->pin_req.min_16_digit);
        break;
    }

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "SSP_CONFIRM:%06" PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_READ_REMOTE_NAME_EVT: {
        char bda_str[18];
        print_bda(param->read_rmt_name.bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "REMOTE_NAME:bda=%s status=0x%x name=\"%s\"",
                 bda_str, param->read_rmt_name.stat,
                 (const char *)param->read_rmt_name.rmt_name);
        break;
    }

    case ESP_BT_GAP_RMT_SRVCS_EVT: {
        char bda_str[18];
        print_bda(param->rmt_srvcs.bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "REMOTE_SERVICES:bda=%s status=0x%x count=%d",
                 bda_str, param->rmt_srvcs.stat, param->rmt_srvcs.num_uuids);
        for (int i = 0; i < param->rmt_srvcs.num_uuids; i++) {
            esp_bt_uuid_t *uuid = &param->rmt_srvcs.uuid_list[i];
            if (uuid->len == ESP_UUID_LEN_16) {
                ESP_LOGI(TAG, "REMOTE_SERVICE_UUID:%s:0x%04x", bda_str, uuid->uuid.uuid16);
            } else if (uuid->len == ESP_UUID_LEN_32) {
                ESP_LOGI(TAG, "REMOTE_SERVICE_UUID:%s:0x%08" PRIx32, bda_str, uuid->uuid.uuid32);
            } else {
                ESP_LOGI(TAG, "REMOTE_SERVICE_UUID:%s:len=%u", bda_str, uuid->len);
            }
        }

        if (auto_reconnect_sdp_pending && have_saved_bda &&
            memcmp(param->rmt_srvcs.bda, saved_bda, ESP_BD_ADDR_LEN) == 0) {
            auto_reconnect_sdp_pending = false;
            if (param->rmt_srvcs.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "AUTO_RECONNECT_SDP_OK:%s", bda_str);
                memcpy(remote_bda, saved_bda, ESP_BD_ADDR_LEN);
                begin_a2dp_connect(remote_bda);
            } else {
                ESP_LOGW(TAG, "AUTO_RECONNECT_SDP_FAILED:%s:0x%x",
                         bda_str, param->rmt_srvcs.stat);
                schedule_auto_reconnect_backoff("sdp_failed");
            }
        }
        break;
    }

    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "PAIRING_OK:%s", param->auth_cmpl.device_name);
            memcpy(remote_bda, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            have_remote_bda = true;
            bt_paired = true;
            save_last_bda(remote_bda);
        } else {
            ESP_LOGW(TAG, "PAIRING_FAIL:0x%x", param->auth_cmpl.stat);
        }
        break;

    case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT: {
        char bda_str[18];
        print_bda(param->remove_bond_dev_cmpl.bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "BT_BOND_REMOVED:%s status=0x%x",
                 bda_str, param->remove_bond_dev_cmpl.status);
        break;
    }

    case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
        ESP_LOGI(TAG, "BT_EIR_CONFIG:status=0x%x types=%u",
                 param->config_eir_data.stat,
                 (unsigned int)param->config_eir_data.eir_type_num);
        break;

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT: {
        char bda_str[18];
        print_bda(param->acl_conn_cmpl_stat.bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "ACL_CONNECTED:bda=%s status=0x%x handle=%u",
                 bda_str, param->acl_conn_cmpl_stat.stat,
                 param->acl_conn_cmpl_stat.handle);
        if (param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS &&
            have_saved_bda &&
            memcmp(param->acl_conn_cmpl_stat.bda, saved_bda, ESP_BD_ADDR_LEN) == 0) {
            passive_acl_seen = true;
        }
        break;
    }

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT: {
        char bda_str[18];
        print_bda(param->acl_disconn_cmpl_stat.bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "ACL_DISCONNECTED:bda=%s reason=0x%x handle=%u",
                 bda_str, param->acl_disconn_cmpl_stat.reason,
                 param->acl_disconn_cmpl_stat.handle);
        if (passive_acl_seen && have_saved_bda &&
            memcmp(param->acl_disconn_cmpl_stat.bda, saved_bda, ESP_BD_ADDR_LEN) == 0 &&
            !a2dp_connected && !a2dp_connecting) {
            passive_acl_seen = false;
            auto_reconnect_paused = false;
            auto_reconnect_not_before = xTaskGetTickCount() +
                                        pdMS_TO_TICKS(PASSIVE_ACL_RECONNECT_DELAY_MS);
            ESP_LOGI(TAG, "AUTO_RECONNECT_AFTER_PASSIVE_ACL:%dms",
                     PASSIVE_ACL_RECONNECT_DELAY_MS);
        }
        break;
    }

    default:
        break;
    }
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        char bda_str[18];
        print_bda(param->conn_stat.remote_bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "A2DP_CONNECTION:%d:%s handle=%" PRIu32 " mtu=%u disc_rsn=%d",
                 param->conn_stat.state, bda_str, param->conn_stat.conn_hdl,
                 param->conn_stat.audio_mtu, param->conn_stat.disc_rsn);
        log_receiver_context("A2DP_RECEIVER", param->conn_stat.remote_bda);
        a2dp_connected = (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED);
        a2dp_connecting = (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING);
        if (a2dp_connecting || a2dp_connected) {
            auto_reconnect_paused = true;
            auto_reconnect_sdp_pending = false;
            auto_reconnect_not_before = 0;
            passive_acl_seen = false;
        }
        if (a2dp_connected) {
            a2dp_connecting = false;
            bt_paired = true;
            auto_reconnect_attempts = 0;
            set_private_mode();
            save_last_bda(param->conn_stat.remote_bda);
            memcpy(remote_bda, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            have_remote_bda = true;
            if (audio_mode != AUDIO_STOPPED) {
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
            }
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            a2dp_connecting = false;
            auto_reconnect_sdp_pending = false;
            if (have_remote_bda) {
                if (have_saved_bda && memcmp(remote_bda, saved_bda, ESP_BD_ADDR_LEN) == 0 &&
                    auto_reconnect_attempts > 0) {
                    set_bonded_listen_mode();
                    schedule_auto_reconnect_backoff("a2dp_disconnected");
                } else {
                    set_peer_reconnect_mode();
                }
            }
        }
        break;
    }

    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP_AUDIO:%d", param->audio_stat.state);
        break;

    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        ESP_LOGI(TAG, "A2DP_MEDIA_ACK:cmd=%d status=%d",
                 param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
        if (param->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY &&
            param->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS &&
            audio_mode != AUDIO_STOPPED) {
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
        }
        break;

    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(TAG, "A2DP_AUDIO_CFG:handle=%" PRIu32 " codec=%d",
                 param->audio_cfg.conn_hdl, param->audio_cfg.mcc.type);
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
        a2dp_profile_ready = (param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS);
        ESP_LOGI(TAG, "A2DP_PROFILE:%s",
                 a2dp_profile_ready ? "INIT_SUCCESS" : "DEINIT_SUCCESS");
        if (a2dp_profile_ready && connect_pending && !discovery_in_progress &&
            !discovery_cancel_pending && !delayed_connect_scheduled) {
            schedule_delayed_pending_connect();
        }
        break;

    default:
        ESP_LOGI(TAG, "A2DP_EVENT:%d", event);
        break;
    }
}

static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        char bda_str[18];
        print_bda(param->conn_stat.remote_bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "AVRCP_CT_CONNECTION:connected=%d bda=%s",
                 param->conn_stat.connected, bda_str);
        break;
    }

    case ESP_AVRC_CT_PROF_STATE_EVT:
        ESP_LOGI(TAG, "AVRCP_CT_PROFILE:state=%d", param->avrc_ct_init_stat.state);
        break;

    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        char bda_str[18];
        print_bda(param->rmt_feats.remote_bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "AVRCP_CT_FEATURES:bda=%s ct=0x%" PRIx32 " tg=0x%x",
                 bda_str, param->rmt_feats.feat_mask, param->rmt_feats.tg_feat_flag);
        break;
    }

    default:
        ESP_LOGI(TAG, "AVRCP_CT_EVENT:%d", event);
        break;
    }
}

static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT: {
        char bda_str[18];
        print_bda(param->conn_stat.remote_bda, bda_str, sizeof(bda_str));
        avrc_tg_connected = param->conn_stat.connected;
        if (!avrc_tg_connected) {
            avrc_rn_play_status_registered = false;
            avrc_rn_track_registered = false;
            avrc_rn_play_pos_registered = false;
        }
        ESP_LOGI(TAG, "AVRCP_TG_CONNECTION:connected=%d bda=%s",
                 param->conn_stat.connected, bda_str);
        break;
    }

    case ESP_AVRC_TG_PROF_STATE_EVT:
        ESP_LOGI(TAG, "AVRCP_TG_PROFILE:state=%d",
                 param->avrc_tg_init_stat.state);
        break;

    case ESP_AVRC_TG_REMOTE_FEATURES_EVT: {
        char bda_str[18];
        print_bda(param->rmt_feats.remote_bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "AVRCP_TG_FEATURES:bda=%s ct=0x%" PRIx32 " flag=0x%x",
                 bda_str, param->rmt_feats.feat_mask,
                 param->rmt_feats.ct_feat_flag);
        break;
    }

    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
        ESP_LOGI(TAG,
                 "AVRCP_CMD:key=0x%02x state=%d track=%" PRIu32 " title=%s status=%d mode=%s",
                 param->psth_cmd.key_code, param->psth_cmd.key_state,
                 simulate_track, esp_avrc_tg_app_track_title(),
                 avrc_play_status, audio_mode_name(audio_mode));
        if (param->psth_cmd.key_state != ESP_AVRC_PT_CMD_STATE_PRESSED) {
            break;
        }
        switch (param->psth_cmd.key_code) {
        case ESP_AVRC_PT_CMD_PLAY:
            ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_PLAY");
            bridge_request_snapshot("avrcp");
            start_media(AUDIO_I2S);
            break;
        case ESP_AVRC_PT_CMD_PAUSE:
            ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_PAUSE");
            bridge_request_snapshot("avrcp");
            set_media_paused(ESP_AVRC_PLAYBACK_PAUSED);
            break;
        case ESP_AVRC_PT_CMD_STOP:
            ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_STOP");
            bridge_request_snapshot("avrcp");
            set_media_paused(ESP_AVRC_PLAYBACK_STOPPED);
            break;
        case ESP_AVRC_PT_CMD_FORWARD:
            ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_NEXT");
            bridge_request_snapshot("avrcp");
            break;
        case ESP_AVRC_PT_CMD_BACKWARD:
            ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_PREVIOUS");
            bridge_request_snapshot("avrcp");
            break;
        default:
            ESP_LOGI(TAG, "AVRCP_CMD_IGNORED:0x%02x",
                     param->psth_cmd.key_code);
            break;
        }
        break;

    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
        esp_avrc_rn_param_t rn_param = {0};
        ESP_LOGI(TAG, "AVRCP_REGISTER_NOTIFICATION:event=0x%02x param=%" PRIu32,
                 param->reg_ntf.event_id, param->reg_ntf.event_parameter);
        if (param->reg_ntf.event_id == ESP_AVRC_RN_PLAY_STATUS_CHANGE) {
            rn_param.playback = current_avrc_playback_status();
            ESP_LOGI(TAG, "AVRCP_PLAY_STATUS_INTERIM:%d", rn_param.playback);
            if (esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_STATUS_CHANGE,
                                        ESP_AVRC_RN_RSP_INTERIM,
                                        &rn_param) == ESP_OK) {
                avrc_rn_play_status_registered = true;
            }
        } else if (param->reg_ntf.event_id == ESP_AVRC_RN_TRACK_CHANGE) {
            rn_param.elm_id[7] = (uint8_t)(esp_avrc_tg_app_track_index() + 1);
            ESP_LOGI(TAG, "AVRCP_TRACK_INTERIM:%u title=%s",
                     rn_param.elm_id[7], esp_avrc_tg_app_track_title());
            if (esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_TRACK_CHANGE,
                                        ESP_AVRC_RN_RSP_INTERIM,
                                        &rn_param) == ESP_OK) {
                avrc_rn_track_registered = true;
            }
        } else if (param->reg_ntf.event_id == ESP_AVRC_RN_PLAY_POS_CHANGED) {
            rn_param.play_pos = esp_avrc_tg_app_song_pos_ms();
            ESP_LOGI(TAG, "AVRCP_PLAY_POS_INTERIM:%" PRIu32, rn_param.play_pos);
            if (esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_POS_CHANGED,
                                        ESP_AVRC_RN_RSP_INTERIM,
                                        &rn_param) == ESP_OK) {
                avrc_rn_play_pos_registered = true;
            }
        }
        break;
    }

    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        ESP_LOGI(TAG, "AVRCP_SET_VOLUME:%u", param->set_abs_vol.volume);
        break;

    default:
        ESP_LOGI(TAG, "AVRCP_TG_EVENT:%d", event);
        break;
    }
}

static bool hfp_mode_is_active(call_mode_t mode)
{
    return mode == CALL_ACTIVE || mode == CALL_MERGED;
}

static bool hfp_mode_is_held(call_mode_t mode)
{
    return mode == CALL_HELD;
}

static bool hfp_mode_is_established(call_mode_t mode)
{
    return hfp_mode_is_active(mode) || hfp_mode_is_held(mode);
}

static bool hfp_has_mode(call_mode_t mode)
{
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && hfp_calls[i].mode == mode) {
            return true;
        }
    }
    return false;
}

static bool hfp_has_established_call(void)
{
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && hfp_mode_is_established(hfp_calls[i].mode)) {
            return true;
        }
    }
    return false;
}

static int hfp_num_established(void)
{
    int count = 0;
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && hfp_mode_is_established(hfp_calls[i].mode)) {
            count++;
        }
    }
    return count;
}

static int hfp_call_count(void)
{
    int count = 0;
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use) {
            count++;
        }
    }
    return count;
}

static bool hfp_media_should_block(void)
{
    return hfp_call_count() > 0;
}

static int hfp_num_active(void)
{
    int count = 0;
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && hfp_mode_is_active(hfp_calls[i].mode)) {
            count++;
        }
    }
    return count;
}

static int hfp_num_held(void)
{
    int count = 0;
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && hfp_mode_is_held(hfp_calls[i].mode)) {
            count++;
        }
    }
    return count;
}

static bool hfp_single_held_call(void)
{
    return hfp_call_count() == 1 && hfp_num_active() == 0 && hfp_num_held() == 1;
}

static int hfp_report_num_active(void)
{
    return hfp_single_held_call() ? 1 : hfp_num_active();
}

static int hfp_report_num_held(void)
{
    return hfp_single_held_call() ? 0 : hfp_num_held();
}

static esp_hf_call_status_t hfp_call_status(void)
{
    return hfp_has_established_call() ?
           ESP_HF_CALL_STATUS_CALL_IN_PROGRESS : ESP_HF_CALL_STATUS_NO_CALLS;
}

static esp_hf_call_setup_status_t hfp_call_setup_status(void)
{
    if (hfp_has_mode(CALL_INCOMING)) {
        return ESP_HF_CALL_SETUP_STATUS_INCOMING;
    }
    if (hfp_has_mode(CALL_OUTGOING)) {
        return ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING;
    }
    return ESP_HF_CALL_SETUP_STATUS_IDLE;
}

static esp_hf_call_held_status_t hfp_call_held_status(void)
{
    const bool active = hfp_report_num_active() > 0;
    const bool held = hfp_report_num_held() > 0;

    if (active && held) {
        return ESP_HF_CALL_HELD_STATUS_HELD_AND_ACTIVE;
    }
    if (held) {
        return ESP_HF_CALL_HELD_STATUS_HELD;
    }
    return ESP_HF_CALL_HELD_STATUS_NONE;
}

static esp_hf_current_call_status_t hfp_current_call_status(call_mode_t mode,
                                                            bool waiting)
{
    if (mode == CALL_HELD && hfp_single_held_call()) {
        return ESP_HF_CURRENT_CALL_STATUS_ACTIVE;
    }

    switch (mode) {
    case CALL_INCOMING:
        return waiting ? ESP_HF_CURRENT_CALL_STATUS_WAITING :
               ESP_HF_CURRENT_CALL_STATUS_INCOMING;
    case CALL_OUTGOING:
        return ESP_HF_CURRENT_CALL_STATUS_DIALING;
    case CALL_ACTIVE:
    case CALL_MERGED:
        return ESP_HF_CURRENT_CALL_STATUS_ACTIVE;
    case CALL_HELD:
        return ESP_HF_CURRENT_CALL_STATUS_HELD;
    default:
        return ESP_HF_CURRENT_CALL_STATUS_ACTIVE;
    }
}

static esp_hf_current_call_mpty_type_t hfp_current_call_mpty_type(call_mode_t mode)
{
    return mode == CALL_MERGED ? ESP_HF_CURRENT_CALL_MPTY_TYPE_MULTI :
           ESP_HF_CURRENT_CALL_MPTY_TYPE_SINGLE;
}

static bool hfp_call_ready(void)
{
    if (!hfp_ag_connected || !have_remote_bda) {
        ESP_LOGW(TAG, "HFP_AG_CALL_IGNORED:connected=%d have_bda=%d",
                 hfp_ag_connected, have_remote_bda);
        return false;
    }
    return true;
}

static hfp_call_t *hfp_find_call_by_index(int index)
{
    if (index < 1 || index > HFP_MAX_CALLS) {
        return NULL;
    }
    hfp_call_t *call = &hfp_calls[index - 1];
    return call->in_use ? call : NULL;
}

static hfp_call_t *hfp_get_call_slot_by_index(int index)
{
    if (index < 1 || index > HFP_MAX_CALLS) {
        return NULL;
    }
    return &hfp_calls[index - 1];
}

static hfp_call_t *hfp_find_call_by_number(const char *number)
{
    if (number == NULL || number[0] == '\0') {
        return NULL;
    }
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && strcmp(hfp_calls[i].number, number) == 0) {
            return &hfp_calls[i];
        }
    }
    return NULL;
}

static void hfp_suppress_phone_sync(uint32_t ms)
{
    hfp_phone_sync_suppress_until = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
}

static bool hfp_phone_sync_suppressed(void)
{
    if (hfp_phone_sync_suppress_until == 0) {
        return false;
    }
    TickType_t now = xTaskGetTickCount();
    if ((int32_t)(hfp_phone_sync_suppress_until - now) > 0) {
        return true;
    }
    hfp_phone_sync_suppress_until = 0;
    return false;
}

static hfp_call_t *hfp_find_first_mode(call_mode_t mode)
{
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && hfp_calls[i].mode == mode) {
            return &hfp_calls[i];
        }
    }
    return NULL;
}

static hfp_call_t *hfp_find_last_mode(call_mode_t mode)
{
    for (int i = HFP_MAX_CALLS - 1; i >= 0; i--) {
        if (hfp_calls[i].in_use && hfp_calls[i].mode == mode) {
            return &hfp_calls[i];
        }
    }
    return NULL;
}

static hfp_call_t *hfp_find_report_call(void)
{
    hfp_call_t *call = hfp_find_last_mode(CALL_INCOMING);
    if (call != NULL) {
        return call;
    }
    call = hfp_find_first_mode(CALL_ACTIVE);
    if (call != NULL) {
        return call;
    }
    call = hfp_find_first_mode(CALL_MERGED);
    if (call != NULL) {
        return call;
    }
    call = hfp_find_first_mode(CALL_HELD);
    if (call != NULL) {
        return call;
    }
    call = hfp_find_first_mode(CALL_OUTGOING);
    if (call != NULL) {
        return call;
    }
    return NULL;
}

static char *hfp_report_number(void)
{
    hfp_call_t *call = hfp_find_report_call();
    if (call != NULL) {
        return call->number;
    }
    return NULL;
}

static void hfp_clear_call(hfp_call_t *call)
{
    if (call == NULL) {
        return;
    }
    memset(call, 0, sizeof(*call));
}

static void hfp_call_control_clear(const char *reason)
{
    if (!hfp_call_control.pending) {
        return;
    }
    ESP_LOGI(TAG, "HFP_AG_CALL_CONTROL_DONE:%s cmd=%s",
             reason, hfp_call_control.chld);
    memset(&hfp_call_control, 0, sizeof(hfp_call_control));
}

static int hfp_call_control_accept(const char *chld)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(HFP_CALL_CONTROL_TIMEOUT_MS);

    if (hfp_call_control.pending &&
        (now - hfp_call_control.started_tick) >= timeout) {
        hfp_call_control_clear("timeout");
    }

    if (hfp_call_control.pending) {
        if (strcmp(chld, hfp_call_control.chld) == 0) {
            ESP_LOGI(TAG, "HFP_AG_CALL_CONTROL_COALESCED:%s", chld);
        } else if (strcmp(hfp_call_control.chld, "2") == 0 &&
                   strcmp(chld, "1") == 0) {
            memcpy(hfp_calls, hfp_call_control.snapshot, sizeof(hfp_calls));
            hfp_call_control.started_tick = now;
            snprintf(hfp_call_control.chld, sizeof(hfp_call_control.chld), "%s", chld);
            ESP_LOGI(TAG, "HFP_AG_CALL_CONTROL_OVERRIDE:2->1");
            return 1;
        } else {
            ESP_LOGI(TAG, "HFP_AG_CALL_CONTROL_BUSY_IGNORED:pending=%s new=%s",
                     hfp_call_control.chld, chld);
        }
        return 0;
    }

    hfp_call_control.pending = true;
    hfp_call_control.started_tick = now;
    memcpy(hfp_call_control.snapshot, hfp_calls, sizeof(hfp_calls));
    snprintf(hfp_call_control.chld, sizeof(hfp_call_control.chld), "%s", chld);
    ESP_LOGI(TAG, "HFP_AG_CALL_CONTROL_BEGIN:%s", hfp_call_control.chld);
    return 1;
}

static void hfp_reset_calls(void)
{
    memset(hfp_calls, 0, sizeof(hfp_calls));
    memset(&hfp_call_control, 0, sizeof(hfp_call_control));
    hfp_phone_clcc_batch_active = false;
    memset(hfp_phone_clcc_seen, 0, sizeof(hfp_phone_clcc_seen));
    hfp_phone_indicator_waiting_for_clcc = false;
    hfp_phone_sync_suppress_until = 0;
    hfp_chld2_defer_pending = false;
}

static void hfp_update_media_policy(const char *source)
{
    const int calls = hfp_call_count();

    if (calls > 0) {
        if (!hfp_media_suspended) {
            hfp_media_suspended = true;
            hfp_media_resume_mode = audio_mode;
            hfp_media_resume_status = avrc_play_status;
            ESP_LOGI(TAG, "HFP_MEDIA_SUSPEND:%s calls=%d resume_mode=%s resume_status=%d",
                     source ? source : "unknown", calls,
                     audio_mode_name(hfp_media_resume_mode),
                     hfp_media_resume_status);
        }

        if (audio_mode != AUDIO_STOPPED || avrc_play_status == ESP_AVRC_PLAYBACK_PLAYING) {
            set_media_paused(ESP_AVRC_PLAYBACK_PAUSED);
        } else if (a2dp_connected) {
            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
        }
        return;
    }

    if (!hfp_media_suspended) {
        return;
    }

    audio_mode_t resume_mode = hfp_media_resume_mode;
    esp_avrc_playback_stat_t resume_status = hfp_media_resume_status;
    hfp_media_suspended = false;
    hfp_media_resume_mode = AUDIO_STOPPED;
    hfp_media_resume_status = ESP_AVRC_PLAYBACK_STOPPED;

    ESP_LOGI(TAG, "HFP_MEDIA_RESUME:%s resume_mode=%s resume_status=%d",
             source ? source : "unknown", audio_mode_name(resume_mode),
             resume_status);

    if (resume_status == ESP_AVRC_PLAYBACK_PLAYING && resume_mode != AUDIO_STOPPED) {
        hfp_media_resume_in_progress = true;
        start_media(resume_mode);
        hfp_media_resume_in_progress = false;
    }
}

static hfp_call_t *hfp_add_call(call_mode_t mode,
                                esp_hf_current_call_direction_t direction,
                                const char *number)
{
    hfp_call_t *existing = hfp_find_call_by_number(number);
    if (existing != NULL && existing->mode == mode) {
        ESP_LOGI(TAG, "HFP_AG_CALL_DUPLICATE:mode=%d num=\"%s\"", mode, existing->number);
        return existing;
    }

    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (!hfp_calls[i].in_use) {
            hfp_calls[i].in_use = true;
            hfp_calls[i].mode = mode;
            hfp_calls[i].direction = direction;
            if (number != NULL && number[0] != '\0') {
                snprintf(hfp_calls[i].number, sizeof(hfp_calls[i].number), "%s", number);
            } else {
                snprintf(hfp_calls[i].number, sizeof(hfp_calls[i].number),
                         "555123456%u", (unsigned int)(i + 1));
            }
            return &hfp_calls[i];
        }
    }
    ESP_LOGW(TAG, "HFP_AG_CALL_TABLE_FULL:max=%d", HFP_MAX_CALLS);
    return NULL;
}

static void hfp_log_call_state(const char *source)
{
    ESP_LOGI(TAG, "HFP_AG_CALL:%s count=%d active=%d held=%d setup=%d",
             source, hfp_call_count(), hfp_num_active(), hfp_num_held(),
             hfp_call_setup_status());
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (!hfp_calls[i].in_use) {
            continue;
        }
        ESP_LOGI(TAG, "HFP_AG_CALL_SLOT:%u mode=%d dir=%d mpty=%d num=\"%s\"",
                 (unsigned int)(i + 1), hfp_calls[i].mode, hfp_calls[i].direction,
                 hfp_current_call_mpty_type(hfp_calls[i].mode), hfp_calls[i].number);
    }
    hfp_update_media_policy(source);
}

static void hfp_report_indicators(void)
{
    if (!hfp_ag_connected || !have_remote_bda) {
        return;
    }
    esp_hf_ag_ciev_report(remote_bda, ESP_HF_IND_TYPE_CALL, hfp_call_status());
    esp_hf_ag_ciev_report(remote_bda, ESP_HF_IND_TYPE_CALLSETUP, hfp_call_setup_status());
    esp_hf_ag_ciev_report(remote_bda, ESP_HF_IND_TYPE_CALLHELD, hfp_call_held_status());
}

static void hfp_ag_set_inband_ring(bool provided)
{
    if (!hfp_ag_connected || !have_remote_bda) {
        return;
    }
    esp_hf_ag_bsir(remote_bda, provided ? ESP_HF_IN_BAND_RINGTONE_PROVIDED :
                   ESP_HF_IN_BAND_RINGTONE_NOT_PROVIDED);
    ESP_LOGI(TAG, "HFP_AG_BSIR:%s", provided ? "provided" : "not_provided");
}

static bool hfp_should_outband_ring(void)
{
    return hfp_ag_connected && have_remote_bda && hfp_has_mode(CALL_INCOMING);
}

static esp_err_t hfp_ag_send_outband_ring_state(const char *reason)
{
    if (!hfp_should_outband_ring()) {
        return ESP_ERR_INVALID_STATE;
    }

    char *number = hfp_report_number();
    hfp_ag_set_inband_ring(false);
    esp_err_t err = esp_hf_ag_answer_call(remote_bda,
                                          hfp_report_num_active(),
                                          hfp_report_num_held(),
                                          hfp_call_status(),
                                          hfp_call_setup_status(),
                                          number,
                                          ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
    bta_ag_res_data_compat_t ring = {0};
    if (number != NULL && number[0] != '\0') {
        snprintf(ring.str, sizeof(ring.str), "\"%s\"", number);
    }
    ring.num = BTA_AG_CLIP_TYPE_DEFAULT_COMPAT;
    BTA_AgResult(BTA_AG_HANDLE_ALL_COMPAT, BTA_AG_IN_CALL_RES_COMPAT, &ring);
    hfp_report_indicators();
    hfp_outband_ring_ticks++;
    ESP_LOGI(TAG,
             "HFP_AG_OUTBAND_RING:%s tick=%" PRIu32 " err=0x%x forced=1 active=%d held=%d setup=%d num=\"%s\"",
             reason ? reason : "tick", hfp_outband_ring_ticks, err,
             hfp_report_num_active(), hfp_report_num_held(),
             hfp_call_setup_status(), number ? number : "");
    return err;
}

static void hfp_outband_ring_task(void *arg)
{
    const char *reason = (const char *)arg;
    ESP_LOGI(TAG, "HFP_AG_OUTBAND_RING_TASK:start reason=%s", reason ? reason : "unknown");

    while (hfp_should_outband_ring()) {
        hfp_ag_send_outband_ring_state(reason);
        vTaskDelay(pdMS_TO_TICKS(HFP_OUTBAND_RING_REPEAT_MS));
        reason = "repeat";
    }

    hfp_outband_ring_task_running = false;
    ESP_LOGI(TAG, "HFP_AG_OUTBAND_RING_TASK:stop");
    vTaskDelete(NULL);
}

static void hfp_start_outband_ring(const char *reason)
{
    if (!hfp_should_outband_ring()) {
        return;
    }
    if (hfp_outband_ring_task_running) {
        hfp_ag_send_outband_ring_state(reason);
        return;
    }

    hfp_outband_ring_task_running = true;
    BaseType_t ok = xTaskCreate(hfp_outband_ring_task, "hfp_ring", 3072,
                                (void *)reason, 6, NULL);
    if (ok != pdPASS) {
        hfp_outband_ring_task_running = false;
        ESP_LOGW(TAG, "HFP_AG_OUTBAND_RING_TASK:create_failed");
        hfp_ag_send_outband_ring_state(reason);
    }
}

static void hfp_stop_outband_ring(const char *reason)
{
    if (hfp_outband_ring_task_running) {
        ESP_LOGI(TAG, "HFP_AG_OUTBAND_RING_TASK:stop_requested reason=%s",
                 reason ? reason : "unknown");
    }
}

static void hfp_sync_phone_state(void)
{
    if (!hfp_ag_connected || !have_remote_bda) {
        return;
    }

    if (hfp_call_count() == 0) {
        esp_hf_ag_end_call(remote_bda, 0, 0, ESP_HF_CALL_STATUS_NO_CALLS,
                           ESP_HF_CALL_SETUP_STATUS_IDLE, NULL,
                           ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        return;
    }

    if (!hfp_has_established_call()) {
        if (hfp_has_mode(CALL_INCOMING)) {
            hfp_ag_send_outband_ring_state("SYNC_INCOMING");
            hfp_start_outband_ring("SYNC_INCOMING");
            return;
        }
        hfp_report_indicators();
        return;
    }

    esp_hf_ag_answer_call(remote_bda, hfp_report_num_active(), hfp_report_num_held(), hfp_call_status(),
                          hfp_call_setup_status(), hfp_report_number(),
                          ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
}

static void hfp_end_call(const char *source);

static void hfp_simulate_incoming_call(const char *number)
{
    if (!hfp_call_ready()) {
        return;
    }
    hfp_phone_sync_suppress_until = 0;
    hfp_call_t *call = hfp_add_call(CALL_INCOMING,
                                    ESP_HF_CURRENT_CALL_DIRECTION_INCOMING,
                                    number);
    if (call == NULL) {
        return;
    }
    hfp_ag_set_inband_ring(false);
    esp_hf_ag_answer_call(remote_bda, hfp_report_num_active(), hfp_report_num_held(),
                          hfp_call_status(), hfp_call_setup_status(),
                          call->number, ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
    hfp_report_indicators();
    hfp_start_outband_ring("INCOMING");
    hfp_log_call_state("INCOMING");
}

static call_mode_t hfp_mode_from_clcc_status(int status)
{
    switch (status) {
    case ESP_HF_CURRENT_CALL_STATUS_ACTIVE:
        return CALL_ACTIVE;
    case ESP_HF_CURRENT_CALL_STATUS_HELD:
        return CALL_HELD;
    case ESP_HF_CURRENT_CALL_STATUS_DIALING:
    case ESP_HF_CURRENT_CALL_STATUS_ALERTING:
        return CALL_OUTGOING;
    case ESP_HF_CURRENT_CALL_STATUS_INCOMING:
    case ESP_HF_CURRENT_CALL_STATUS_WAITING:
        return CALL_INCOMING;
    default:
        return CALL_ACTIVE;
    }
}

static void hfp_phone_clcc_begin(void)
{
    hfp_phone_clcc_batch_active = true;
    memset(hfp_phone_clcc_seen, 0, sizeof(hfp_phone_clcc_seen));
    ESP_LOGI(TAG, "HFP_AG_PHONE_CLCC_BEGIN");
}

static void hfp_phone_clcc_end(void)
{
    if (!hfp_phone_clcc_batch_active) {
        ESP_LOGI(TAG, "HFP_AG_PHONE_CLCC_END:no_batch");
        return;
    }

    hfp_phone_clcc_batch_active = false;
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && !hfp_phone_clcc_seen[i]) {
            hfp_clear_call(&hfp_calls[i]);
        }
    }

    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_phone_indicator_waiting_for_clcc = false;
    hfp_log_call_state("PHONE_CLCC_END");
}

static void hfp_sync_from_hf_clcc(const char *payload)
{
    int idx = 0;
    int dir = 0;
    int status = 0;
    int mpty = 0;
    char number[32] = {0};

    int parsed = sscanf(payload,
                        "HFP_HF_CLCC:idx=%d dir=%d status=%d mpty=%d num=\"%31[^\"]\"",
                        &idx, &dir, &status, &mpty, number);
    if (parsed < 4) {
        ESP_LOGW(TAG, "HFP_AG_PHONE_CLCC_BAD:%s", payload);
        return;
    }

    call_mode_t mode = hfp_mode_from_clcc_status(status);
    if (hfp_phone_sync_suppressed() && hfp_call_count() == 0 && mode != CALL_INCOMING) {
        ESP_LOGI(TAG, "HFP_AG_PHONE_CLCC_SUPPRESSED:%s", payload);
        return;
    }

    hfp_call_t *call = hfp_get_call_slot_by_index(idx);
    if (call == NULL) {
        ESP_LOGW(TAG, "HFP_AG_PHONE_CLCC_INDEX_BAD:%d", idx);
        return;
    }
    if (idx >= 1 && idx <= HFP_MAX_CALLS) {
        hfp_phone_clcc_seen[idx - 1] = true;
    }

    call->in_use = true;
    call->mode = mode;
    call->direction = dir == 1 ? ESP_HF_CURRENT_CALL_DIRECTION_INCOMING :
                      ESP_HF_CURRENT_CALL_DIRECTION_OUTGOING;
    if (parsed == 5 && number[0] != '\0') {
        snprintf(call->number, sizeof(call->number), "%s", number);
    } else if (call->number[0] == '\0') {
        snprintf(call->number, sizeof(call->number), "555123456%u",
                 (unsigned int)idx);
    }

    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (&hfp_calls[i] == call || !hfp_calls[i].in_use) {
            continue;
        }
        if (number[0] != '\0' && strcmp(hfp_calls[i].number, number) == 0 &&
            hfp_calls[i].mode == CALL_INCOMING && mode != CALL_INCOMING) {
            hfp_clear_call(&hfp_calls[i]);
        }
    }

    if (mpty != 0 && hfp_num_active() > 1) {
        for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
            if (hfp_calls[i].in_use && hfp_calls[i].mode == CALL_ACTIVE) {
                hfp_calls[i].mode = CALL_MERGED;
            }
        }
    }

    if (hfp_phone_clcc_batch_active) {
        hfp_log_call_state("PHONE_CLCC_ROW");
        return;
    }
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state("PHONE_CLCC");
}

static void hfp_sync_from_hf_cind_call(int call_status)
{
    if (call_status == 0 && !hfp_has_mode(CALL_INCOMING) && !hfp_has_mode(CALL_HELD)) {
        hfp_end_call("PHONE_CIND_END");
    } else if (call_status == 0) {
        if (hfp_phone_indicator_waiting_for_clcc) {
            hfp_log_call_state("PHONE_CIND_CALL0_WAIT_CLCC");
            return;
        }
        hfp_report_indicators();
        hfp_log_call_state("PHONE_CIND_CALL0");
    } else {
        if (hfp_phone_sync_suppressed() && hfp_call_count() == 0) {
            hfp_log_call_state("PHONE_CIND_CALL1_SUPPRESSED");
            return;
        }
        if (hfp_phone_indicator_waiting_for_clcc) {
            hfp_log_call_state("PHONE_CIND_CALL1_WAIT_CLCC");
            return;
        }
        hfp_report_indicators();
        hfp_log_call_state("PHONE_CIND_CALL1");
    }
}

static void hfp_sync_from_hf_cind_setup(int setup_status)
{
    if (setup_status != 0 && hfp_phone_sync_suppressed() && hfp_call_count() == 0) {
        hfp_log_call_state("PHONE_CIND_SETUP_SUPPRESSED");
        return;
    }
    if (setup_status == 0 && !hfp_has_established_call()) {
        for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
            if (hfp_calls[i].in_use && hfp_calls[i].mode == CALL_INCOMING) {
                hfp_clear_call(&hfp_calls[i]);
            }
        }
        hfp_sync_phone_state();
    }
    if (hfp_phone_indicator_waiting_for_clcc) {
        hfp_log_call_state("PHONE_CIND_SETUP_WAIT_CLCC");
        return;
    }
    hfp_report_indicators();
    hfp_log_call_state("PHONE_CIND_SETUP");
}

static void hfp_simulate_outgoing_call(const char *number)
{
    if (!hfp_call_ready()) {
        return;
    }
    hfp_call_t *call = hfp_add_call(CALL_OUTGOING,
                                    ESP_HF_CURRENT_CALL_DIRECTION_OUTGOING,
                                    number);
    if (call == NULL) {
        return;
    }
    esp_hf_ag_out_call(remote_bda, hfp_report_num_active(), hfp_report_num_held(),
                       ESP_HF_CALL_STATUS_CALL_IN_PROGRESS,
                       ESP_HF_CALL_SETUP_STATUS_OUTGOING_DIALING,
                       call->number, ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
    hfp_log_call_state("OUTGOING");
}

static void hfp_answer_call(void)
{
    if (!hfp_call_ready()) {
        return;
    }

    hfp_call_t *call = hfp_find_last_mode(CALL_INCOMING);
    if (call == NULL) {
        call = hfp_find_first_mode(CALL_OUTGOING);
    }
    if (call == NULL) {
        hfp_log_call_state("ANSWER_NO_SETUP");
        return;
    }

    if (hfp_has_established_call()) {
        for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
            if (&hfp_calls[i] != call && hfp_calls[i].in_use &&
                hfp_mode_is_active(hfp_calls[i].mode)) {
                hfp_calls[i].mode = CALL_HELD;
            }
        }
    }
    call->mode = CALL_ACTIVE;
    hfp_stop_outband_ring("ANSWER");

    ESP_LOGI(TAG, "HFP_AG_ANSWER_AUDIO_STATE:connected=%d sco_gpio=%d",
             hfp_ag_audio_connected, sco_pcm_gpio_active);
    esp_hf_ag_answer_call(remote_bda, hfp_report_num_active(), hfp_report_num_held(),
                          hfp_call_status(), hfp_call_setup_status(), call->number,
                          ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
    hfp_report_indicators();
    hfp_log_call_state("ANSWER");
}

static void hfp_end_call(const char *source)
{
    if (!hfp_call_ready()) {
        return;
    }
    bool suppress_phone_echo = source != NULL && strcmp(source, "END_BY_HF") == 0;
    bool keep_phone_echo_suppressed = hfp_phone_sync_suppressed();
    hfp_reset_calls();
    if (suppress_phone_echo || keep_phone_echo_suppressed) {
        hfp_suppress_phone_sync(HFP_POST_HANGUP_SUPPRESS_MS);
    }
    hfp_stop_outband_ring(source);
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state(source);
}

static void hfp_reject_call(void)
{
    if (!hfp_call_ready()) {
        return;
    }
    hfp_call_t *call = hfp_find_last_mode(CALL_INCOMING);
    if (call == NULL) {
        hfp_end_call("REJECT_NO_INCOMING");
        return;
    }
    char rejected_number[sizeof(call->number)];
    snprintf(rejected_number, sizeof(rejected_number), "%s", call->number);
    hfp_clear_call(call);
    hfp_stop_outband_ring("REJECT");
    esp_hf_ag_reject_call(remote_bda, hfp_report_num_active(), hfp_report_num_held(),
                          hfp_call_status(), hfp_call_setup_status(),
                          rejected_number, ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state("REJECT");
}

static void hfp_release_waiting_or_held(void)
{
    if (!hfp_call_ready()) {
        return;
    }
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use &&
            (hfp_calls[i].mode == CALL_INCOMING || hfp_calls[i].mode == CALL_HELD)) {
            hfp_clear_call(&hfp_calls[i]);
        }
    }
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state("REL_WAIT_HELD");
}

static void hfp_release_active_accept_other(void)
{
    if (!hfp_call_ready()) {
        return;
    }
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && hfp_mode_is_active(hfp_calls[i].mode)) {
            hfp_clear_call(&hfp_calls[i]);
        }
    }

    hfp_call_t *call = hfp_find_last_mode(CALL_INCOMING);
    if (call == NULL) {
        call = hfp_find_first_mode(CALL_HELD);
    }
    if (call != NULL) {
        call->mode = CALL_ACTIVE;
    }
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state("REL_ACTIVE_ACCEPT");
}

static void hfp_hangup_from_hf(void)
{
    if (!hfp_call_ready()) {
        return;
    }
    if (hfp_has_mode(CALL_INCOMING)) {
        hfp_reject_call();
        return;
    }
    if (hfp_num_active() > 0 && hfp_num_held() > 0) {
        hfp_release_active_accept_other();
        return;
    }
    hfp_end_call("END_BY_HF");
}

static void hfp_hold_active_accept_other(void)
{
    if (!hfp_call_ready()) {
        return;
    }
    hfp_call_t *next = hfp_find_last_mode(CALL_INCOMING);
    if (next == NULL) {
        next = hfp_find_first_mode(CALL_HELD);
    }

    if (next != NULL) {
        for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
            if (&hfp_calls[i] != next && hfp_calls[i].in_use &&
                hfp_mode_is_active(hfp_calls[i].mode)) {
                hfp_calls[i].mode = CALL_HELD;
            }
        }
        next->mode = CALL_ACTIVE;
    } else {
        hfp_call_t *active = hfp_find_first_mode(CALL_ACTIVE);
        if (active == NULL) {
            active = hfp_find_first_mode(CALL_MERGED);
        }
        if (active != NULL) {
            active->mode = CALL_HELD;
        } else {
            hfp_call_t *held = hfp_find_first_mode(CALL_HELD);
            if (held != NULL) {
                held->mode = CALL_ACTIVE;
            }
        }
    }
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state("HOLD_ACCEPT_SWAP");
}

static void hfp_merge_calls(void)
{
    if (!hfp_call_ready()) {
        return;
    }
    int established = 0;
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (hfp_calls[i].in_use && hfp_mode_is_established(hfp_calls[i].mode)) {
            established++;
            hfp_calls[i].mode = CALL_MERGED;
        }
    }
    if (established < 2) {
        ESP_LOGW(TAG, "HFP_AG_MERGE_IGNORED:established=%d", established);
    }
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state("MERGE");
}

static void hfp_release_indexed_call(int index)
{
    hfp_call_t *call = hfp_find_call_by_index(index);
    if (call == NULL) {
        ESP_LOGW(TAG, "HFP_AG_CHLD_INDEX_MISSING:%d", index);
        return;
    }
    hfp_clear_call(call);
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state("REL_INDEX");
}

static void hfp_private_consult_indexed_call(int index)
{
    hfp_call_t *selected = hfp_find_call_by_index(index);
    if (selected == NULL) {
        ESP_LOGW(TAG, "HFP_AG_CHLD_INDEX_MISSING:%d", index);
        return;
    }
    for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
        if (!hfp_calls[i].in_use) {
            continue;
        }
        hfp_calls[i].mode = (&hfp_calls[i] == selected) ? CALL_ACTIVE : CALL_HELD;
    }
    hfp_sync_phone_state();
    hfp_report_indicators();
    hfp_log_call_state("PRIV_INDEX");
}

static void hfp_bridge_chld_to_hf(const char *chld)
{
    ESP_LOGI(TAG, "BRIDGE_TX:HF_CHLD:%s", chld);
}

static bool hfp_should_defer_chld2(void)
{
    return hfp_num_active() > 0 && hfp_num_held() > 0 &&
           !hfp_has_mode(CALL_INCOMING);
}

static bool hfp_should_ignore_chld2(void)
{
    return hfp_num_established() < 2 &&
           !hfp_has_mode(CALL_INCOMING);
}

static void hfp_apply_chld(const char *chld)
{
    if (chld == NULL) {
        return;
    }
    if (chld[0] == '1' && isdigit((unsigned char)chld[1])) {
        hfp_release_indexed_call(atoi(&chld[1]));
        return;
    }
    if (chld[0] == '2' && isdigit((unsigned char)chld[1])) {
        hfp_private_consult_indexed_call(atoi(&chld[1]));
        return;
    }

    if (!hfp_call_control_accept(chld)) {
        return;
    }

    if (strcmp(chld, "0") == 0) {
        hfp_release_waiting_or_held();
    } else if (strcmp(chld, "1") == 0) {
        hfp_release_active_accept_other();
    } else if (strcmp(chld, "2") == 0) {
        hfp_hold_active_accept_other();
    } else if (strcmp(chld, "3") == 0) {
        hfp_merge_calls();
    } else {
        ESP_LOGW(TAG, "HFP_AG_CHLD_UNHANDLED:%s", chld);
    }
}

static void hfp_chld2_defer_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(HFP_CHLD2_DEFER_MS));
    if (hfp_chld2_defer_pending) {
        hfp_chld2_defer_pending = false;
        ESP_LOGI(TAG, "HFP_AG_CHLD2_DEFER_APPLY");
        hfp_bridge_chld_to_hf("2");
        hfp_apply_chld("2");
    } else {
        ESP_LOGI(TAG, "HFP_AG_CHLD2_DEFER_SKIPPED");
    }
    vTaskDelete(NULL);
}

static bool hfp_defer_chld2(void)
{
    if (hfp_chld2_defer_pending) {
        ESP_LOGI(TAG, "HFP_AG_CHLD2_DEFER_COALESCED");
        return true;
    }

    hfp_chld2_defer_pending = true;
    BaseType_t ok = xTaskCreate(hfp_chld2_defer_task, "hfp_chld2", 2048,
                                NULL, tskIDLE_PRIORITY + 3, NULL);
    if (ok != pdPASS) {
        hfp_chld2_defer_pending = false;
        ESP_LOGW(TAG, "HFP_AG_CHLD2_DEFER_FAILED");
        return false;
    }

    ESP_LOGI(TAG, "HFP_AG_CHLD2_DEFER:%dms", HFP_CHLD2_DEFER_MS);
    return true;
}

static void hfp_handle_chld(const char *chld)
{
    if (chld == NULL) {
        return;
    }

    if (hfp_chld2_defer_pending) {
        if (strcmp(chld, "1") == 0) {
            hfp_chld2_defer_pending = false;
            ESP_LOGI(TAG, "HFP_AG_CHLD2_DEFER_CANCEL:1");
            hfp_bridge_chld_to_hf(chld);
            hfp_apply_chld(chld);
            return;
        }
        if (strcmp(chld, "2") == 0) {
            ESP_LOGI(TAG, "HFP_AG_CHLD2_DEFER_COALESCED");
            return;
        }
        hfp_chld2_defer_pending = false;
        ESP_LOGI(TAG, "HFP_AG_CHLD2_DEFER_CANCEL:%s", chld);
    }

    if (strcmp(chld, "2") == 0 && hfp_should_defer_chld2()) {
        if (hfp_defer_chld2()) {
            return;
        }
    }
    if (strcmp(chld, "2") == 0 && hfp_should_ignore_chld2()) {
        ESP_LOGI(TAG, "HFP_AG_CHLD2_IGNORED:single_call");
        hfp_report_indicators();
        hfp_log_call_state("CHLD2_IGNORED");
        return;
    }

    hfp_bridge_chld_to_hf(chld);
    hfp_apply_chld(chld);
}

static void bt_app_hf_ag_cb(esp_hf_cb_event_t event, esp_hf_cb_param_t *param)
{
    char bda_str[18];

    switch (event) {
    case ESP_HF_PROF_STATE_EVT:
        ESP_LOGI(TAG, "HFP_AG_PROFILE:state=%d", param->prof_stat.state);
        break;

    case ESP_HF_CONNECTION_STATE_EVT:
        print_bda(param->conn_stat.remote_bda, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "HFP_AG_CONNECTION:state=%d bda=%s peer=0x%" PRIx32
                 " chld=0x%" PRIx32,
                 param->conn_stat.state, bda_str, param->conn_stat.peer_feat,
                 param->conn_stat.chld_feat);
        if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_CONNECTED) {
            memcpy(remote_bda, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            have_remote_bda = true;
            bt_paired = true;
            hfp_ag_connected = true;
            save_last_bda(remote_bda);
            hfp_ag_set_inband_ring(false);
        } else if (param->conn_stat.state == ESP_HF_CONNECTION_STATE_DISCONNECTED) {
            hfp_ag_connected = false;
            hfp_ag_audio_connected = false;
            hfp_reset_calls();
            hfp_update_media_policy("HFP_DISCONNECTED");
        }
        break;

    case ESP_HF_AUDIO_STATE_EVT:
        print_bda(param->audio_stat.remote_addr, bda_str, sizeof(bda_str));
        if (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED ||
            param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED_MSBC) {
            hfp_ag_audio_connected = true;
            sco_pcm_gpio_config();
        } else if (sco_pcm_gpio_active) {
            hfp_ag_audio_connected = false;
            a2dp_i2s_gpio_config();
        } else {
            hfp_ag_audio_connected = false;
        }
        ESP_LOGI(TAG, "HFP_AG_AUDIO:state=%d bda=%s handle=%d",
                 param->audio_stat.state, bda_str,
                 param->audio_stat.sync_conn_handle);
        if (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED_MSBC) {
            snprintf(hfp_rate_status, sizeof(hfp_rate_status),
                     "HFP_HF_AUDIO_RATE:codec=mSBC sample_rate=16000 channels=mono frame=%u",
                     param->audio_stat.preferred_frame_size);
        } else if (param->audio_stat.state == ESP_HF_AUDIO_STATE_CONNECTED) {
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

    case ESP_HF_CIND_RESPONSE_EVT:
        esp_hf_ag_cind_response(param->cind_rep.remote_addr,
                                hfp_call_status(),
                                hfp_call_setup_status(),
                                ESP_HF_NETWORK_STATE_AVAILABLE,
                                5,
                                ESP_HF_ROAMING_STATUS_INACTIVE,
                                5,
                                hfp_call_held_status());
        ESP_LOGI(TAG, "HFP_AG_CIND_RESPONSE");
        break;

    case ESP_HF_IND_UPDATE_EVT:
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_CALL,
                              hfp_call_status());
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_CALLSETUP,
                              hfp_call_setup_status());
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_SERVICE,
                              ESP_HF_NETWORK_STATE_AVAILABLE);
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_SIGNAL, 5);
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_BATTCHG, 5);
        esp_hf_ag_ciev_report(param->ind_upd.remote_addr, ESP_HF_IND_TYPE_CALLHELD,
                              hfp_call_held_status());
        ESP_LOGI(TAG, "HFP_AG_IND_UPDATE");
        break;

    case ESP_HF_COPS_RESPONSE_EVT:
        esp_hf_ag_cops_response(param->cops_rep.remote_addr, "ESP32");
        ESP_LOGI(TAG, "HFP_AG_COPS_RESPONSE");
        break;

    case ESP_HF_CLCC_RESPONSE_EVT:
        for (size_t i = 0; i < HFP_MAX_CALLS; i++) {
            if (!hfp_calls[i].in_use) {
                continue;
            }
            bool waiting = hfp_calls[i].mode == CALL_INCOMING &&
                           hfp_has_established_call();
            esp_hf_current_call_status_t status =
                hfp_current_call_status(hfp_calls[i].mode, waiting);
            esp_hf_current_call_mpty_type_t mpty =
                hfp_current_call_mpty_type(hfp_calls[i].mode);
            esp_hf_ag_clcc_response(param->clcc_rep.remote_addr, i + 1,
                                    hfp_calls[i].direction,
                                    status,
                                    ESP_HF_CURRENT_CALL_MODE_VOICE,
                                    mpty,
                                    hfp_calls[i].number,
                                    ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
            ESP_LOGI(TAG, "HFP_AG_CLCC:idx=%u dir=%d status=%d mpty=%d num=\"%s\"",
                     (unsigned int)(i + 1), hfp_calls[i].direction, status,
                     mpty, hfp_calls[i].number);
        }
        esp_hf_ag_clcc_response(param->clcc_rep.remote_addr, 0,
                                ESP_HF_CURRENT_CALL_DIRECTION_OUTGOING,
                                ESP_HF_CURRENT_CALL_STATUS_ACTIVE,
                                ESP_HF_CURRENT_CALL_MODE_VOICE,
                                ESP_HF_CURRENT_CALL_MPTY_TYPE_SINGLE,
                                NULL,
                                ESP_HF_CALL_ADDR_TYPE_UNKNOWN);
        ESP_LOGI(TAG, "HFP_AG_CLCC_RESPONSE");
        hfp_call_control_clear("clcc");
        break;

    case ESP_HF_CNUM_RESPONSE_EVT:
        esp_hf_ag_cnum_response(param->cnum_rep.remote_addr, "0000000000", 129,
                                ESP_HF_SUBSCRIBER_SERVICE_TYPE_VOICE);
        ESP_LOGI(TAG, "HFP_AG_CNUM_RESPONSE");
        break;

    case ESP_HF_UNAT_RESPONSE_EVT:
        print_bda(param->unat_rep.remote_addr, bda_str, sizeof(bda_str));
        ESP_LOGI(TAG, "HFP_AG_UNKNOWN_AT:bda=%s cmd=\"%s\"",
                 bda_str, param->unat_rep.unat ? param->unat_rep.unat : "");
        if (param->unat_rep.unat != NULL &&
            strncmp(param->unat_rep.unat, "+CHLD=", 6) == 0) {
            memcpy(remote_bda, param->unat_rep.remote_addr, ESP_BD_ADDR_LEN);
            have_remote_bda = true;
            hfp_handle_chld(param->unat_rep.unat + 6);
        }
        esp_hf_ag_unknown_at_send(param->unat_rep.remote_addr, NULL);
        break;

    case ESP_HF_ATA_RESPONSE_EVT:
        memcpy(remote_bda, param->ata_rep.remote_addr, ESP_BD_ADDR_LEN);
        have_remote_bda = true;
        ESP_LOGI(TAG, "BRIDGE_TX:HF_ANSWER");
        hfp_answer_call();
        break;

    case ESP_HF_CHUP_RESPONSE_EVT:
        memcpy(remote_bda, param->chup_rep.remote_addr, ESP_BD_ADDR_LEN);
        have_remote_bda = true;
        ESP_LOGI(TAG, "BRIDGE_TX:HF_HANGUP");
        hfp_hangup_from_hf();
        break;

    case ESP_HF_DIAL_EVT:
        memcpy(remote_bda, param->out_call.remote_addr, ESP_BD_ADDR_LEN);
        have_remote_bda = true;
        esp_hf_ag_cmee_send(param->out_call.remote_addr,
                            ESP_HF_AT_RESPONSE_CODE_OK,
                            ESP_HF_CME_AG_FAILURE);
        hfp_simulate_outgoing_call(param->out_call.num_or_loc);
        break;

    case ESP_HF_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "HFP_AG_VOLUME:type=%d volume=%d",
                 param->volume_control.type, param->volume_control.volume);
        break;

    default:
        ESP_LOGI(TAG, "HFP_AG_EVENT:%d", event);
        break;
    }
}

static void connect_bda(const char *bda_text)
{
    esp_bd_addr_t selected_bda;
    if (!parse_bda(bda_text, selected_bda)) {
        ESP_LOGW(TAG, "CONNECT_REJECTED:bad_bda");
        return;
    }

    memcpy(pending_connect_bda, selected_bda, ESP_BD_ADDR_LEN);
    memcpy(remote_bda, selected_bda, ESP_BD_ADDR_LEN);
    have_remote_bda = true;

    if (discovery_in_progress) {
        connect_pending = true;
        stop_discovery();
        ESP_LOGI(TAG, "CONNECT_PENDING:%s", bda_text);
        return;
    }

    begin_a2dp_connect(selected_bda);
}

static void send_pin_reply(const char *pin)
{
    esp_bt_pin_code_t pin_code = {0};
    uint8_t pin_len = 0;

    while (pin[pin_len] != '\0' && pin_len < ESP_BT_PIN_CODE_LEN) {
        if (!isdigit((unsigned char)pin[pin_len])) {
            ESP_LOGW(TAG, "PIN_REJECTED:digits_only");
            return;
        }
        pin_code[pin_len] = (uint8_t)pin[pin_len];
        pin_len++;
    }

    if (!have_remote_bda || pin_len == 0) {
        ESP_LOGW(TAG, "PIN_REJECTED:no_remote_or_empty");
        return;
    }

    esp_bt_gap_pin_reply(remote_bda, true, pin_len, pin_code);
    ESP_LOGI(TAG, "PIN_SENT:%u", pin_len);
}

static void print_status(void)
{
    char bda_str[18] = "none";
    if (have_remote_bda) {
        print_bda(remote_bda, bda_str, sizeof(bda_str));
    }
    ESP_LOGI(TAG, "STATUS:connected=%d connecting=%d profile_ready=%d pending=%d pairable=%d paired=%d reconnect=%" PRIu32 "/%d paused=%d mode=%d remote=%s i2s_rx=%" PRIu32 " short=%" PRIu32 " ring_over=%" PRIu32 "/%" PRIu32 " ring_under=%" PRIu32 "/%" PRIu32,
             a2dp_connected, a2dp_connecting, a2dp_profile_ready, connect_pending,
             bt_pairable, bt_paired, auto_reconnect_attempts, AUTO_RECONNECT_MAX_ATTEMPTS,
             auto_reconnect_paused, audio_mode, bda_str, i2s_rx_bytes, i2s_rx_short_count,
             pcm_ring_overrun_count, pcm_ring_overrun_bytes,
             pcm_ring_underflow_count, pcm_ring_underflow_bytes);
    ESP_LOGI(TAG, "%s", a2dp_rate_status);
    ESP_LOGI(TAG, "%s", hfp_rate_status);
}

static void print_board_id(void)
{
    ESP_LOGI(TAG, "BOARD_ID:role=AG name=%s build=%s profiles=A2DP_SOURCE,HFP_AG,AVRCP_TG",
             BT_DEVICE_NAME, AG_BUILD_TAG);
}

static void bridge_copy_quoted_text(const char *payload, char *out, size_t out_len)
{
    if (out_len == 0) {
        return;
    }
    out[0] = '\0';
    unsigned int payload_len = 0;
    const char *len_field = strstr(payload, " len=");
    if (len_field != NULL) {
        sscanf(len_field, " len=%u", &payload_len);
    }
    const char *start = strstr(payload, "text=\"");
    if (start == NULL) {
        return;
    }
    start += 6;
    size_t len = payload_len > 0 ? (size_t)payload_len : strlen(start);
    if (payload_len == 0) {
        const char *end = strrchr(start, '"');
        if (end != NULL && end >= start) {
            len = (size_t)(end - start);
        }
    }
    if (len >= out_len) {
        len = out_len - 1;
    }
    memcpy(out, start, len);
    out[len] = '\0';
}

static void handle_bridge_rx(const char *payload)
{
    if (strncmp(payload, "AVRCP_STATUS:", 13) == 0) {
        unsigned int len = 0;
        unsigned int pos = 0;
        int status = ESP_AVRC_PLAYBACK_STOPPED;
        if (sscanf(payload + 13, "len=%u pos=%u status=%d", &len, &pos, &status) == 3) {
            bool pos_changed = bridge_media_pos_ms != pos;
            bool status_changed = avrc_play_status != (esp_avrc_playback_stat_t)status;
            if (bridge_media_len_ms == len && !pos_changed && !status_changed) {
                return;
            }
            ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
            bridge_media_len_ms = len;
            bridge_media_pos_ms = pos;
            if (pos_changed) {
                notify_avrc_play_pos_changed();
            }
            if (status_changed) {
                set_bridge_play_status((esp_avrc_playback_stat_t)status);
            }
        }
    } else if (strncmp(payload, "AVRCP_POS:", 10) == 0) {
        uint32_t pos = (uint32_t)strtoul(payload + 10, NULL, 10);
        if (bridge_media_pos_ms != pos) {
            ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
            bridge_media_pos_ms = pos;
            notify_avrc_play_pos_changed();
        }
    } else if (strncmp(payload, "AVRCP_NOTIFY_PLAYBACK:", 22) == 0) {
        int status = atoi(payload + 22);
        if (avrc_play_status != (esp_avrc_playback_stat_t)status) {
            ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
            set_bridge_play_status((esp_avrc_playback_stat_t)status);
        }
    } else if (strncmp(payload, "AVRCP_METADATA:", 15) == 0) {
        unsigned int attr = 0;
        if (sscanf(payload + 15, "attr=%u", &attr) == 1) {
            if (attr == 1) {
                char title[sizeof(bridge_media_title)];
                bridge_copy_quoted_text(payload, title, sizeof(title));
                if (strcmp(bridge_media_title, title) != 0) {
                    ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
                    snprintf(bridge_media_title, sizeof(bridge_media_title), "%s", title);
                    bridge_track_index++;
                    bridge_track_change_pending = true;
                }
            } else if (attr == 2) {
                char artist[sizeof(bridge_media_artist)];
                bridge_copy_quoted_text(payload, artist, sizeof(artist));
                if (strcmp(bridge_media_artist, artist) != 0) {
                    ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
                    snprintf(bridge_media_artist, sizeof(bridge_media_artist), "%s", artist);
                }
            } else if (attr == 64) {
                char len_text[24];
                bridge_copy_quoted_text(payload, len_text, sizeof(len_text));
                uint32_t len = (uint32_t)strtoul(len_text, NULL, 10);
                if (bridge_media_len_ms != len) {
                    ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
                    bridge_media_len_ms = len;
                }
                if (bridge_track_change_pending) {
                    notify_avrc_track_changed();
                    bridge_track_change_pending = false;
                }
            }
        }
    } else if (strncmp(payload, "CALL_IN:", 8) == 0) {
        ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
        hfp_simulate_incoming_call(payload + 8);
    } else if (strcmp(payload, "HFP_HF_RING") == 0) {
        ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
        hfp_start_outband_ring("PHONE_RING");
    } else if (strcmp(payload, "CALL_END") == 0) {
        ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
        hfp_end_call("BRIDGE_END");
    } else if (strcmp(payload, "HFP_HF_CLCC_BEGIN") == 0) {
        ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
        hfp_phone_clcc_begin();
    } else if (strcmp(payload, "HFP_HF_CLCC_END") == 0) {
        ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
        hfp_phone_clcc_end();
    } else if (strncmp(payload, "HFP_HF_CLCC:", 12) == 0) {
        ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
        hfp_sync_from_hf_clcc(payload);
    } else if (strncmp(payload, "HFP_HF_CIND_CALL:", 17) == 0) {
        ESP_LOGI(TAG, "BRIDGE_RX:%s", payload);
        hfp_sync_from_hf_cind_call(atoi(payload + 17));
    } else if (strncmp(payload, "HFP_HF_CIND_CALLSETUP:", 22) == 0) {
        hfp_sync_from_hf_cind_setup(atoi(payload + 22));
    } else if (strncmp(payload, "HFP_HF_CIND_CALLHELD:", 21) == 0) {
        hfp_phone_indicator_waiting_for_clcc = true;
        hfp_log_call_state("PHONE_CIND_HELD_WAIT_CLCC");
    }
}

static void handle_uart_command(char *line)
{
    char *nl = strpbrk(line, "\r\n");
    if (nl != NULL) {
        *nl = '\0';
    }
    ESP_LOGI(TAG, "UART_RX:%s", line);

    if (strcmp(line, "BOARD_ID") == 0 || strcmp(line, "IDENT") == 0) {
        print_board_id();
    } else if (strcmp(line, "PLAY") == 0) {
        ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_PLAY");
        bridge_request_snapshot("avrcp");
        start_media(AUDIO_I2S);
    } else if (strcmp(line, "I2S") == 0) {
        start_media(AUDIO_I2S);
    } else if (strcmp(line, "PAUSE") == 0) {
        ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_PAUSE");
        bridge_request_snapshot("avrcp");
        set_media_paused(ESP_AVRC_PLAYBACK_PAUSED);
    } else if (strcmp(line, "STOP") == 0) {
        ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_STOP");
        bridge_request_snapshot("avrcp");
        set_media_paused(ESP_AVRC_PLAYBACK_STOPPED);
    } else if (strcmp(line, "SIMULATE") == 0) {
        start_media(AUDIO_SIMULATE);
    } else if (strcmp(line, "NEXT") == 0) {
        ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_NEXT");
        bridge_request_snapshot("avrcp");
    } else if (strcmp(line, "PREVIOUS") == 0) {
        ESP_LOGI(TAG, "BRIDGE_TX:AVRCP_PREVIOUS");
        bridge_request_snapshot("avrcp");
    } else if (strcmp(line, "CALL_IN") == 0) {
        hfp_simulate_incoming_call(NULL);
    } else if (strncmp(line, "CALL_IN:", 8) == 0) {
        hfp_simulate_incoming_call(line + 8);
    } else if (strcmp(line, "CALL_IN2") == 0) {
        hfp_simulate_incoming_call("5551234568");
    } else if (strncmp(line, "CALL_IN2:", 9) == 0) {
        hfp_simulate_incoming_call(line + 9);
    } else if (strcmp(line, "CALL_OUT") == 0) {
        hfp_simulate_outgoing_call(NULL);
    } else if (strncmp(line, "CALL_OUT:", 9) == 0) {
        hfp_simulate_outgoing_call(line + 9);
    } else if (strcmp(line, "CALL_ANSWER") == 0) {
        hfp_answer_call();
    } else if (strcmp(line, "CALL_REJECT") == 0) {
        hfp_reject_call();
    } else if (strcmp(line, "CALL_END") == 0) {
        hfp_end_call("END");
    } else if (strcmp(line, "CALL_SWAP") == 0 || strcmp(line, "CALL_HOLD") == 0) {
        hfp_hold_active_accept_other();
    } else if (strcmp(line, "CALL_MERGE") == 0) {
        hfp_merge_calls();
    } else if (strcmp(line, "CALL_RELEASE_WAITING") == 0) {
        hfp_release_waiting_or_held();
    } else if (strcmp(line, "CALL_RELEASE_ACTIVE") == 0) {
        hfp_release_active_accept_other();
    } else if (strcmp(line, "SCAN") == 0) {
        start_discovery(true);
    } else if (strcmp(line, "STOP_SCAN") == 0) {
        stop_discovery();
    } else if (strncmp(line, "CONNECT:", 8) == 0) {
        connect_bda(line + 8);
    } else if (strcmp(line, "CLEAR_PAIRING") == 0) {
        clear_last_bda(true);
        set_peer_reconnect_mode();
    } else if (strcmp(line, "PAUSE_RECONNECT") == 0) {
        auto_reconnect_paused = true;
        auto_reconnect_sdp_pending = false;
        set_bonded_listen_mode();
        ESP_LOGI(TAG, "AUTO_RECONNECT_PAUSED");
    } else if (strcmp(line, "PASSIVE_RECONNECT") == 0) {
        auto_reconnect_paused = true;
        auto_reconnect_sdp_pending = false;
        set_bonded_listen_mode();
        ESP_LOGI(TAG, "AUTO_RECONNECT_PASSIVE");
    } else if (strcmp(line, "RESUME_RECONNECT") == 0) {
        auto_reconnect_paused = false;
        auto_reconnect_not_before = 0;
        ESP_LOGI(TAG, "AUTO_RECONNECT_RESUMED");
    } else if (strcmp(line, "ACTIVE_RECONNECT") == 0) {
        auto_reconnect_paused = false;
        auto_reconnect_not_before = 0;
        ESP_LOGI(TAG, "AUTO_RECONNECT_ACTIVE");
    } else if (strncmp(line, "PIN:", 4) == 0) {
        send_pin_reply(line + 4);
    } else if (strcmp(line, "STATUS") == 0) {
        print_status();
    } else if (strncmp(line, "BRIDGE_RX:", 10) == 0) {
        handle_bridge_rx(line + 10);
    } else if (line[0] != '\0') {
        ESP_LOGW(TAG, "UNKNOWN_COMMAND:%s", line);
    }
}

static void uart_task(void *arg)
{
    (void)arg;
    char line[UART_CMD_BUF_LEN];
    size_t line_len = 0;
    uint8_t byte = 0;

    while (true) {
        int read_len = uart_read_bytes(UART_NUM_0, &byte, 1, portMAX_DELAY);
        if (read_len <= 0) {
            continue;
        }

        if (byte == '\r' || byte == '\n') {
            if (line_len > 0) {
                line[line_len] = '\0';
                handle_uart_command(line);
                line_len = 0;
            }
        } else if (line_len < (sizeof(line) - 1)) {
            line[line_len++] = (char)byte;
        } else {
            line_len = 0;
            ESP_LOGW(TAG, "UART_COMMAND_TOO_LONG");
        }
    }
}

static void reconnect_task(void *arg)
{
    (void)arg;

    while (true) {
        if (!a2dp_connected) {
            if (!a2dp_profile_ready) {
                refresh_a2dp_profile_ready();
            }
            if (a2dp_profile_ready && connect_pending && !discovery_in_progress &&
                !discovery_cancel_pending && !a2dp_connecting && !delayed_connect_scheduled) {
                schedule_delayed_pending_connect();
            } else if (a2dp_profile_ready && have_saved_bda && have_remote_bda &&
                       !discovery_in_progress && !discovery_cancel_pending &&
                       !a2dp_connecting && !delayed_connect_scheduled &&
                       !auto_reconnect_sdp_pending &&
                       !auto_reconnect_paused &&
                       auto_reconnect_attempts < AUTO_RECONNECT_MAX_ATTEMPTS) {
                TickType_t now = xTaskGetTickCount();
                if (now < auto_reconnect_not_before) {
                    ESP_LOGI(TAG, "AUTO_RECONNECT_WAIT:%" PRIu32 "ms",
                             (uint32_t)pdTICKS_TO_MS(auto_reconnect_not_before - now));
                    vTaskDelay(pdMS_TO_TICKS(RECONNECT_PERIOD_MS));
                    continue;
                }
                auto_reconnect_attempts++;
                ESP_LOGI(TAG, "AUTO_RECONNECT:%" PRIu32 "/%d",
                         auto_reconnect_attempts, AUTO_RECONNECT_MAX_ATTEMPTS);
                log_bond_state("auto_reconnect");
                memcpy(remote_bda, saved_bda, ESP_BD_ADDR_LEN);
                begin_a2dp_connect(remote_bda);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_PERIOD_MS));
    }
}

static void led_task(void *arg)
{
    (void)arg;

    while (true) {
        if (a2dp_connected) {
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

static void bt_init(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bredr_sco_datapath_set(ESP_SCO_DATA_PATH_PCM));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_app_gap_cb));
    ESP_ERROR_CHECK(esp_hf_ag_register_callback(bt_app_hf_ag_cb));
    ESP_ERROR_CHECK(esp_hf_ag_init());
    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(bt_app_avrc_ct_cb));
    ESP_ERROR_CHECK(esp_avrc_tg_init());
    ESP_ERROR_CHECK(esp_avrc_tg_register_callback(bt_app_avrc_tg_cb));
    configure_avrcp_target_controls();
    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_app_a2d_cb));
    ESP_ERROR_CHECK(esp_a2d_source_init());
    ESP_ERROR_CHECK(esp_a2d_source_register_data_callback(a2dp_data_cb));
    configure_phone_media_identity();

    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(BT_DEVICE_NAME));
    const uint8_t *local_bda = esp_bt_dev_get_address();
    char local_bda_str[18];
    print_bda(local_bda, local_bda_str, sizeof(local_bda_str));
    ESP_LOGI(TAG, "BT_LOCAL_BDA:%s", local_bda_str);

    esp_bt_eir_data_t eir_data = {
        .fec_required = false,
        .include_txpower = true,
        .include_uuid = false,
        .include_name = true,
        .flag = ESP_BT_EIR_FLAG_GEN_DISC,
    };
    esp_err_t eir_err = esp_bt_gap_config_eir_data(&eir_data);
    if (eir_err != ESP_OK) {
        ESP_LOGW(TAG, "BT_EIR_CONFIG_FAILED:0x%x", eir_err);
    }

    set_private_mode();

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap)));

    esp_bt_pin_code_t pin_code = {0};
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, pin_code));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_last_bda();
    ESP_ERROR_CHECK(rgb_led_init());
    ESP_ERROR_CHECK(i2s_input_init());
    pcm_ringbuf = xRingbufferCreate(PCM_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    ESP_ERROR_CHECK(pcm_ringbuf == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    BaseType_t task_ok = xTaskCreate(i2s_rx_task, "i2s_rx", 4096, NULL, 18, NULL);
    ESP_ERROR_CHECK(task_ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "I2S_RX_READY:rate=%d bclk=%d ws=%d din=%d role=slave dma=8x256 ring=%u",
             SAMPLE_RATE, I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DIN_GPIO,
             (unsigned int)PCM_RINGBUF_SIZE);

    bt_init();
    ESP_LOGI(TAG, "BT_READY:name=%s", BT_DEVICE_NAME);
    print_board_id();
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
    log_bond_state("boot");
    if (have_saved_bda && have_remote_bda) {
        set_bonded_listen_mode();
        auto_reconnect_paused = false;
        auto_reconnect_not_before = xTaskGetTickCount() +
                                    pdMS_TO_TICKS(BOOT_ACTIVE_RECONNECT_DELAY_MS);
        ESP_LOGI(TAG, "AUTO_RECONNECT_ACTIVE_BOOT:%dms",
                 BOOT_ACTIVE_RECONNECT_DELAY_MS);
    }
    print_status();

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0));
    xTaskCreate(uart_task, "uart_cmd", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(reconnect_task, "bt_reconnect", 4096, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(led_task, "rgb_led", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
}
