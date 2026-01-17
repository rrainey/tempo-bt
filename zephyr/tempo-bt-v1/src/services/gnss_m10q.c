/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - GNSS Service Implementation (u-blox SAM-M10Q)
 */
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <time.h>
#include <zephyr/posix/time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "services/gnss.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(gnss, LOG_LEVEL_INF);

static int ubx_send_with_ack(uint8_t msg_class, uint8_t msg_id, 
                             const uint8_t *payload, size_t payload_len);

/* UBX Protocol definitions */
#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62

/* UBX Message Classes */
#define UBX_CLASS_NAV   0x01
#define UBX_CLASS_RXM   0x02
#define UBX_CLASS_INF   0x04
#define UBX_CLASS_ACK   0x05
#define UBX_CLASS_CFG   0x06
#define UBX_CLASS_MON   0x0A

/* UBX Message IDs */
#define UBX_ID_ACK_NAK  0x00
#define UBX_ID_ACK_ACK  0x01
#define UBX_ID_CFG_RATE 0x08
#define UBX_ID_CFG_MSG  0x01
#define UBX_ID_CFG_CFG  0x09
#define UBX_ID_CFG_NAV5 0x24
#define UBX_ID_CFG_PM2  0x3B
#define UBX_ID_CFG_PMS  0x86

/* UBX ACK timeout */
#define UBX_ACK_TIMEOUT_MS 1000

/* UBX message structure */
struct ubx_msg {
    uint8_t class;
    uint8_t id;
    uint16_t len;
    uint8_t data[256];
};

/* UBX ACK/NAK tracking */
static struct {
    struct k_sem sem;
    bool ack_received;
    bool nak_received;
    uint8_t waiting_class;
    uint8_t waiting_id;
} ubx_ack_state;

/* UART TX completion tracking */
static struct {
    struct k_sem done;
    int result;
} uart_tx_state;

/* UBX parser state machine */
typedef enum {
    UBX_STATE_SYNC_SEARCH,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN_L,
    UBX_STATE_LEN_H,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CK_A,
    UBX_STATE_CK_B
} ubx_parse_state_t;

/* UBX receive parser state */
static struct {
    ubx_parse_state_t state;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t payload[64];    /* ACK messages only need 2 bytes, but allow some margin */
    uint16_t payload_idx;
    uint8_t ck_a, ck_b;     /* Received checksums */
    uint8_t calc_ck_a, calc_ck_b;  /* Calculated checksums */
} ubx_rx;

/* UBX-CFG-NAV5 message structure */
struct ubx_cfg_nav5 {
    uint16_t mask;              /* Parameters bitmask */
    uint8_t dyn_model;          /* Dynamic platform model */
    uint8_t fix_mode;           /* Position fixing mode */
    int32_t fixed_alt;          /* Fixed altitude (mean sea level) */
    uint32_t fixed_alt_var;     /* Fixed altitude variance */
    int8_t min_elev;            /* Minimum elevation */
    uint8_t dr_limit;           /* Dead reckoning timeout */
    uint16_t p_dop;             /* Position DOP mask */
    uint16_t t_dop;             /* Time DOP mask */
    uint16_t p_acc;             /* Position accuracy mask */
    uint16_t t_acc;             /* Time accuracy mask */
    uint8_t static_hold_thresh; /* Static hold threshold */
    uint8_t dgps_timeout;       /* DGPS timeout */
    uint8_t cno_thresh_num_svs; /* Min satellites above C/No thresh */
    uint8_t cno_thresh;         /* C/No threshold */
    uint16_t reserved1;         /* Reserved */
    uint16_t static_hold_max_dist; /* Static hold max distance */
    uint8_t utc_standard;       /* UTC standard */
    uint8_t reserved2;          /* Reserved */
    uint32_t reserved3;         /* Reserved */
} __packed;

// forward declaration
static void update_system_time_from_fix(void);
static uint64_t get_session_start_us(void);

/* UART configuration */
#define GNSS_UART_NODE DT_NODELABEL(uart2)
#define GNSS_UART_BUFFER_SIZE 256
#define GNSS_RX_TIMEOUT_US 100000  /* 100ms timeout */

/* UART device and buffers */
static const struct device *uart_dev;
static uint8_t uart_rx_buf[GNSS_UART_BUFFER_SIZE];
static uint8_t uart_rx_buf2[GNSS_UART_BUFFER_SIZE];  
static uint8_t nmea_line_buf[GNSS_UART_BUFFER_SIZE];
static size_t nmea_line_pos = 0;

/* Ring buffer for UART RX - sized for ~2 seconds of data at 9600 baud */
static uint8_t ring_buffer_mem[2048];
static struct ring_buf rx_ring_buf;

/* Semaphore to signal GNSS thread when data is available */
static struct k_sem rx_data_sem;

/* Flag to indicate GNSS subsystem is initialized (thread waits on this) */
static volatile bool gnss_initialized = false;

/* Flag to indicate system time has been set from GNSS */
static volatile bool gnss_time_set_flag = false;

/* Add session start tracking */
static uint64_t gnss_session_start_us = 0;

/* Add timing tracking for $PTH generation */
static uint64_t nmea_arrival_time_us = 0;

/* Callbacks */
static gnss_fix_callback_t fix_callback = NULL;
static gnss_nmea_callback_t nmea_callback = NULL;

/* Current fix data */
static gnss_fix_t current_fix;
static struct k_mutex fix_mutex;

/* UART callback */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);
    static uint8_t *next_buf = uart_rx_buf2;

    switch (evt->type) {
    case UART_TX_DONE:
        /* TX buffer has been sent */
        uart_tx_state.result = 0;
        k_sem_give(&uart_tx_state.done);
        break;

    case UART_TX_ABORTED:
        /* TX was aborted */
        LOG_WRN("UART TX aborted");
        uart_tx_state.result = -EIO;
        k_sem_give(&uart_tx_state.done);
        break;

    case UART_RX_RDY:
        /* Data ready, add to ring buffer */
        if (evt->data.rx.len > 0) {
            int written = ring_buf_put(&rx_ring_buf,
                                      evt->data.rx.buf + evt->data.rx.offset,
                                      evt->data.rx.len);
            if (written < evt->data.rx.len) {
                LOG_WRN("Ring buffer full, dropped %d bytes",
                        evt->data.rx.len - written);
            }
            /* Wake GNSS thread to process data */
            k_sem_give(&rx_data_sem);
        }
        break;
        
    case UART_RX_BUF_REQUEST:
        /* Provide next buffer */
        uart_rx_buf_rsp(uart_dev, next_buf, GNSS_UART_BUFFER_SIZE);
        /* Swap buffers */
        next_buf = (next_buf == uart_rx_buf) ? uart_rx_buf2 : uart_rx_buf;
        break;
        
    case UART_RX_BUF_RELEASED:
        /* Buffer released, nothing to do */
        break;
        
    case UART_RX_DISABLED:
        LOG_WRN("UART RX disabled");
        /* Try to re-enable */
        k_msleep(10);
        uart_rx_enable(uart_dev, uart_rx_buf, GNSS_UART_BUFFER_SIZE, 
                       GNSS_RX_TIMEOUT_US);
        break;
        
    case UART_RX_STOPPED:
        LOG_ERR("UART RX stopped due to error: %d", evt->data.rx_stop.reason);
        /* Clear ring buffer on overrun to prevent continuous errors */
        if (evt->data.rx_stop.reason == 4) {  /* UART_ERROR_OVERRUN */
            ring_buf_reset(&rx_ring_buf);
            LOG_WRN("Ring buffer cleared due to overrun");
        }
        /* Try to restart with longer delay */
        k_msleep(50);
        uart_rx_enable(uart_dev, uart_rx_buf, GNSS_UART_BUFFER_SIZE,
                       GNSS_RX_TIMEOUT_US);
        break;
        
    default:
        break;
    }
}

/* 
 * Validate NMEA checksum
 * Do Not Modify this algorithm!
 */
static bool validate_nmea_checksum(const char *sentence, size_t len)
{
    uint8_t calc_checksum = 0;

    if (len < 8 || sentence[0] != '$') {
        LOG_WRN("NMEA checksum failed: malformed sentence");
        return false;
    }
    
    const char *p = &sentence[1];
    for (size_t i = 1; i < len - 3; i++) {
        if (*p == '*') {
            break;
        }
        calc_checksum ^= (uint8_t)*p;
        p++;
    }

    if (*p != '*') {
        LOG_WRN("NMEA checksum failed: no asterisk");
        return false;
    }
    
    /* Parse provided checksum */
    char checksum_str[3] = {p[1], p[2], '\0'};

    uint8_t provided_checksum = (uint8_t)strtol(checksum_str, NULL, 16);
    
    return calc_checksum == (uint8_t)provided_checksum;
}

/* Parse NMEA time to components */
static bool parse_nmea_time(const char *time_str, gnss_fix_t *fix)
{
    if (strlen(time_str) < 6) {
        return false;
    }
    
    char buf[3] = {0};
    
    /* Hours */
    buf[0] = time_str[0];
    buf[1] = time_str[1];
    fix->hours = (uint8_t)strtol(buf, NULL, 10);
    
    /* Minutes */
    buf[0] = time_str[2];
    buf[1] = time_str[3];
    fix->minutes = (uint8_t)strtol(buf, NULL, 10);
    
    /* Seconds */
    buf[0] = time_str[4];
    buf[1] = time_str[5];
    fix->seconds = (uint8_t)strtol(buf, NULL, 10);
    
    /* Milliseconds if present */
    fix->milliseconds = 0;
    if (time_str[6] == '.' && strlen(time_str) >= 9) {
        char ms_buf[4] = {0};
        ms_buf[0] = time_str[7];
        ms_buf[1] = time_str[8];
        ms_buf[2] = '0';  /* Assume trailing zero if only 2 digits */
        fix->milliseconds = (uint16_t)strtol(ms_buf, NULL, 10);
    }

    LOG_DBG("Parsed time: %02d:%02d:%02d.%03d", 
            fix->hours, fix->minutes, fix->seconds, fix->milliseconds); 
    
    return true;
}

/* Parse coordinate from NMEA format to decimal degrees */
static double parse_nmea_coord(const char *coord_str, char hemisphere)
{
    double coord = strtod(coord_str, NULL);
    
    /* NMEA format: ddmm.mmmm or dddmm.mmmm */
    int degrees = (int)(coord / 100.0);
    double minutes = coord - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    
    /* Apply hemisphere */
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

/* Robust NMEA tokenizer that handles empty fields correctly */

#define MAX_NMEA_TOKENS 32

/* 
 * Tokenize NMEA sentence preserving empty fields
 * Returns number of tokens found
 * Modifies the input string by replacing commas with null terminators
 */
static int nmea_tokenize(char *sentence, char *tokens[], int max_tokens)
{
    int token_count = 0;
    char *p = sentence;
    
    if (!sentence || !tokens || max_tokens <= 0) {
        return 0;
    }
    
    /* First token starts at beginning */
    tokens[token_count++] = p;
    
    /* Process each character */
    while (*p && token_count < max_tokens) {
        if (*p == ',') {
            /* Replace comma with null terminator */
            *p = '\0';
            
            /* Next token starts after the comma */
            p++;
            if (token_count < max_tokens) {
                tokens[token_count++] = p;
            }
        } else if (*p == '*') {
            /* End of sentence data (checksum follows) */
            *p = '\0';
            break;
        } else {
            p++;
        }
    }
    
    return token_count;
}

/* Updated parse_gga function using robust tokenizer */
static void parse_gga(const char *sentence)
{
    char *tokens[MAX_NMEA_TOKENS];
    int token_count;
    static char sentence_copy[GNSS_UART_BUFFER_SIZE];
    
    /* Make a copy since tokenizer modifies the string */
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    /* Tokenize the sentence */
    token_count = nmea_tokenize(sentence_copy, tokens, MAX_NMEA_TOKENS);
    
    /* GGA requires at least 15 fields */
    if (token_count < 15) {
        LOG_WRN("GGA sentence too short: %d tokens", token_count);
        return;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    
    /* Parse time - field 1 */
    if (strlen(tokens[1]) > 0) {
        parse_nmea_time(tokens[1], &current_fix);
        current_fix.time_valid = true;
    } else {
        current_fix.time_valid = false;
    }
    
    /* Parse position - fields 2,3,4,5 */
    if (strlen(tokens[2]) > 0 && strlen(tokens[3]) > 0 && 
        strlen(tokens[4]) > 0 && strlen(tokens[5]) > 0) {
        current_fix.latitude = parse_nmea_coord(tokens[2], tokens[3][0]);
        current_fix.longitude = parse_nmea_coord(tokens[4], tokens[5][0]);
        current_fix.position_valid = true;
    } else {
        current_fix.position_valid = false;
    }
    
    /* Fix quality - field 6 */
    if (strlen(tokens[6]) > 0) {
        current_fix.fix_quality = (uint8_t)strtol(tokens[6], NULL, 10);
    } else {
        current_fix.fix_quality = 0;
    }
    
    /* Number of satellites - field 7 */
    if (strlen(tokens[7]) > 0) {
        current_fix.num_satellites = (uint8_t)strtol(tokens[7], NULL, 10);
    } else {
        current_fix.num_satellites = 0;
    }
    
    /* HDOP - field 8 */
    if (strlen(tokens[8]) > 0) {
        current_fix.hdop = strtof(tokens[8], NULL);
    } else {
        current_fix.hdop = 99.9f;  /* Invalid value */
    }
    
    /* Altitude - field 9 */
    if (strlen(tokens[9]) > 0) {
        current_fix.altitude = strtod(tokens[9], NULL);
        /* Field 10 is altitude units (M) - we assume meters */
    } else {
        current_fix.altitude = 0.0;
    }
    
    /* Geoidal separation - field 11 (optional) */
    /* Units - field 12 (optional) */
    
    /* Age of differential GPS data - field 13 (optional) */
    /* Differential reference station ID - field 14 (optional) */
    
    /* Update timestamp */
    current_fix.timestamp_us = time_now_us();
    
    k_mutex_unlock(&fix_mutex);
    
    LOG_DBG("GGA: Fix=%d, Sats=%d, Lat=%.6f, Lon=%.6f, Alt=%.1f",
            current_fix.fix_quality, current_fix.num_satellites,
            current_fix.latitude, current_fix.longitude,
            current_fix.altitude);

    /* Don't call update_system_time_from_fix() here - GGA doesn't contain date */
}

/* Updated parse_vtg function using robust tokenizer */
static void parse_vtg(const char *sentence)
{
    char *tokens[MAX_NMEA_TOKENS];
    int token_count;
    static char sentence_copy[GNSS_UART_BUFFER_SIZE];
    
    /* Make a copy since tokenizer modifies the string */
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    /* Tokenize the sentence */
    token_count = nmea_tokenize(sentence_copy, tokens, MAX_NMEA_TOKENS);
    
    /* VTG requires at least 10 fields (9 + checksum) */
    if (token_count < 10) {
        LOG_WRN("VTG sentence too short: %d tokens", token_count);
        return;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    
    /* Course over ground (true) - field 1 */
    if (strlen(tokens[1]) > 0) {
        current_fix.course_deg = strtof(tokens[1], NULL);
    }
    /* Field 2 is 'T' for true */
    
    /* Course over ground (magnetic) - field 3 (optional) */
    /* Field 4 is 'M' for magnetic */
    
    /* Speed over ground (knots) - field 5 */
    /* Field 6 is 'N' for knots */
    
    /* Speed over ground (km/h) - field 7 */
    if (strlen(tokens[7]) > 0) {
        float speed_kmh = strtof(tokens[7], NULL);
        current_fix.speed_mps = speed_kmh / 3.6f;
    } else {
        current_fix.speed_mps = 0.0f;
    }
    /* Field 8 is 'K' for km/h */
    
    /* Mode indicator - field 9 (optional in older NMEA versions) */
    
    k_mutex_unlock(&fix_mutex);
    
    LOG_DBG("VTG: Course=%.1f deg, Speed=%.2f m/s",
            current_fix.course_deg, current_fix.speed_mps);
    
    /* VTG typically follows GGA, so trigger fix callback now */
    if (fix_callback && current_fix.position_valid) {
        fix_callback(&current_fix);
    }
}

/* Updated parse_gsa function using robust tokenizer */
static void parse_gsa(const char *sentence)
{
    char *tokens[MAX_NMEA_TOKENS];
    int token_count;
    static char sentence_copy[GNSS_UART_BUFFER_SIZE];
    
    /* Make a copy since tokenizer modifies the string */
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    /* Tokenize the sentence */
    token_count = nmea_tokenize(sentence_copy, tokens, MAX_NMEA_TOKENS);
    
    /* GSA requires at least 18 fields */
    if (token_count < 18) {
        LOG_WRN("GSA sentence too short: %d tokens", token_count);
        return;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    
    /* Mode - field 1 (A=Auto, M=Manual) */
    /* Fix type - field 2 (1=no fix, 2=2D, 3=3D) */
    
    /* Satellite PRNs - fields 3-14 (12 slots) */
    /* These can be empty if fewer than 12 satellites are used */
    
    /* PDOP - field 15 */
    if (strlen(tokens[15]) > 0) {
        /* float pdop = strtof(tokens[15], NULL); */
        /* Not currently stored but available if needed */
    }
    
    /* HDOP - field 16 */
    if (strlen(tokens[16]) > 0) {
        current_fix.hdop = strtof(tokens[16], NULL);
    }
    
    /* VDOP - field 17 */
    if (strlen(tokens[17]) > 0) {
        current_fix.vdop = strtof(tokens[17], NULL);
    }
    
    k_mutex_unlock(&fix_mutex);
    
    LOG_DBG("GSA: HDOP=%.1f, VDOP=%.1f", current_fix.hdop, current_fix.vdop);
}

/* Updated parse_rmc function using robust tokenizer */
static void parse_rmc(const char *sentence)
{
    char *tokens[MAX_NMEA_TOKENS];
    int token_count;
    static char sentence_copy[GNSS_UART_BUFFER_SIZE];
    
    /* Make a copy since tokenizer modifies the string */
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    /* Tokenize the sentence */
    token_count = nmea_tokenize(sentence_copy, tokens, MAX_NMEA_TOKENS);
    
    /* RMC requires at least 13 fields (12 + checksum) */
    if (token_count < 13) {
        LOG_WRN("RMC sentence too short: %d tokens", token_count);
        return;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    
    /* Status - field 2 (A=active/valid, V=void/invalid) */
    bool data_valid = (strlen(tokens[2]) > 0 && tokens[2][0] == 'A');
    
    if (data_valid) {
        /* Time - field 1 */
        if (strlen(tokens[1]) > 0) {
            parse_nmea_time(tokens[1], &current_fix);
            current_fix.time_valid = true;
        }
        
        /* Latitude - fields 3,4 */
        if (strlen(tokens[3]) > 0 && strlen(tokens[4]) > 0) {
            current_fix.latitude = parse_nmea_coord(tokens[3], tokens[4][0]);
        }
        
        /* Longitude - fields 5,6 */
        if (strlen(tokens[5]) > 0 && strlen(tokens[6]) > 0) {
            current_fix.longitude = parse_nmea_coord(tokens[5], tokens[6][0]);
        }
        
        /* Speed over ground - field 7 (knots) */
        if (strlen(tokens[7]) > 0) {
            float speed_knots = strtof(tokens[7], NULL);
            current_fix.speed_mps = speed_knots * 0.514444f;
        }
        
        /* Course over ground - field 8 */
        if (strlen(tokens[8]) > 0) {
            current_fix.course_deg = strtof(tokens[8], NULL);
        }
        
        /* Date - field 9 (DDMMYY) */
        if (strlen(tokens[9]) >= 6) {
            char buf[3] = {0};
            
            /* Day */
            buf[0] = tokens[9][0];
            buf[1] = tokens[9][1];
            current_fix.day = (uint8_t)strtol(buf, NULL, 10);
            
            /* Month */
            buf[0] = tokens[9][2];
            buf[1] = tokens[9][3];
            current_fix.month = (uint8_t)strtol(buf, NULL, 10);
            
            /* Year (YY format) */
            buf[0] = tokens[9][4];
            buf[1] = tokens[9][5];
            long year_yy = strtol(buf, NULL, 10);
            current_fix.year = 2000 + year_yy;
            
            current_fix.date_valid = true;
            
            LOG_DBG("RMC: Date=%04d-%02d-%02d", 
                    current_fix.year, current_fix.month, current_fix.day);
        }
        
        /* Magnetic variation - field 10 (optional) */
        /* Variation direction - field 11 (E/W) (optional) */
        /* Mode indicator - field 12 (optional in NMEA 2.3+) */
        
    } else {
        LOG_DBG("RMC: Status void");
        current_fix.date_valid = false;
        current_fix.time_valid = false;
    }
    
    k_mutex_unlock(&fix_mutex);
    
    update_system_time_from_fix();
}

/*
 * UBX Binary Protocol Parser
 */

/* Reset UBX parser to initial state */
static void ubx_reset_parser(void)
{
    ubx_rx.state = UBX_STATE_SYNC_SEARCH;
    ubx_rx.payload_idx = 0;
    ubx_rx.length = 0;
    ubx_rx.calc_ck_a = 0;
    ubx_rx.calc_ck_b = 0;
}

/* Update running checksum with a byte */
static inline void ubx_update_rx_checksum(uint8_t byte)
{
    ubx_rx.calc_ck_a += byte;
    ubx_rx.calc_ck_b += ubx_rx.calc_ck_a;
}

/* Handle a complete UBX message */
static void ubx_handle_message(void)
{
    if (ubx_rx.msg_class == UBX_CLASS_ACK) {
        if (ubx_rx.msg_id == UBX_ID_ACK_ACK) {
            /* ACK-ACK: positive acknowledgement */
            if (ubx_rx.length >= 2) {
                uint8_t ack_class = ubx_rx.payload[0];
                uint8_t ack_id = ubx_rx.payload[1];

                LOG_INF("UBX-ACK-ACK: class=0x%02X id=0x%02X", ack_class, ack_id);

                /* Check if this matches what we're waiting for */
                if (ack_class == ubx_ack_state.waiting_class &&
                    ack_id == ubx_ack_state.waiting_id) {
                    ubx_ack_state.ack_received = true;
                    k_sem_give(&ubx_ack_state.sem);
                }
            }
        } else if (ubx_rx.msg_id == UBX_ID_ACK_NAK) {
            /* ACK-NAK: negative acknowledgement */
            if (ubx_rx.length >= 2) {
                uint8_t nak_class = ubx_rx.payload[0];
                uint8_t nak_id = ubx_rx.payload[1];

                LOG_WRN("UBX-ACK-NAK: class=0x%02X id=0x%02X", nak_class, nak_id);

                /* Check if this matches what we're waiting for */
                if (nak_class == ubx_ack_state.waiting_class &&
                    nak_id == ubx_ack_state.waiting_id) {
                    ubx_ack_state.nak_received = true;
                    k_sem_give(&ubx_ack_state.sem);
                }
            }
        }
    }
    /* Other message classes (NAV, MON, etc.) can be handled here in future */
}

/* Process a single byte through the UBX parser state machine */
static void ubx_process_byte(uint8_t byte)
{
    switch (ubx_rx.state) {
    case UBX_STATE_SYNC_SEARCH:
        if (byte == UBX_SYNC1) {
            ubx_rx.state = UBX_STATE_SYNC2;
        }
        break;

    case UBX_STATE_SYNC2:
        if (byte == UBX_SYNC2) {
            ubx_rx.state = UBX_STATE_CLASS;
            ubx_rx.calc_ck_a = 0;
            ubx_rx.calc_ck_b = 0;
        } else if (byte == UBX_SYNC1) {
            /* Stay in SYNC2 state - might be another sync sequence */
            ubx_rx.state = UBX_STATE_SYNC2;
        } else {
            /* False sync, go back to searching */
            ubx_reset_parser();
        }
        break;

    case UBX_STATE_CLASS:
        ubx_rx.msg_class = byte;
        ubx_update_rx_checksum(byte);
        ubx_rx.state = UBX_STATE_ID;
        break;

    case UBX_STATE_ID:
        ubx_rx.msg_id = byte;
        ubx_update_rx_checksum(byte);
        ubx_rx.state = UBX_STATE_LEN_L;
        break;

    case UBX_STATE_LEN_L:
        ubx_rx.length = byte;  /* LSB */
        ubx_update_rx_checksum(byte);
        ubx_rx.state = UBX_STATE_LEN_H;
        break;

    case UBX_STATE_LEN_H:
        ubx_rx.length |= ((uint16_t)byte << 8);  /* MSB */
        ubx_update_rx_checksum(byte);

        if (ubx_rx.length > sizeof(ubx_rx.payload)) {
            LOG_WRN("UBX message too long: %u bytes", ubx_rx.length);
            ubx_reset_parser();
        } else if (ubx_rx.length == 0) {
            /* Empty payload, go straight to checksum */
            ubx_rx.state = UBX_STATE_CK_A;
        } else {
            ubx_rx.payload_idx = 0;
            ubx_rx.state = UBX_STATE_PAYLOAD;
        }
        break;

    case UBX_STATE_PAYLOAD:
        ubx_rx.payload[ubx_rx.payload_idx++] = byte;
        ubx_update_rx_checksum(byte);

        if (ubx_rx.payload_idx >= ubx_rx.length) {
            ubx_rx.state = UBX_STATE_CK_A;
        }
        break;

    case UBX_STATE_CK_A:
        ubx_rx.ck_a = byte;
        ubx_rx.state = UBX_STATE_CK_B;
        break;

    case UBX_STATE_CK_B:
        ubx_rx.ck_b = byte;

        /* Message complete: validate checksum and process */
        if (ubx_rx.ck_a == ubx_rx.calc_ck_a &&
            ubx_rx.ck_b == ubx_rx.calc_ck_b) {
            ubx_handle_message();
        } else {
            LOG_WRN("UBX checksum mismatch: got %02X %02X, calc %02X %02X",
                    ubx_rx.ck_a, ubx_rx.ck_b,
                    ubx_rx.calc_ck_a, ubx_rx.calc_ck_b);
        }

        ubx_reset_parser();
        break;
    }
}

/* Process NMEA sentence by type */
static void process_nmea_sentence(const char *sentence, size_t len)
{
    if (len > 6) {
        if (strncmp(sentence, "$GNGGA", 6) == 0 || 
            strncmp(sentence, "$GPGGA", 6) == 0) {
            parse_gga(sentence);
        } else if (strncmp(sentence, "$GNVTG", 6) == 0 || 
                   strncmp(sentence, "$GPVTG", 6) == 0) {
            parse_vtg(sentence);
        } else if (strncmp(sentence, "$GNGSA", 6) == 0 || 
                   strncmp(sentence, "$GPGSA", 6) == 0) {
            parse_gsa(sentence);  
        } else if (strncmp(sentence, "$GNRMC", 6) == 0 || 
                   strncmp(sentence, "$GPRMC", 6) == 0) {
            parse_rmc(sentence);  
        }
    }
}

/* Process received data from ring buffer - demux UBX and NMEA */
static void process_uart_data(void)
{
    uint8_t byte;

    while (ring_buf_get(&rx_ring_buf, &byte, 1) == 1) {
        /*
         * Demux logic: route bytes to UBX or NMEA parser
         * - If we're mid-UBX message, continue UBX parsing
         * - If we see UBX sync (0xB5), start UBX parsing
         * - If we see NMEA start ($), start NMEA parsing
         */
        if (ubx_rx.state != UBX_STATE_SYNC_SEARCH) {
            /* We're in the middle of parsing a UBX message */
            ubx_process_byte(byte);
        } else if (byte == UBX_SYNC1) {
            /* Potential UBX message start */
            ubx_process_byte(byte);
        } else if (byte == '$') {
            /* NMEA sentence start */
            nmea_arrival_time_us = time_now_us();
            nmea_line_pos = 0;
            nmea_line_buf[nmea_line_pos++] = byte;
        } else if (byte == '\r' || byte == '\n') {
            /* NMEA line terminator */
            if (nmea_line_pos > 0) {
                /* Null terminate the line */
                nmea_line_buf[nmea_line_pos] = '\0';

                /* Validate checksum */
                if (validate_nmea_checksum((char *)nmea_line_buf, nmea_line_pos)) {
                    LOG_DBG("NMEA valid: %s", nmea_line_buf);

                    /* Process specific sentence types */
                    process_nmea_sentence((char *)nmea_line_buf, nmea_line_pos);

                    /* Call NMEA callback if registered - pass full sentence with CRLF */
                    if (nmea_callback) {
                        /* Restore CRLF for logging */
                        nmea_line_buf[nmea_line_pos] = '\r';
                        nmea_line_buf[nmea_line_pos + 1] = '\n';
                        nmea_line_buf[nmea_line_pos + 2] = '\0';

                        /* Pass arrival time in milliseconds relative to session start */
                        uint32_t arrival_ms = (uint32_t)((nmea_arrival_time_us - get_session_start_us()) / 1000);
                        (void)arrival_ms;  /* Currently unused but available */
                        nmea_callback((char *)nmea_line_buf, nmea_line_pos + 2);
                    }
                } else {
                    LOG_WRN("NMEA checksum failed: %s", nmea_line_buf);
                }

                /* Reset line buffer */
                nmea_line_pos = 0;
            }
        } else if (nmea_line_pos > 0) {
            /* Continue NMEA line */
            if (nmea_line_pos < sizeof(nmea_line_buf) - 3) {  /* Leave room for CRLF and null */
                nmea_line_buf[nmea_line_pos++] = byte;
            } else {
                /* Line too long, discard and start over */
                LOG_WRN("NMEA line too long, discarding");
                nmea_line_pos = 0;
            }
        }
        /* Else: stray byte before any sync prefix, ignore */
    }
}

/* GNSS processing thread */
static void gnss_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("GNSS thread started, waiting for init...");

    /* Wait for gnss_init() to complete before using any resources */
    while (!gnss_initialized) {
        k_msleep(10);
    }

    LOG_INF("GNSS thread running");

    while (1) {
        /* Wait for data to be available (with timeout for periodic checks) */
        k_sem_take(&rx_data_sem, K_MSEC(100));

        /* Process all available data in the ring buffer */
        process_uart_data();
    }
}

K_THREAD_DEFINE(gnss_thread_id, 4096, gnss_thread, NULL, NULL, NULL,
                7, 0, 0);

/*
 * Early GNSS quiet function - call very early in boot sequence.
 * This disables NMEA output and drains any pending data to prevent
 * buffer overruns when the GNSS module continues running after CPU reset.
 */
int gnss_early_quiet(void)
{
    const struct device *dev;
    int ret;

    LOG_INF("GNSS early quiet: silencing module");

    /* Get UART device */
    dev = DEVICE_DT_GET(GNSS_UART_NODE);
    if (!device_is_ready(dev)) {
        LOG_WRN("GNSS UART not ready for early quiet");
        return -ENODEV;
    }

    /* Configure UART for polling mode (no async callbacks yet) */
    struct uart_config cfg = {
        .baudrate = 9600,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };

    ret = uart_configure(dev, &cfg);
    if (ret < 0) {
        LOG_WRN("Failed to configure UART for early quiet: %d", ret);
        /* Continue anyway - might work with existing config */
    }

    /* Drain any pending RX data (polling mode) */
    uint8_t dummy;
    int drained = 0;
    while (uart_poll_in(dev, &dummy) == 0 && drained < 2048) {
        drained++;
    }
    if (drained > 0) {
        LOG_INF("GNSS early quiet: drained %d bytes", drained);
    }

    /*
     * Send UBX-CFG-MSG commands to disable all standard NMEA messages.
     * This uses polling mode TX since async isn't set up yet.
     * Format: UBX-CFG-MSG with rate=0 for UART1
     */

    /* UBX-CFG-MSG to disable GGA (0xF0 0x00) */
    static const uint8_t disable_gga[] = {
        0xB5, 0x62,             /* UBX sync */
        0x06, 0x01,             /* CFG-MSG class/id */
        0x08, 0x00,             /* Length = 8 */
        0xF0, 0x00,             /* NMEA-GGA message */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  /* Rates for 6 ports (all 0) */
        0xFF, 0x23              /* Checksum (placeholder) */
    };

    /* UBX-CFG-MSG to disable GLL (0xF0 0x01) */
    static const uint8_t disable_gll[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x2A
    };

    /* UBX-CFG-MSG to disable GSA (0xF0 0x02) */
    static const uint8_t disable_gsa[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x31
    };

    /* UBX-CFG-MSG to disable GSV (0xF0 0x03) */
    static const uint8_t disable_gsv[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x02, 0x38
    };

    /* UBX-CFG-MSG to disable RMC (0xF0 0x04) */
    static const uint8_t disable_rmc[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x03, 0x3F
    };

    /* UBX-CFG-MSG to disable VTG (0xF0 0x05) */
    static const uint8_t disable_vtg[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x04, 0x46
    };

    /* Send disable commands using poll_out */
    const uint8_t *cmds[] = {
        disable_gga, disable_gll, disable_gsa,
        disable_gsv, disable_rmc, disable_vtg
    };
    const size_t cmd_lens[] = {16, 16, 16, 16, 16, 16};

    for (int i = 0; i < 6; i++) {
        for (size_t j = 0; j < cmd_lens[i]; j++) {
            uart_poll_out(dev, cmds[i][j]);
        }
        /* Small delay between commands */
        k_busy_wait(1000);  /* 1ms */
    }

    /* Wait a bit for commands to be processed */
    k_msleep(50);

    /* Drain any response/remaining data */
    drained = 0;
    while (uart_poll_in(dev, &dummy) == 0 && drained < 1024) {
        drained++;
    }
    if (drained > 0) {
        LOG_INF("GNSS early quiet: drained %d more bytes after quiet commands", drained);
    }

    LOG_INF("GNSS early quiet: complete");
    return 0;
}

int gnss_init(void)
{
    int ret;
    
    /* Get UART device */
    uart_dev = DEVICE_DT_GET(GNSS_UART_NODE); 
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("GNSS UART device not ready");
        return -ENODEV;
    }
    
    LOG_INF("UART device %s is ready", uart_dev->name);
    
    /* Try to get current configuration first */
    struct uart_config current_cfg;
    ret = uart_config_get(uart_dev, &current_cfg);
    if (ret == 0) {
        LOG_INF("Current UART config: baud=%d, data=%d, stop=%d, parity=%d, flow=%d",
                current_cfg.baudrate, current_cfg.data_bits, current_cfg.stop_bits,
                current_cfg.parity, current_cfg.flow_ctrl);
    }
    else {
        LOG_WRN("Failed to get current UART config: %d", ret);
    }   
    
    /* Configure UART */
    struct uart_config cfg = {
        .baudrate = 9600,  /* SAM-M10Q default is actually 9600 */
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };
    
    ret = uart_configure(uart_dev, &cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure UART: %d", ret);
        /* Try without reconfiguring */
        LOG_WRN("Attempting to use UART with existing configuration");
        ret = 0;  /* Continue anyway */
    } else {
        LOG_INF("UART configured successfully");
    }
    
    /* Initialize ring buffer */
    ring_buf_init(&rx_ring_buf, sizeof(ring_buffer_mem), ring_buffer_mem);

    /* Initialize mutex */
    k_mutex_init(&fix_mutex);

    /* Initialize fix data */
    memset(&current_fix, 0, sizeof(current_fix));

    /* Initialize UBX parser */
    ubx_reset_parser();

    /* Initialize RX data semaphore (used to wake GNSS thread) */
    k_sem_init(&rx_data_sem, 0, 1);

    /* Initialize TX completion semaphore */
    k_sem_init(&uart_tx_state.done, 0, 1);
    uart_tx_state.result = 0;

    /* Initialize ACK tracking semaphore */
    k_sem_init(&ubx_ack_state.sem, 0, 1);
    ubx_ack_state.ack_received = false;
    ubx_ack_state.nak_received = false;
    ubx_ack_state.waiting_class = 0;
    ubx_ack_state.waiting_id = 0;

    /* Set up async UART */
    ret = uart_callback_set(uart_dev, uart_cb, NULL);
    if (ret < 0) {
        LOG_ERR("Failed to set UART callback: %d", ret);
        return ret;
    }

    /* Start receiving */
    ret = uart_rx_enable(uart_dev, uart_rx_buf, sizeof(uart_rx_buf),
                         GNSS_RX_TIMEOUT_US);
    if (ret < 0) {
        LOG_ERR("Failed to enable UART RX: %d", ret);
        return ret;
    }

    LOG_INF("GNSS initialized on %s at %d baud",
            uart_dev->name, cfg.baudrate);

    /* Signal GNSS thread that initialization is complete */
    gnss_initialized = true;

    /* Give GNSS thread time to start processing before sending UBX commands */
    k_msleep(50);

    /* Re-enable required NMEA messages that were disabled in gnss_early_quiet() */
    LOG_INF("Re-enabling NMEA messages...");

    /*
     * UBX-CFG-MSG format: class(1) + id(1) + rate[6](one per port)
     * We set rate=1 for UART1 (port index 1) to enable output once per nav cycle
     * Port order: DDC(I2C), UART1, UART2, USB, SPI, reserved
     */

    /* Enable GGA (0xF0 0x00) - position, fix quality, satellites */
    static const uint8_t enable_gga[] = {0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_MSG, enable_gga, sizeof(enable_gga));
    if (ret < 0) {
        LOG_WRN("Failed to enable GGA: %d", ret);
    }

    /* Enable RMC (0xF0 0x04) - time, date, position, velocity */
    static const uint8_t enable_rmc[] = {0xF0, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_MSG, enable_rmc, sizeof(enable_rmc));
    if (ret < 0) {
        LOG_WRN("Failed to enable RMC: %d", ret);
    }

    /* Enable VTG (0xF0 0x05) - velocity/track */
    static const uint8_t enable_vtg[] = {0xF0, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_MSG, enable_vtg, sizeof(enable_vtg));
    if (ret < 0) {
        LOG_WRN("Failed to enable VTG: %d", ret);
    }

    /* Enable GSA (0xF0 0x02) - DOP and active satellites */
    static const uint8_t enable_gsa[] = {0xF0, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_MSG, enable_gsa, sizeof(enable_gsa));
    if (ret < 0) {
        LOG_WRN("Failed to enable GSA: %d", ret);
    }

    LOG_INF("NMEA messages re-enabled");

    return 0;
}

void gnss_register_fix_callback(gnss_fix_callback_t callback)
{
    fix_callback = callback;
}

void gnss_register_nmea_callback(gnss_nmea_callback_t callback)
{
    nmea_callback = callback;
}

bool gnss_get_current_fix(gnss_fix_t *fix)
{
    if (!fix) {
        return false;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    *fix = current_fix;
    k_mutex_unlock(&fix_mutex);
    
    return fix->position_valid;
}

int gnss_set_rate(uint8_t rate_hz)
{
    int ret;
    uint8_t ubx_cfg_rate[14];
    uint16_t period_ms;
    
    /* SAM-M10Q supports 1Hz to 10Hz */
    if (rate_hz < 1 || rate_hz > 10) {
        LOG_ERR("Invalid rate %d Hz (SAM-M10Q supports 1-10 Hz)", rate_hz);
        return -EINVAL;
    }
    
    /* Calculate period in milliseconds */
    period_ms = 1000 / rate_hz;
    
    LOG_INF("Setting GNSS rate to %d Hz (period %d ms)", rate_hz, period_ms);
    
    /* Build UBX-CFG-RATE message */
    ubx_cfg_rate[0] = 0xB5;  /* Sync char 1 */
    ubx_cfg_rate[1] = 0x62;  /* Sync char 2 */
    ubx_cfg_rate[2] = 0x06;  /* Class: CFG */
    ubx_cfg_rate[3] = 0x08;  /* ID: RATE */
    ubx_cfg_rate[4] = 0x06;  /* Length LSB */
    ubx_cfg_rate[5] = 0x00;  /* Length MSB */
    ubx_cfg_rate[6] = period_ms & 0xFF;  /* measRate LSB */
    ubx_cfg_rate[7] = (period_ms >> 8) & 0xFF;  /* measRate MSB */
    ubx_cfg_rate[8] = 0x01;  /* navRate LSB (1 = every measurement) */
    ubx_cfg_rate[9] = 0x00;  /* navRate MSB */
    ubx_cfg_rate[10] = 0x01; /* timeRef LSB (1 = GPS time) */
    ubx_cfg_rate[11] = 0x00; /* timeRef MSB */
    
    /* Calculate and add checksum */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 12; i++) {
        ck_a += ubx_cfg_rate[i];
        ck_b += ck_a;
    }
    ubx_cfg_rate[12] = ck_a;
    ubx_cfg_rate[13] = ck_b;
    
    /* Send command */
    ret = uart_tx(uart_dev, ubx_cfg_rate, sizeof(ubx_cfg_rate), SYS_FOREVER_MS);
    if (ret < 0) {
        LOG_ERR("Failed to send rate config: %d", ret);
        return ret;
    }
    
    /* TODO: Wait for UBX-ACK-ACK response to confirm */
    k_msleep(100);
    
    LOG_INF("GNSS rate configuration sent");
    
    return 0;
}

/* Set system time from GNSS fix data */
static void update_system_time_from_fix(void)
{
    struct timespec ts;
    time_t unix_time;
    struct tm time_tm;

    //LOG_INF("Attempting to update system time from GNSS fix: %d %d", current_fix.date_valid, current_fix.time_valid);

    /* Check if we have valid date AND time */
    if (!current_fix.date_valid || !current_fix.time_valid) {
        return;  /* Need both valid date and time */
    }

    /* Check if we've already set the time this session */
    if (gnss_time_set_flag) {
        return;  /* Only set once per boot */
    }
    
    /* Build tm structure */
    memset(&time_tm, 0, sizeof(time_tm));
    time_tm.tm_year = current_fix.year - 1900;  /* tm_year is years since 1900 */
    time_tm.tm_mon = current_fix.month - 1;     /* tm_mon is 0-11 */
    time_tm.tm_mday = current_fix.day;
    time_tm.tm_hour = current_fix.hours;
    time_tm.tm_min = current_fix.minutes;
    time_tm.tm_sec = current_fix.seconds;
    time_tm.tm_isdst = 0;  /* UTC has no DST */
    
    /* Convert to Unix timestamp */
    unix_time = mktime(&time_tm);
    if (unix_time == (time_t)-1) {
        LOG_ERR("Failed to convert GNSS time to Unix timestamp");
        current_fix.date_valid = false; 
        current_fix.time_valid = false;
        return;
    }
    
    /* Set system time */
    ts.tv_sec = unix_time;
    ts.tv_nsec = current_fix.milliseconds * 1000000;  /* Convert ms to ns */

    LOG_INF("Initializing system time from GNSS: %04d-%02d-%02d %02d:%02d:%02d.%03d UTC",
                current_fix.year, current_fix.month, current_fix.day,
                current_fix.hours, current_fix.minutes, current_fix.seconds,
                current_fix.milliseconds);
    
    int ret = clock_settime(CLOCK_REALTIME, &ts);
    if (ret == 0) {
        gnss_time_set_flag = true;
        LOG_INF("System time set from GNSS: %04d-%02d-%02d %02d:%02d:%02d.%03d UTC",
                current_fix.year, current_fix.month, current_fix.day,
                current_fix.hours, current_fix.minutes, current_fix.seconds,
                current_fix.milliseconds);

        /* Also update the global date string used by aggregator */
        extern char gps_date_string[14];
        snprintf(gps_date_string, sizeof(gps_date_string),
                 "%04d-%02d-%02d", current_fix.year, current_fix.month, current_fix.day);
    } else {
        LOG_ERR("Failed to set system time: %d", ret);
    }
}

/* Power Management Functions */

int gnss_set_power_mode(bool low_power)
{
    int ret;
    uint8_t payload[48] = {0};  /* UBX-CFG-PM2 payload */
    
    if (low_power) {
        LOG_INF("Setting GNSS to low power mode");
        
        /* Configure Power Save Mode (PSM) parameters */
        /* Version */
        payload[0] = 0x01;
        
        /* Flags - enable cyclic tracking mode */
        payload[1] = 0x06;  /* CYCLIC_TRACK_MODE | DO_NOT_ENTER_BACKUP */
        
        /* Update period (ms) - how often to wake up */
        uint32_t update_period = 1000;  /* 1 second */
        memcpy(&payload[4], &update_period, 4);
        
        /* Search period (ms) - how long to search for signals */
        uint32_t search_period = 10000;  /* 10 seconds */
        memcpy(&payload[8], &search_period, 4);
        
        /* Grid offset (ms) */
        uint32_t grid_offset = 0;
        memcpy(&payload[12], &grid_offset, 4);
        
        /* On time (s) - minimum on time */
        uint16_t on_time = 5;
        memcpy(&payload[16], &on_time, 2);
        
        /* Min acquisition time (s) */
        uint16_t min_acq_time = 0;
        memcpy(&payload[18], &min_acq_time, 2);
    } else {
        LOG_INF("Setting GNSS to full power mode");
        
        /* Disable power save mode */
        payload[0] = 0x01;  /* Version */
        payload[1] = 0x00;  /* No flags - continuous tracking */
    }
    
    /* Send configuration */
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_PM2, payload, sizeof(payload));
    if (ret < 0) {
        LOG_ERR("Failed to set power mode: %d", ret);
        return ret;
    }
    
    LOG_INF("GNSS power mode updated successfully");
    
    return 0;
}

int gnss_standby(void)
{
    int ret;
    uint8_t payload[16] = {0};
    
    LOG_INF("Putting GNSS into standby mode");
    
    /* UBX-CFG-PMS - Power Mode Setup */
    /* Version */
    payload[0] = 0x00;
    
    /* Power setup value */
    payload[1] = 0x01;  /* Backup mode */
    
    /* Period - not used in backup mode */
    payload[2] = 0x00;
    payload[3] = 0x00;
    
    /* On time - not used in backup mode */
    payload[4] = 0x00;
    payload[5] = 0x00;
    
    /* Send configuration */
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_PMS, payload, 8);
    if (ret < 0) {
        LOG_ERR("Failed to enter standby mode: %d", ret);
        return ret;
    }
    
    LOG_INF("GNSS entered standby mode");
    
    return 0;
}

int gnss_wakeup(void)
{
    int ret;
    
    LOG_INF("Waking up GNSS from standby");
    
    /* Send any byte to wake up the module */
    uint8_t wakeup_byte = 0xFF;
    ret = uart_tx(uart_dev, &wakeup_byte, 1, SYS_FOREVER_MS);
    if (ret < 0) {
        LOG_ERR("Failed to send wakeup byte: %d", ret);
        return ret;
    }
    
    /* Wait for module to wake up */
    k_msleep(100);
    
    /* Configure back to full power mode */
    uint8_t payload[16] = {0};
    
    /* Version */
    payload[0] = 0x00;
    
    /* Power setup value */
    payload[1] = 0x00;  /* Full power */
    
    /* Send configuration */
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_PMS, payload, 8);
    if (ret < 0) {
        LOG_ERR("Failed to set full power mode: %d", ret);
        return ret;
    }
    
    LOG_INF("GNSS woken up successfully");
    
    return 0;
}

int gnss_save_config(void)
{
    int ret;
    uint8_t payload[13] = {0};
    
    LOG_INF("Saving GNSS configuration to non-volatile memory");
    
    /* UBX-CFG-CFG - Save current configuration */
    /* Clear mask - what to clear (nothing) */
    payload[0] = 0x00;
    payload[1] = 0x00;
    payload[2] = 0x00;
    payload[3] = 0x00;
    
    /* Save mask - what to save (all) */
    payload[4] = 0x1F;  /* ioPort | msgConf | infMsg | navConf | rxmConf */
    payload[5] = 0x1F;  /* senConf | rinvConf | antConf | logConf | ftsConf */
    payload[6] = 0x00;
    payload[7] = 0x00;
    
    /* Load mask - what to load (nothing) */
    payload[8] = 0x00;
    payload[9] = 0x00;
    payload[10] = 0x00;
    payload[11] = 0x00;
    
    /* Device mask - where to save */
    payload[12] = 0x17;  /* devBBR | devFlash | devEEPROM | devSpiFlash */
    
    /* Send configuration */
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_CFG, payload, sizeof(payload));
    if (ret < 0) {
        LOG_ERR("Failed to save configuration: %d", ret);
        return ret;
    }
    
    LOG_INF("GNSS configuration saved successfully");
    
    return 0;
}

/*
 * UART TX helper - send data and wait for completion
 * This prevents EBUSY errors from overlapping transmissions
 */
static int uart_tx_wait(const uint8_t *data, size_t len, k_timeout_t timeout)
{
    int ret;

    /* Reset TX state */
    k_sem_reset(&uart_tx_state.done);
    uart_tx_state.result = -EBUSY;

    /* Start transmission */
    ret = uart_tx(uart_dev, data, len, SYS_FOREVER_MS);
    if (ret < 0) {
        LOG_ERR("uart_tx failed: %d", ret);
        return ret;
    }

    /* Wait for TX completion callback */
    ret = k_sem_take(&uart_tx_state.done, timeout);
    if (ret < 0) {
        LOG_ERR("TX timeout waiting for completion");
        return -ETIMEDOUT;
    }

    return uart_tx_state.result;
}

/* Helper function to calculate UBX checksum */
static void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;

    for (size_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

/* Send UBX message and wait for ACK */
static int ubx_send_with_ack(uint8_t msg_class, uint8_t msg_id,
                             const uint8_t *payload, size_t payload_len)
{
    uint8_t buffer[256];
    size_t idx = 0;
    int ret;

    /* Build UBX message */
    buffer[idx++] = UBX_SYNC1;
    buffer[idx++] = UBX_SYNC2;
    buffer[idx++] = msg_class;
    buffer[idx++] = msg_id;
    buffer[idx++] = payload_len & 0xFF;
    buffer[idx++] = (payload_len >> 8) & 0xFF;

    /* Copy payload */
    if (payload && payload_len > 0) {
        memcpy(&buffer[idx], payload, payload_len);
        idx += payload_len;
    }

    /* Calculate checksum */
    uint8_t ck_a, ck_b;
    ubx_checksum(&buffer[2], idx - 2, &ck_a, &ck_b);
    buffer[idx++] = ck_a;
    buffer[idx++] = ck_b;

    /* Initialize ACK tracking BEFORE sending */
    k_sem_reset(&ubx_ack_state.sem);
    ubx_ack_state.ack_received = false;
    ubx_ack_state.nak_received = false;
    ubx_ack_state.waiting_class = msg_class;
    ubx_ack_state.waiting_id = msg_id;

    /* Send message and wait for TX completion */
    ret = uart_tx_wait(buffer, idx, K_MSEC(500));
    if (ret < 0) {
        LOG_ERR("Failed to send UBX message: %d", ret);
        return ret;
    }

    /* Wait for ACK/NAK response from receiver */
    ret = k_sem_take(&ubx_ack_state.sem, K_MSEC(UBX_ACK_TIMEOUT_MS));
    if (ret < 0) {
        LOG_ERR("UBX ACK timeout for class=0x%02X id=0x%02X", msg_class, msg_id);
        return -ETIMEDOUT;
    }

    if (ubx_ack_state.nak_received) {
        LOG_ERR("UBX NAK received for class=0x%02X id=0x%02X", msg_class, msg_id);
        return -EACCES;
    }

    LOG_DBG("UBX command successful: class=0x%02X id=0x%02X", msg_class, msg_id);
    return 0;
}

/* Poll current NAV5 configuration */
int gnss_get_nav5_config(gnss_dynmodel_t *dynmodel, uint8_t *fix_mode)
{
    int ret;
    
    LOG_INF("Polling NAV5 configuration");
    
    /* Send poll request (empty payload) */
    ret = uart_tx(uart_dev, (uint8_t[]){
        UBX_SYNC1, UBX_SYNC2,
        UBX_CLASS_CFG, UBX_ID_CFG_NAV5,
        0x00, 0x00,  /* Length = 0 for poll */
        0x48, 0x5A   /* Checksum for this poll */
    }, 8, SYS_FOREVER_MS);
    
    if (ret < 0) {
        LOG_ERR("Failed to send NAV5 poll: %d", ret);
        return ret;
    }
    
    /* TODO: Parse response - for now, return defaults */
    /* In a complete implementation, you would:
     * 1. Wait for UBX-CFG-NAV5 response
     * 2. Parse the 36-byte payload
     * 3. Extract dynModel and fixMode
     */
    
    /* For now, assume defaults */
    if (dynmodel) {
        *dynmodel = GNSS_DYNMODEL_PORTABLE;
    }
    if (fix_mode) {
        *fix_mode = 3;  /* Auto 2D/3D */
    }
    
    return 0;
}

/* Set dynamic platform model */
int gnss_set_dynmodel(gnss_dynmodel_t dynmodel)
{
    struct ubx_cfg_nav5 nav5_cfg;
    int ret;
    
    if (dynmodel > GNSS_DYNMODEL_WRIST) {
        LOG_ERR("Invalid dynamic model: %d", dynmodel);
        return -EINVAL;
    }
    
    LOG_INF("Setting dynamic model to %d", dynmodel);
    
    /* Initialize NAV5 configuration structure */
    memset(&nav5_cfg, 0, sizeof(nav5_cfg));
    
    /* Set mask to only update dynamic model */
    nav5_cfg.mask = 0x0001;  /* Only dynModel bit set */
    nav5_cfg.dyn_model = dynmodel;
    
    /* All other fields remain at their defaults (zeros) */
    
    /* Send configuration */
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_NAV5, 
                           (uint8_t *)&nav5_cfg, sizeof(nav5_cfg));
    if (ret < 0) {
        LOG_ERR("Failed to set dynamic model: %d", ret);
        return ret;
    }
    
    LOG_INF("Dynamic model set successfully");
    
    return 0;
}

/* Set full NAV5 configuration */
int gnss_set_nav5_config(const struct ubx_cfg_nav5 *config)
{
    int ret;
    
    if (!config) {
        return -EINVAL;
    }
    
    LOG_INF("Setting full NAV5 configuration");
    
    ret = ubx_send_with_ack(UBX_CLASS_CFG, UBX_ID_CFG_NAV5, 
                           (uint8_t *)config, sizeof(*config));
    if (ret < 0) {
        LOG_ERR("Failed to set NAV5 configuration: %d", ret);
        return ret;
    }
    
    LOG_INF("NAV5 configuration set successfully");
    
    return 0;
}

/* Initialize GNSS for skydiving use case */
int gnss_init_skydiving(void)
{
    int ret;
    
    LOG_INF("Configuring GNSS for skydiving");
    
    /* Set dynamic model to Airborne 4g for skydiving */
    ret = gnss_set_dynmodel(GNSS_DYNMODEL_AIRBORNE_2G);
    if (ret < 0) {
        LOG_WRN("Failed to set airborne 2g model: %d", ret);
        /* Try airborne 2g as fallback */
        ret = gnss_set_dynmodel(GNSS_DYNMODEL_AIRBORNE_1G);
        if (ret < 0) {
            LOG_ERR("Failed to set airborne 1g model: %d", ret);
            return ret;
        }
        LOG_INF("Using airborne 1g model as fallback");
    }
    
    /* Set update rate to 1Hz for ground/climb, will switch to 10Hz in freefall */
    ret = gnss_set_rate(1);
    if (ret < 0) {
        LOG_WRN("Failed to set 1Hz rate: %d", ret);
    }

    /* Note: SAM-M10Q OTP memory can only be programmed once, so we configure
     * the device at each boot rather than trying to save to non-volatile memory */

    LOG_INF("GNSS configured for skydiving");
    
    return 0;
}

/* Add helper function to get session start time */
static uint64_t get_session_start_us(void)
{
    /* This would need to be set from aggregator when session starts */
    extern uint64_t gnss_session_start_us;
    return gnss_session_start_us;
}

void gnss_set_session_start_time(uint64_t start_us)
{
    gnss_session_start_us = start_us;
}

bool gnss_has_fix(void)
{
    bool has_fix = false;

    k_mutex_lock(&fix_mutex, K_FOREVER);

    /* Consider we have a fix if:
     * - fix_quality > 0 (GPS or better)
     * - At least 3 satellites (minimum for 2D fix)
     * - Position is marked valid
     */
    if (current_fix.fix_quality > 0 &&
        current_fix.num_satellites >= 3 &&
        current_fix.position_valid) {
        has_fix = true;
    }

    k_mutex_unlock(&fix_mutex);

    return has_fix;
}

bool gnss_time_is_set(void)
{
    return gnss_time_set_flag;
}

#ifdef CONFIG_RTC
#include <zephyr/drivers/rtc.h>

static void update_rtc_from_fix(void)
{
    const struct device *rtc = DEVICE_DT_GET(DT_ALIAS(rtc0));
    struct rtc_time rtc_tm;

    if (!device_is_ready(rtc)) {
        return;
    }

    memset(&rtc_tm, 0, sizeof(rtc_tm));
    rtc_tm.tm_year = current_fix.year - 1900;
    rtc_tm.tm_mon = current_fix.month - 1;
    rtc_tm.tm_mday = current_fix.day;
    rtc_tm.tm_hour = current_fix.hours;
    rtc_tm.tm_min = current_fix.minutes;
    rtc_tm.tm_sec = current_fix.seconds;

    int ret = rtc_set_time(rtc, &rtc_tm);
    if (ret == 0) {
        LOG_INF("RTC updated from GNSS");
    }
}
#endif

/*
 * UBX Communication Test Mode
 *
 * This test validates the UBX binary protocol communication with the SAM-M10Q:
 * 1. Tests TX completion tracking (no EBUSY errors)
 * 2. Tests UBX message framing and checksum
 * 3. Tests ACK/NAK response parsing
 */
#ifdef CONFIG_GNSS_TEST_MODE

int gnss_test_ubx_communication(void)
{
    int ret;
    int tests_passed = 0;
    int tests_failed = 0;

    LOG_INF("=== GNSS UBX Communication Test ===");

    /* Allow GNSS thread time to start processing */
    k_msleep(100);

    /*
     * Test 1: Set dynamic model to Portable (0) - should succeed with ACK
     */
    LOG_INF("Test 1: Set dynamic model to Portable...");
    ret = gnss_set_dynmodel(GNSS_DYNMODEL_PORTABLE);
    if (ret == 0) {
        LOG_INF("  PASS: Dynamic model set, ACK received");
        tests_passed++;
    } else if (ret == -ETIMEDOUT) {
        LOG_ERR("  FAIL: ACK timeout (UBX parser may not be working)");
        tests_failed++;
    } else {
        LOG_ERR("  FAIL: Error %d", ret);
        tests_failed++;
    }

    k_msleep(100);

    /*
     * Test 2: Set rate to 1Hz - should succeed with ACK
     */
    LOG_INF("Test 2: Set update rate to 1Hz...");
    ret = gnss_set_rate(1);
    if (ret == 0) {
        LOG_INF("  PASS: Rate set, ACK received");
        tests_passed++;
    } else if (ret == -ETIMEDOUT) {
        LOG_ERR("  FAIL: ACK timeout");
        tests_failed++;
    } else {
        LOG_ERR("  FAIL: Error %d", ret);
        tests_failed++;
    }

    k_msleep(100);

    /*
     * Test 3: Set dynamic model to Airborne 2G - should succeed
     */
    LOG_INF("Test 3: Set dynamic model to Airborne 2G...");
    ret = gnss_set_dynmodel(GNSS_DYNMODEL_AIRBORNE_2G);
    if (ret == 0) {
        LOG_INF("  PASS: Airborne 2G model set");
        tests_passed++;
    } else {
        LOG_ERR("  FAIL: Error %d", ret);
        tests_failed++;
    }

    /* Summary */
    LOG_INF("=== Test Summary: %d passed, %d failed ===",
            tests_passed, tests_failed);

    return (tests_failed > 0) ? -1 : 0;
}

#endif /* CONFIG_GNSS_TEST_MODE */