/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Event Bus System
 */

#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include <stdint.h>
#include <zephyr/kernel.h>

/* Event types */
typedef enum {
    /* Session events */
    EVT_SESSION_START,
    EVT_SESSION_STOP,
    
    /* Logging state machine change */
    EVT_STATE_CHANGE,
    
    /* Storage events */
    EVT_STORAGE_LOW,
    EVT_STORAGE_ERROR,
    
    /* Upload/DFU events */
    EVT_UPLOAD_START,
    EVT_UPLOAD_DONE,
    EVT_DFU_START,
    EVT_DFU_DONE,
    
    /* Sensor events */
    EVT_SENSOR_ERROR,
    EVT_GNSS_FIX_ACQUIRED,
    EVT_GNSS_FIX_LOST,

    EVT_BLE_CONNECTED,
    EVT_BLE_DISCONNECTED,

    /* Test event */
    EVT_TEST_DUMMY,
    
    EVT_TYPE_COUNT
    
} event_type_t;

/* Event data structure */
typedef struct {
    event_type_t type;
    uint32_t timestamp_ms;
    union {
        /* Mode change data */
        struct {
            uint8_t old_mode;
            uint8_t new_mode;
        } mode_change;
        
        /* Storage data */
        struct {
            uint8_t free_percent;
        } storage;
        
        /* Error data */
        struct {
            int error_code;
            uint8_t source;  /* Which subsystem */
        } error;
        
        /* Generic data */
        uint32_t data[4];

        struct {
            uint32_t id;
        } session;

        struct {
            uint8_t old_state;
            uint8_t new_state;
        } state_change; 
    } payload;
} app_event_t;

/* Event handler callback type */
typedef void (*event_handler_t)(const app_event_t *event, void *user_data);

/* Subscriber structure */
typedef struct event_subscriber {
    event_handler_t handler;
    void *user_data;
    uint32_t event_mask;  /* Bitmask of event types to receive */
    struct event_subscriber *next;
} event_subscriber_t;

/**
 * @brief Initialize the event bus
 * 
 * @return 0 on success, negative error code on failure
 */
int event_bus_init(void);

/**
 * @brief Subscribe to events
 * 
 * @param subscriber Subscriber structure (must remain valid)
 * @param handler Event handler callback
 * @param event_mask Bitmask of event types to receive (1 << EVT_xxx)
 * @param user_data Optional user data passed to handler
 * @return 0 on success, negative error code on failure
 */
int event_bus_subscribe(event_subscriber_t *subscriber,
                       event_handler_t handler,
                       uint32_t event_mask,
                       void *user_data);

/**
 * @brief Unsubscribe from events
 * 
 * @param subscriber Subscriber to remove
 * @return 0 on success, negative error code on failure
 */
int event_bus_unsubscribe(event_subscriber_t *subscriber);

/**
 * @brief Publish an event
 * 
 * @param event Event to publish (will be copied)
 * @return 0 on success, negative error code on failure
 */
int event_bus_publish(const app_event_t *event);

/**
 * @brief Helper to create and publish an event
 * 
 * @param type Event type
 * @return 0 on success, negative error code on failure
 */
int event_bus_publish_simple(event_type_t type);

#endif /* APP_EVENTS_H */