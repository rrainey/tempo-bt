/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Event Bus Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "app/events.h"

LOG_MODULE_REGISTER(event_bus, LOG_LEVEL_INF);

/* Event queue and processing thread */
#define EVENT_QUEUE_SIZE 16
#define EVENT_THREAD_STACK_SIZE 2048
#define EVENT_THREAD_PRIORITY 7

K_MSGQ_DEFINE(event_queue, sizeof(app_event_t), EVENT_QUEUE_SIZE, 4);
K_THREAD_STACK_DEFINE(event_thread_stack, EVENT_THREAD_STACK_SIZE);

static struct k_thread event_thread_data;
static k_tid_t event_thread_id;

/* Subscriber list */
static event_subscriber_t *subscriber_list = NULL;
static struct k_mutex subscriber_mutex;

/* Event type names for logging */
static const char *event_names[] = {
    [EVT_SESSION_START] = "SESSION_START",
    [EVT_SESSION_STOP] = "SESSION_STOP",
    [EVT_STATE_CHANGE] = "STATE_CHANGE",
    [EVT_STORAGE_LOW] = "STORAGE_LOW",
    [EVT_STORAGE_ERROR] = "STORAGE_ERROR",
    [EVT_UPLOAD_START] = "UPLOAD_START",
    [EVT_UPLOAD_DONE] = "UPLOAD_DONE",
    [EVT_DFU_START] = "DFU_START",
    [EVT_DFU_DONE] = "DFU_DONE",
    [EVT_SENSOR_ERROR] = "SENSOR_ERROR",
    [EVT_GNSS_FIX_ACQUIRED] = "GNSS_FIX_ACQUIRED",
    [EVT_GNSS_FIX_LOST] = "GNSS_FIX_LOST",
    [EVT_TEST_DUMMY] = "TEST_DUMMY"
};

/* Event processing thread */
static void event_thread_func(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    app_event_t event;
    
    LOG_INF("Event bus thread started");
    
    while (1) {
        /* Wait for event */
        if (k_msgq_get(&event_queue, &event, K_FOREVER) == 0) {
            LOG_DBG("Processing event: %s", 
                    event.type < ARRAY_SIZE(event_names) ? event_names[event.type] : "UNKNOWN");
            
            /* Lock subscriber list */
            k_mutex_lock(&subscriber_mutex, K_FOREVER);
            
            /* Notify all interested subscribers */
            event_subscriber_t *sub = subscriber_list;
            int count = 0;
            while (sub) {
                /* Check if subscriber wants this event type */
                if (sub->event_mask & (1U << event.type)) {
                    if (sub->handler) {
                        sub->handler(&event, sub->user_data);
                        count++;
                    }
                }
                sub = sub->next;
            }
            
            if (count == 0) {
                LOG_DBG("No subscribers for event %s", 
                        event.type < ARRAY_SIZE(event_names) ? event_names[event.type] : "UNKNOWN");
            }
            
            k_mutex_unlock(&subscriber_mutex);
        }
    }
}

int event_bus_init(void)
{
    /* Initialize mutex */
    k_mutex_init(&subscriber_mutex);
    
    /* Start event processing thread */
    event_thread_id = k_thread_create(&event_thread_data,
                                     event_thread_stack,
                                     K_THREAD_STACK_SIZEOF(event_thread_stack),
                                     event_thread_func,
                                     NULL, NULL, NULL,
                                     EVENT_THREAD_PRIORITY, 0, K_NO_WAIT);
    
    k_thread_name_set(event_thread_id, "event_bus");
    
    LOG_INF("Event bus initialized");
    
    return 0;
}

int event_bus_subscribe(event_subscriber_t *subscriber,
                       event_handler_t handler,
                       uint32_t event_mask,
                       void *user_data)
{
    if (!subscriber || !handler) {
        return -EINVAL;
    }
    
    /* Initialize subscriber */
    subscriber->handler = handler;
    subscriber->event_mask = event_mask;
    subscriber->user_data = user_data;
    subscriber->next = NULL;
    
    /* Add to list */
    k_mutex_lock(&subscriber_mutex, K_FOREVER);
    
    if (!subscriber_list) {
        subscriber_list = subscriber;
    } else {
        /* Find end of list */
        event_subscriber_t *last = subscriber_list;
        while (last->next) {
            last = last->next;
        }
        last->next = subscriber;
    }
    
    k_mutex_unlock(&subscriber_mutex);
    
    LOG_DBG("Subscriber added, mask: 0x%08x", event_mask);
    
    return 0;
}

int event_bus_unsubscribe(event_subscriber_t *subscriber)
{
    if (!subscriber) {
        return -EINVAL;
    }
    
    k_mutex_lock(&subscriber_mutex, K_FOREVER);
    
    if (subscriber_list == subscriber) {
        /* Remove from head */
        subscriber_list = subscriber->next;
    } else {
        /* Find and remove from list */
        event_subscriber_t *prev = subscriber_list;
        while (prev && prev->next != subscriber) {
            prev = prev->next;
        }
        if (prev) {
            prev->next = subscriber->next;
        }
    }
    
    k_mutex_unlock(&subscriber_mutex);
    
    LOG_DBG("Subscriber removed");
    
    return 0;
}

int event_bus_publish(const app_event_t *event)
{
    if (!event) {
        return -EINVAL;
    }
    
    /* Add timestamp if not set */
    app_event_t evt = *event;
    if (evt.timestamp_ms == 0) {
        evt.timestamp_ms = k_uptime_get_32();
    }
    
    /* Queue event */
    int ret = k_msgq_put(&event_queue, &evt, K_NO_WAIT);
    if (ret != 0) {
        LOG_WRN("Event queue full, dropping event: %s",
                evt.type < ARRAY_SIZE(event_names) ? event_names[evt.type] : "UNKNOWN");
        return ret;
    }
    
    LOG_DBG("Published event: %s", 
            evt.type < ARRAY_SIZE(event_names) ? event_names[evt.type] : "UNKNOWN");
    
    return 0;
}

int event_bus_publish_simple(event_type_t type)
{
    app_event_t event = {
        .type = type,
        .timestamp_ms = k_uptime_get_32()
    };
    
    return event_bus_publish(&event);
}