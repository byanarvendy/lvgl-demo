#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "lvgl.h"
#include "lvgl_helpers.h"

#if defined CONFIG_USE_LV_TOUCH_CALIBRATION
    #include "lv_tc.h"
    #include "lv_tc_screen.h"

    #ifndef USE_CUSTOM_LV_TC_COEFFICIENTS
        #include "esp_nvs_tc.h"
    #endif

#endif

#define LV_TICK_PERIOD_MS 1

static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);

void app_main() {
    xTaskCreatePinnedToCore(guiTask, "gui", 4096 * 2, NULL, 0, NULL, 1);
}

SemaphoreHandle_t xGuiSemaphore;

static void lv_tick_task(void *arg) {
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void event_cb(lv_event_t * e) {
    // inisialisasi event
    lv_event_code_t code = lv_event_get_code(e);

    // inisialisasi label
    lv_obj_t * label = lv_label_create(lv_scr_act());

    // membuat suatu text label baru ketika button di tekan
    lv_label_set_text(label, "Tombol telah diklik");
    lv_obj_center(label);
}

void button(void) {
    // inisialisasi button
    lv_obj_t * btn = lv_btn_create(lv_scr_act());           
    lv_obj_center(btn);                             // buat posisi button berada di tengah                       
    lv_obj_set_size(btn, 120, 50);                  // mengatur ukuran button                 

    // inisialisasi text (label) dalam button
    lv_obj_t * label = lv_label_create(btn);                
    lv_label_set_text(label, "Tekan disini!");                
    lv_obj_center(label);

    // memberi event (click) pada button 
    lv_obj_add_event_cb(btn, event_cb, LV_EVENT_CLICKED, NULL);
}

static void guiTask(void *pvParameter) {
    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();
    lvgl_driver_init();
    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_draw_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

#if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
#endif

#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb = disp_driver_set_px;
    disp_drv.antialiasing = 0;
#endif

    disp_drv.hor_res = CONFIG_LV_HOR_RES_MAX;
    disp_drv.ver_res = CONFIG_LV_VER_RES_MAX;
    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
#if CONFIG_USE_LV_TOUCH_CALIBRATION // if using LVGL Touch Calibration
    lv_tc_indev_drv_init(&indev_drv, touch_driver_read);
#ifndef CONFIG_USE_CUSTOM_LV_TC_COEFFICIENTS // if NOT using custom calibration coefficients
    lv_tc_register_coeff_save_cb(esp_nvs_tc_coeff_save_cb);
#endif
#else // if NOT using LVGL Touch Calibration
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
#endif
    lv_indev_drv_register(&indev_drv);
#endif

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &lv_tick_task,
            .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));


    /* YOUR LVGL DISPLAY FUNCTION HERE*/ 

    button();           // panggil fungsi yang telah dibuat

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }

    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}