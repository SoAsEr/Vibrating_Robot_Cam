

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_camera.h"

// apriltag
#include "apriltag.h"
#include "tag16h5.h"
#include "common/image_u8.h"
#include "common/zarray.h"


#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

#define CAM_XCLK_FREQ   20000000

#define TAG "main"

static camera_config_t camera_config = {
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_VGA,//Do not use sizes above QVGA when not JPEG
    //.jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1 //if more than one, i2s runs in continuous mode.

};

esp_err_t camera_init(){
    //power up the camera if PWDN pin is defined
    if(PWDN_GPIO_NUM != -1){
        gpio_pad_select_gpio(PWDN_GPIO_NUM);
        // Set the GPIO as a push/pull output
        gpio_set_direction(PWDN_GPIO_NUM, GPIO_MODE_OUTPUT);
        gpio_set_level(PWDN_GPIO_NUM, 0);
    }

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        printf("Camera Init Failed\n");
        return err;
    }

    return ESP_OK;
}
void app_main(){
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag16h5_create();
    apriltag_detector_add_family(td, tf);

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
    while(1){
        //acquire a frame
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera Capture Failed");
            return;
        }

        image_u8_t im = {
            .width = fb->width,
            .height = fb->height,
            .stride = fb->width,
            .buf = fb->buf
        };
        zarray_t *detections = apriltag_detector_detect(td, &im);
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            printf("detection %d ",det->id);
        }
        printf("\n");

        apriltag_detections_destroy(detections);
        //return the frame buffer back to the driver for reuse

        esp_camera_fb_return(fb);
        double t =  timeprofile_total_utime(td->tp) / 1.0E3;
        printf("%12.3f \n", t);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    // Cleanup.
    tag16h5_destroy(tf);
    apriltag_detector_destroy(td);
}