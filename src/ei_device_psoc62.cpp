/*
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


#include <string>

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "ei_device_psoc62.h"
#include "ei_flash_memory.h"
#include "ei_microphone.h"
#include "cy_syslib.h"
#include "cyhal_gpio.h"

#ifdef FREERTOS_ENABLED
#include <FreeRTOS.h>
#include <timers.h>
#endif


/******
 *
 * @brief EdgeImpulse Device structure and information
 *
 ******/

#define FLASH_DATA_AFTER_ERASE  0x00
#define FLASH_TEST_SIZE (512u)
#define FLASH_TEST_ADDR (0x1000)

#ifndef FREERTOS_ENABLED /* bare-metal case */
#define PERIODIC_TIMER_CLOCK_HZ (1000000) /* 1 MHz */
#define PERIODIC_TIMER_PRIORITY 7
#else
/** Global objects */
TimerHandle_t fusion_timer;
TimerHandle_t led_timer;
void (*sample_cb_ptr)(void);
/* Private function declarations ------------------------------------------- */
void vTimerCallback(TimerHandle_t xTimer);
void vLedCallback(TimerHandle_t xTimer);
#endif

typedef void (*timer_callback_t) (void*, cyhal_timer_event_t);

void led_handler(void *callback_arg, cyhal_timer_event_t event)
{
    EiDevicePSoC62 *dev = static_cast<EiDevicePSoC62*>(callback_arg);
    EiState state =dev->get_state();
    static uint8_t animation = 0;

    switch(state)
    {
        case eiStateErasingFlash:
            cyhal_gpio_toggle(CYBSP_USER_LED1);
            break;
        case eiStateSampling:
            cyhal_gpio_toggle(CYBSP_USER_LED2);
            break;
        case eiStateUploading:
            cyhal_gpio_toggle(CYBSP_USER_LED1);
            cyhal_gpio_toggle(CYBSP_USER_LED2);
            break;
        case eiStateFinished:
            if(animation == 0) {
                animation = 10;
            }
            break;
        default:
            break;
    }

    if(animation == 0) {
        return;
    }


    switch(animation) {
        case 10:
            cyhal_gpio_write(CYBSP_USER_LED1, !0);
            cyhal_gpio_write(CYBSP_USER_LED2, !0);
            break;
        case 9:
            cyhal_gpio_write(CYBSP_USER_LED1, !0);
            cyhal_gpio_write(CYBSP_USER_LED2, !1);
            break;
        case 8:
            cyhal_gpio_write(CYBSP_USER_LED1, !1);
            cyhal_gpio_write(CYBSP_USER_LED2, !0);
            break;
        case 7:
            cyhal_gpio_write(CYBSP_USER_LED1, !1);
            cyhal_gpio_write(CYBSP_USER_LED2, !1);
            break;
        case 6:
            cyhal_gpio_write(CYBSP_USER_LED1, !1);
            cyhal_gpio_write(CYBSP_USER_LED2, !0);
            break;
        case 5:
            cyhal_gpio_write(CYBSP_USER_LED1, !0);
            cyhal_gpio_write(CYBSP_USER_LED2, !0);
            break;
        case 4:
            cyhal_gpio_write(CYBSP_USER_LED1, !1);
            cyhal_gpio_write(CYBSP_USER_LED2, !1);
            break;
        case 3:
            cyhal_gpio_write(CYBSP_USER_LED1, !0);
            cyhal_gpio_write(CYBSP_USER_LED2, !0);
            break;
        case 2:
            cyhal_gpio_write(CYBSP_USER_LED1, !1);
            cyhal_gpio_write(CYBSP_USER_LED2, !1);
            break;
        case 1:
            dev->set_state(eiStateIdle);
            break;
    }
    animation--;
}

EiDevicePSoC62::EiDevicePSoC62(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;
    cy_rslt_t result;

    init_device_id();

    load_config();

    device_type = "INFINEON_PSOC63";

    // init LEDs
    cyhal_gpio_configure(CYBSP_USER_LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);
    cyhal_gpio_configure(CYBSP_USER_LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG);

#ifdef FREERTOS_ENABLED
    // create LED timer
    led_timer = xTimerCreate(
                            "led timer",
                            (uint32_t)250 / portTICK_PERIOD_MS,
                            pdTRUE,
                            this,
                            vLedCallback
                        );
    xTimerStart(led_timer, 0);
    // sample timer is created in start_sample_thread()
#else /* bare-metal */
    // create LED timer
    const cyhal_timer_cfg_t led_timer_cfg =
    {
        .is_continuous = true, /* use cyhal_timer_start/stop to control */
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = false,
        .period = 250 * 1000, // 250 ms
        .compare_value = 0,
        .value = 0
    };
    cyhal_timer_init(&led_timer, NC, NULL);
    cyhal_timer_configure(&led_timer, &led_timer_cfg);
    cyhal_timer_set_frequency(&led_timer, PERIODIC_TIMER_CLOCK_HZ);
    cyhal_timer_register_callback(&led_timer, led_handler, this);
    cyhal_timer_enable_event(&led_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 7, true);

    // pre-configure sample timer
    sample_timer_cfg.is_continuous = true;
    sample_timer_cfg.direction = CYHAL_TIMER_DIR_UP;
    sample_timer_cfg.is_compare = false;
    sample_timer_cfg.period = 100 * 1000;
    sample_timer_cfg.compare_value = 0;
    sample_timer_cfg.value = 0;
    cyhal_timer_init(&sample_timer, NC, NULL);
    cyhal_timer_configure(&sample_timer, &sample_timer_cfg);
    cyhal_timer_set_frequency(&sample_timer, PERIODIC_TIMER_CLOCK_HZ);
    cyhal_timer_enable_event(&sample_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 7, true);
#endif

    sensors[EI_STANDALONE_SENSOR_MIC].name = "Microphone";
    sensors[EI_STANDALONE_SENSOR_MIC].start_sampling_cb = ei_microphone_sample_start;
    sensors[EI_STANDALONE_SENSOR_MIC].frequencies[0] = 8000.0f;
    sensors[EI_STANDALONE_SENSOR_MIC].frequencies[1] = 16000.0f;
    sensors[EI_STANDALONE_SENSOR_MIC].frequencies[2] = 32000.0f;
    sensors[EI_STANDALONE_SENSOR_MIC].max_sample_length_s = mem->get_available_sample_bytes() / (sensors[EI_STANDALONE_SENSOR_MIC].frequencies[0] * 2);
}

EiDevicePSoC62::~EiDevicePSoC62()
{

}

EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    /* Initializing EdgeImpulse classes here in order for
     * QSPI and other PSoC6 peripherals to be initialized.
     */
    static EiFlashMemory memory(sizeof(EiConfig));
    static EiDevicePSoC62 dev(&memory);

    return &dev;
}

void EiDevicePSoC62::init_device_id(void)
{
    uint64_t id;
    char temp[18];

    /*
    * [63:57] - DIE_YEAR
    * [56:56] - DIE_MINOR
    * [55:48] - DIE_SORT
    * [47:40] - DIE_Y
    * [39:32] - DIE_X
    * [31:24] - DIE_WAFER
    * [23:16] - DIE_LOT[2]
    * [15: 8] - DIE_LOT[1]
    * [ 7: 0] - DIE_LOT[0]
    */
    id = Cy_SysLib_GetUniqueId();

    snprintf(temp, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
            (uint8_t)(id>>56),
            (uint8_t)(id>>48),
            (uint8_t)(id>>40),
            (uint8_t)(id>>16),
            (uint8_t)(id>>8),
            (uint8_t)(id));

    device_id = std::string(temp);
}

/**
 *
 * @param sensor_list
 * @param sensor_list_size
 * @return
 */
bool EiDevicePSoC62::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    *sensor_list      = sensors;
    *sensor_list_size = EI_STANDALONE_SENSORS_COUNT;

    return true;
}

void EiDevicePSoC62::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

uint32_t EiDevicePSoC62::get_data_output_baudrate(void)
{
    /* Using EI_DEVICE_BAUDRATE_MAX and speeds above 115k baudrate
     * requires a firmware update of the KitProg3. Most Pioneer Kits
     * come with a KitProg3 firmware that supports speeds up to 115k.
     */
    return EI_DEVICE_BAUDRATE;
}

/**
 * @brief      Set output baudrate to max
 *
 */
void EiDevicePSoC62::set_max_data_output_baudrate()
{
    cy_rslt_t result;

    result = cyhal_uart_set_baud(&cy_retarget_io_uart_obj, EI_DEVICE_BAUDRATE_MAX, NULL);
    if(result != CY_RSLT_SUCCESS) {
        ei_printf("ERR: Failed to change baudrate to %d\n", EI_DEVICE_BAUDRATE_MAX);
    }
}

/**
 * @brief      Set output baudrate to default
 *
 */
void EiDevicePSoC62::set_default_data_output_baudrate()
{
    cy_rslt_t result;

    result = cyhal_uart_set_baud(&cy_retarget_io_uart_obj, EI_DEVICE_BAUDRATE, NULL);
    if(result != CY_RSLT_SUCCESS) {
        ei_printf("ERR: Failed to change baudrate to %d\n", EI_DEVICE_BAUDRATE);
    }
}

bool EiDevicePSoC62::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    cy_rslt_t result;
    bool ret = false;

#ifdef FREERTOS_ENABLED
    sample_cb_ptr = sample_read_cb;
    fusion_timer = xTimerCreate("Fusion sampler",
                                (uint32_t)sample_interval_ms / portTICK_PERIOD_MS,
                                pdTRUE, (void *) 0, vTimerCallback);

    if(xTimerStart(fusion_timer, 0) == pdPASS) {
#else
    /* Assign the ISR to execute on timer interrupt */
    sample_timer_cfg.period = sample_interval_ms * 1000;
    cyhal_timer_configure(&sample_timer, &sample_timer_cfg);
    cyhal_timer_register_callback(&sample_timer, (timer_callback_t)sample_read_cb, nullptr);
    result = cyhal_timer_start(&sample_timer);
    if (result == CY_RSLT_SUCCESS) {
#endif
        if(this->is_environmental_sampling()) {
            /* Workaround for ADC issue */
            //ei_environment_sensor_async_trigger();
        }
        ret = true;
        this->set_state(eiStateSampling);
    }
    else {
        ei_printf("ERR: Failed to start sample timer.\n");
    }

    return ret;
}

bool EiDevicePSoC62::stop_sample_thread(void)
{
#ifdef FREERTOS_ENABLED
    if (xTimerStop(fusion_timer, 0) != pdPASS)
    {
        ei_printf("ERR: timer has not been stopped \n");
    }
#else
    cyhal_timer_stop(&sample_timer);
#endif
    this->set_state(eiStateIdle);

    return true;
}

void EiDevicePSoC62::set_state(EiState state)
{
    this->state = state;

    // state 1 = LED off
    cyhal_gpio_write(CYBSP_USER_LED1, 1);
    cyhal_gpio_write(CYBSP_USER_LED2, 1);

    switch(state) {
        case eiStateErasingFlash:
        case eiStateSampling:
        case eiStateUploading:
        case eiStateFinished:
#ifdef FREERTOS_ENABLED
            xTimerStart(led_timer, 0);
#else
            cyhal_timer_start(&led_timer);
#endif
            break;
        case eiStateIdle:
        default:
#ifdef FREERTOS_ENABLED
            xTimerStop(led_timer, 0);
#else
            cyhal_timer_stop(&led_timer);
#endif
            break;
    }
}

EiState EiDevicePSoC62::get_state(void)
{
    return this->state;
}

void EiDevicePSoC62::set_environmental_sampling(void)
{
    this->environmental_sampling = true;
}

void EiDevicePSoC62::clear_environmental_sampling(void)
{
    this->environmental_sampling = false;
}

bool EiDevicePSoC62::is_environmental_sampling(void)
{
    return this->environmental_sampling;
}

#ifdef FREERTOS_ENABLED
void vTimerCallback(TimerHandle_t xTimer)
{
    sample_cb_ptr();
}

void vLedCallback(TimerHandle_t xTimer) {
    led_handler(pvTimerGetTimerID(xTimer), (cyhal_timer_event_t)0);
}
#endif
