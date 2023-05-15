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

#ifndef EI_BLUETOOTH_PSOC63_H_
#define EI_BLUETOOTH_PSOC63_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/* BLE stack for Infineon PSoC 6 requires FreeRTOS */
#ifdef FREERTOS_ENABLED
#include <FreeRTOS.h>
#include <timers.h>
#endif

enum ble_char_index
{
    CLASS_RESULT = 0,
    INFERENCE = 1,
    SETTINGS = 2
};

cy_rslt_t ei_bluetooth_init(void);
void bt_app_send_notification(uint8_t index);


#endif /* EI_BLUETOOTH_PSOC63_H_ */
