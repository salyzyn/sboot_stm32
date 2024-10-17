/* This file is the part of the STM32 secure bootloader
 *
 * Copyright ©2024 Mark Salyzyn <mark.salyzyn[at]gmail[dot]com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _USART_H_
#define _USART_H_

#include <stdbool.h>

#if defined(__cplusplus)
    extern "C" {
#endif

/** @brief Initialize/Decommission USART
 */
void usart_init(void);
void usart_deinit(void);

/** @brief run STM32 bootblock serial protocol state machine for USART
 */
void usart_poll(void);

#if defined(__cplusplus)
    }
#endif

#endif /* _USART_H_ */
