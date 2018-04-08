/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <platform.h>

#include "common/utils.h"

#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1), .inputIrq = TIM1_CC_IRQn },
    { .TIMx = TIM2,  .rcc = RCC_APB1(TIM2), .inputIrq = TIM2_IRQn },
    { .TIMx = TIM3,  .rcc = RCC_APB1(TIM3), .inputIrq = TIM3_IRQn },
    { .TIMx = TIM4,  .rcc = RCC_APB1(TIM4), .inputIrq = TIM4_IRQn },
};

uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return SystemCoreClock;
}
