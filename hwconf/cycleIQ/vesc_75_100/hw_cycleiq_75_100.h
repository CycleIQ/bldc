    /*
    Copyright 2018 Benjamin Vedder	benjamin@vedder.se

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HW_CYCLEIQ_75_100_H_
#define HW_CYCLEIQ_75_100_H_

#include "hw_cycleiq_75_100_core.h"

#define CYCLEIQ_HIGH_POWER
#define CYCLEIQ_HAS_2_WIRE_PAS

#define PAS_WIRE_A_BANK GPIOB
#define PAS_WIRE_A_PIN 11
#define PAS_EXTI_LINE_A EXTI_Line11
#define PAS_WIRE_B_BANK GPIOB
#define PAS_EXTI_LINE_B EXTI_Line10
#define PAS_WIRE_B_PIN 10

#define TS_INDEX ADC_IND_EXT2

#endif /* HW_CYCLEIQ_75_100_H_ */
