//*****************************************************************************
// Copyright (c) 2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This file was automatically generated by the Tiva C Series PinMux Utility
// Version: 1.0.2
//
//*****************************************************************************

#ifndef __MAME_PINS_H__
#define __MAME_PINS_H__

extern void PortFunctionInit(void);

// PORT A
#define GPIO_P2_BASE       		GPIO_PORTA_BASE
								//0 and 1 DEBUG
#define GPIO_P2_A				GPIO_PIN_2
#define GPIO_P2_B				GPIO_PIN_3
#define GPIO_P2_C				GPIO_PIN_4
#define GPIO_P2_D				GPIO_PIN_5
#define GPIO_P2_E				GPIO_PIN_6
#define GPIO_P2_F				GPIO_PIN_7

//PORT E
#define GPIO_JOY2_BASE          GPIO_PORTE_BASE
#define GPIO_P2_START			GPIO_PIN_0
#define GPIO_P2_COIN			GPIO_PIN_1
#define GPIO_JOY2_UP			GPIO_PIN_2
#define GPIO_JOY2_DOWN			GPIO_PIN_3
#define GPIO_JOY2_RIGHT			GPIO_PIN_4
#define GPIO_JOY2_LEFT			GPIO_PIN_5

// PORT B
#define GPIO_P1_BASE       		GPIO_PORTB_BASE
#define GPIO_P1_A				GPIO_PIN_0
#define GPIO_P1_B				GPIO_PIN_1
#define GPIO_P1_C				GPIO_PIN_2
#define GPIO_P1_D				GPIO_PIN_3
#define GPIO_P1_E				GPIO_PIN_4
#define GPIO_P1_F				GPIO_PIN_5
#define GPIO_P1_START			GPIO_PIN_6
#define GPIO_P1_COIN			GPIO_PIN_7

//PORT C
#define GPIO_TRK_Y_BASE			GPIO_PORTC_BASE
								//0-3 DEBUG
#define GPIO_TRK_Y_A_QEI		GPIO_PC5_PHA1
#define GPIO_TRK_Y_A			GPIO_PIN_5
#define GPIO_TRK_Y_B_QEI		GPIO_PC6_PHB1
#define GPIO_TRK_Y_B			GPIO_PIN_6

// PORT D
#define GPIO_JOY1_BASE          GPIO_PORTD_BASE
#define GPIO_JOY1_UP			GPIO_PIN_3
#define GPIO_JOY1_DOWN			GPIO_PIN_2
#define GPIO_JOY1_RIGHT			GPIO_PIN_0
#define GPIO_JOY1_LEFT			GPIO_PIN_1
								//4 USB
								//5 USB
#define GPIO_TRK_X_BASE			GPIO_PORTD_BASE
#define GPIO_TRK_X_A_QEI		GPIO_PD6_PHA0
#define GPIO_TRK_X_A			GPIO_PIN_6
#define GPIO_TRK_X_B_QEI		GPIO_PD7_PHB0
#define GPIO_TRK_X_B			GPIO_PIN_7
#define QEI_TRK_X 				QEI0_BASE
#define QEI_TRK_Y 				QEI1_BASE

//PORT F
								//0 - BUTTON
								//1 - LED
								//2 - LED
								//3 - LED
								//4 - BUTTON

#endif //  __MAME_PINS_H__
