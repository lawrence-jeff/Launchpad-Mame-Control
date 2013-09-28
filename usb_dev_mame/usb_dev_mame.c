//*****************************************************************************
//
// usb_dev_mame.c - Main routines for a Mame control device.
//
// Copyright (c) 2011-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 1.0 of the EK-LM4F232 Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/qei.h"
#include "driverlib/interrupt.h"
#include "usblib/usblib.h"
#include "usblib/usbhid.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdhid.h"
#include "usblib/device/usbdhidmame.h"
#include "usb_mame_structs.h"
#include "Mame_pins.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB HID Mame Control Device (usb_dev_mame)</h1>
//!
//! This example application turns the evaluation board into a USB volume control
//! supporting the Human Interface Device class.  When the left button is
//! pressed the volume is increased, when the right button is pressed the volume is decreased
//!
//! The device implemented by this application also supports USB remote wakeup
//! allowing it to request the host to reactivate a suspended bus.  If the bus
//! is suspended (as indicated on the application display), pressing the
//! push button will request a remote wakeup assuming the host has not
//! specifically disabled such requests.
//!
//
//*****************************************************************************

//*****************************************************************************
//
// The system tick timer period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     1000  // 1ms systick rate

//*****************************************************************************
//
// The number of checks for switch debouncing.
//
//*****************************************************************************
#define MAX_CHECKS 5

//*****************************************************************************
//
// Pre-scalar for mouse input.
//
//*****************************************************************************
#define MOUSE_SCALAR 2

//*****************************************************************************
//
// This global indicates whether or not we are connected to a USB host.
//
//*****************************************************************************
volatile bool g_bConnected = false;

//*****************************************************************************
//
// This global indicates whether or not the USB bus is currently in the suspend
// state.
//
//*****************************************************************************
volatile bool g_bSuspended = false;

//*****************************************************************************
//
// Global system tick counter holds elapsed time since the application started
// expressed in 100ths of a second.
//
//*****************************************************************************
volatile uint32_t g_ui32SysTickCount;

//*****************************************************************************
//
// Global button arrays and ticks hold button data from each loop for debouncing
//
//*****************************************************************************
volatile signed char g_ui8Pad1LastSent[2];
volatile signed char g_ui8Pad2LastSent[2];
volatile signed char g_ui8MouseLastSent[3];
volatile signed char g_ui8Pad1_1State[MAX_CHECKS];
volatile signed char g_ui8Pad1_2State[MAX_CHECKS];
volatile signed char g_ui8Pad2_1State[MAX_CHECKS];
volatile signed char g_ui8Pad2_2State[MAX_CHECKS];
volatile signed char g_ui8Mouse_State[MAX_CHECKS];
volatile signed char g_ui8IndexDebounce;
signed char g_ui8Pad1_Debounced[3];
signed char g_ui8Pad2_Debounced[2];
signed char g_ui8Mouse_Debounced[3];

//*****************************************************************************
//
// Global variable indicating if the board is in programing or GPIO mode.
//
//*****************************************************************************
volatile bool g_bProgramMode;

//*****************************************************************************
//
// This enumeration holds the various states that the device can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
    //
    // Unconfigured.
    //
    STATE_UNCONFIGURED,

    //
    // No keys to send and not waiting on data.
    //
    STATE_IDLE,

    //
    // Waiting on data to be sent out.
    //
    STATE_SENDING
}
g_eCustomHidState = STATE_UNCONFIGURED;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Handles asynchronous events from the HID driver.
//
// \param pvCBData is the event callback pointer provided during
// USBDHIDCustomHidInit().  This is a pointer to our device structure
// (&g_sCustomHidDevice).
// \param ui32Event identifies the event we are being called back for.
// \param ui32MsgData is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the HID driver to inform the application
// of particular asynchronous events related to operation of the HID
// device.
//
// \return Returns 0 in all cases.
//
//*****************************************************************************
uint32_t
CustomHidHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgData,
                void *pvMsgData)
{
    switch (ui32Event)
    {
        //
        // The host has connected to us and configured the device.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bConnected = true;
            g_bSuspended = false;
            break;
        }

        //
        // The host has disconnected from us.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bConnected = false;
            break;
        }

        //
        // We receive this event every time the host acknowledges transmission
        // of a report. It is used here purely as a way of determining whether
        // the host is still talking to us or not.
        //
        case USB_EVENT_TX_COMPLETE:
        {
            //
            // Enter the idle state since we finished sending something.
            //
            g_eCustomHidState = STATE_IDLE;
            break;
        }

        //
        // This event indicates that the host has suspended the USB bus.
        //
        case USB_EVENT_SUSPEND:
        {
            g_bSuspended = true;
            break;
        }

        //
        // This event signals that the host has resumed signalling on the bus.
        //
        case USB_EVENT_RESUME:
        {
            g_bSuspended = false;
            break;
        }


        //
        // We ignore all other events.
        //
        default:
        {
            break;
        }
    }

    return(0);
}

//*****************************************************************************
//
// Send Data if necessary
//
//*****************************************************************************
void SendHIDReport(char ReportNum, signed char ReportData[])
{
	g_eCustomHidState = STATE_SENDING;
	USBDHIDCustomHidStateChange((void *)&g_sCustomHidDevice,ReportNum,ReportData);
	while(g_eCustomHidState != STATE_IDLE)
	{
	}
}

//*****************************************************************************
//
// Store switch states in buffers
//
//*****************************************************************************
void
StoreSwitches(void)
{
	//Read Player1 Controls
	g_ui8Pad1_1State[g_ui8IndexDebounce] =~GPIOPinRead(GPIO_JOY1_BASE, GPIO_JOY1_UP|GPIO_JOY1_DOWN|GPIO_JOY1_LEFT|GPIO_JOY1_RIGHT);
	g_ui8Pad1_2State[g_ui8IndexDebounce] =~GPIOPinRead(GPIO_P1_BASE, GPIO_P1_COIN|GPIO_P1_START|GPIO_P1_F|GPIO_P1_E|GPIO_P1_D|GPIO_P1_C|GPIO_P1_B|GPIO_P1_A);

	//Read Player2 Controls
	g_ui8Pad2_1State[g_ui8IndexDebounce] =~GPIOPinRead(GPIO_JOY2_BASE, GPIO_JOY2_UP|GPIO_JOY2_DOWN|GPIO_JOY2_LEFT|GPIO_JOY2_RIGHT)>>2;
	g_ui8Pad2_2State[g_ui8IndexDebounce] =~GPIOPinRead(GPIO_P2_BASE, GPIO_P2_COIN|GPIO_P2_START|GPIO_P2_F|GPIO_P2_E|GPIO_P2_D|GPIO_P2_C|GPIO_P2_B|GPIO_P2_A);

	//Read Mouse Buttons
	g_ui8Mouse_State[g_ui8IndexDebounce] =0;//~GPIOPinRead(GPIO_P1_BASE, GPIO_P1_B|GPIO_P1_A);

	++g_ui8IndexDebounce;
	if(g_ui8IndexDebounce >= MAX_CHECKS)
	{
		g_ui8IndexDebounce = 0;
	}

	//Read Mouse/Trackball X/Y
	g_ui8Mouse_Debounced[1] = (QEIPositionGet(QEI0_BASE)-127) * MOUSE_SCALAR;
	g_ui8Mouse_Debounced[2] = (QEIPositionGet(QEI1_BASE)-127) * MOUSE_SCALAR;
}

//*****************************************************************************
//
// Debounce switches
//
//*****************************************************************************
void
DebounceSwitches(void)
{
	unsigned char i,j;

	//Pad1 DPad
	j=0xff;
	for (i=0; i<MAX_CHECKS; i++)
	{
		j=j & g_ui8Pad1_1State[i];
	}
	g_ui8Pad1_Debounced[0]=j;

	//Pad1 Buttons
	j=0xff;
	for (i=0; i<MAX_CHECKS; i++)
	{
		j=j & g_ui8Pad1_2State[i];
	}
	g_ui8Pad1_Debounced[1]=j;

	//Pad2 DPad
	j=0xff;
	for (i=0; i<MAX_CHECKS; i++)
	{
		j=j & g_ui8Pad2_1State[i];
	}
	g_ui8Pad2_Debounced[0]=j;

	//Pad2 Buttons
	j=0xff;
	for (i=0; i<MAX_CHECKS; i++)
	{
		j=j & g_ui8Pad2_2State[i];
	}
	g_ui8Pad2_Debounced[1]=j;

	//Mouse (Buttons only)
	j=0xff;
	for (i=0; i<MAX_CHECKS; i++)
	{
		j=j & g_ui8Mouse_State[i];
	}
	g_ui8Mouse_Debounced[0]=j;
}

//*****************************************************************************
//
// Check buttons
//
//*****************************************************************************
void
CustomHidChangeHandler(void)
{
	// If the bus is suspended then resume it.
	//
	if(g_bSuspended)
	{
		USBDHIDCustomHidRemoteWakeupRequest((void *)&g_sCustomHidDevice);
	}

	//Get debounced switch states
	//
	DebounceSwitches();

	//Player 1 Controls
	if ((g_ui8Pad1_Debounced[0] != g_ui8Pad1LastSent[0]) | (g_ui8Pad1_Debounced[1] != g_ui8Pad1LastSent[1]))		//Send report if state has changed
	{
		SendHIDReport(1,g_ui8Pad1_Debounced);
		//Update the last sent value
		g_ui8Pad1LastSent[0] = g_ui8Pad1_Debounced[0];		//Update old states
		g_ui8Pad1LastSent[1] = g_ui8Pad1_Debounced[1];
	}

	//Player 2 Controls
	if ((g_ui8Pad2_Debounced[0] != g_ui8Pad2LastSent[0]) | (g_ui8Pad2_Debounced[1] != g_ui8Pad2LastSent[1]))
	{
		SendHIDReport(2,g_ui8Pad2_Debounced);
		//Update the last sent value
		g_ui8Pad2LastSent[0] = g_ui8Pad2_Debounced[0];
		g_ui8Pad2LastSent[1] = g_ui8Pad2_Debounced[1];
	}

	//Mouse
	if ((g_ui8Mouse_Debounced[0] != g_ui8MouseLastSent[0]) | (g_ui8Mouse_Debounced[1] != g_ui8MouseLastSent[1]) | (g_ui8Mouse_Debounced[2] != g_ui8MouseLastSent[2]))
	{
		SendHIDReport(3,g_ui8Mouse_Debounced);

		//Update the last sent values
		g_ui8MouseLastSent[0] = g_ui8Mouse_Debounced[0];
		g_ui8MouseLastSent[1] = g_ui8Mouse_Debounced[1];
		g_ui8MouseLastSent[2] = g_ui8Mouse_Debounced[2];

		//Reset Counter once sent
		QEIPositionSet(QEI0_BASE, 127);
		QEIPositionSet(QEI1_BASE, 127);
	}

}

//*****************************************************************************
//
// This is the interrupt handler for the SysTick interrupt.  It is used to
// update our local tick count and place the board in programming mode when appropriate
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
	g_ui32SysTickCount++;
	//Read Buttons and trackball values
	StoreSwitches();

}

//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    bool bLastSuspend;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run from the PLL at 50MHz.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	// Initialize the inputs
    PortFunctionInit();

    // Set the system tick to control how often the buttons are polled.
	ROM_SysTickEnable();
	ROM_SysTickIntEnable();
	ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);

	// Set initial LED Status to RED to indicate not connected
	ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);

    //
    // Not configured initially.
    //
    g_bConnected = false;
    g_bSuspended = false;
    bLastSuspend = false;
    g_bProgramMode = false;
    g_ui8Pad1_Debounced[0] = 0x00;
    g_ui8Pad1_Debounced[1] = 0x00;

    g_ui8Pad2_Debounced[0] = 0x00;
    g_ui8Pad2_Debounced[1] = 0x00;

    g_ui8Mouse_Debounced[0] = 0x00;
    g_ui8Mouse_Debounced[1] = 0x00;
    g_ui8Mouse_Debounced[2] = 0x00;

    g_ui8Pad1LastSent[0] = 0x00;
    g_ui8Pad1LastSent[1] = 0x00;

    g_ui8Pad2LastSent[0] = 0x00;
    g_ui8Pad2LastSent[1] = 0x00;

    g_ui8MouseLastSent[0] = 0x00;
    g_ui8MouseLastSent[1] = 0x00;
    g_ui8MouseLastSent[2] = 0x00;

    //
    // Initialize the USB stack for device mode. (must use force on the Tiva launchpad since it doesn't have detection pins connected)
    //
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    //
    // Pass our device information to the USB HID device class driver,
    // initialize the USB
    // controller and connect the device to the bus.
    //
    USBDHIDCustomHidInit(0, &g_sCustomHidDevice);

    //DISable peripheral and int before configuration
   	QEIDisable(QEI0_BASE);
   	QEIDisable(QEI1_BASE);
   	QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
   	QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

   	// Configure quadrature encoder, use a top limit of 256
   	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 255);
   	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 255);

   	// Enable the quadrature encoder.
   	QEIEnable(QEI0_BASE);
   	QEIEnable(QEI1_BASE);

   	//Set position to a middle value so we can see if things are working
   	QEIPositionSet(QEI0_BASE, 127);
   	QEIPositionSet(QEI1_BASE, 127);

    IntMasterEnable();

    //
    // The main loop starts here.  We begin by waiting for a host connection
    // then drop into the main volume handling section.  If the host
    // disconnects, we return to the top and wait for a new connection.
    //
    while(1)
    {
        //
        // Wait here until USB device is connected to a host.
        //
        while(!g_bConnected)
        {
        	//Set the onboard LED to red when not connected
        	ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
        }

        //
        // Update the status to green when connected.
        ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
        //
        // Enter the idle state.
        //
        g_eCustomHidState = STATE_IDLE;

        //
        // Assume that the bus is not currently suspended if we have just been
        // configured.
        //
        bLastSuspend = false;

        //
        // Keep checking the volume buttons for as
        // long as we are connected to the host. This is a simple example
        //
        while(g_bConnected)
        {
            //
			// Has the suspend state changed since last time we checked?
			//
			if(bLastSuspend != g_bSuspended)
			{
				//
				// Yes - the state changed so update the display.
				//
				bLastSuspend = g_bSuspended;
				if(bLastSuspend)
				{
					//Set the onboard LED to red when not connected
					ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
				}
				else
				{
				    // Update the status to green when connected.
					ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
				}
			}

		    if(g_ui32SysTickCount>=MAX_CHECKS  && !g_bProgramMode)
		    {
		    	//Reset systick counter
		    	g_ui32SysTickCount = 0;

				//Check inputs and act accordingly
		    	CustomHidChangeHandler();
		    }


        }
    }
}
