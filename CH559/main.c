typedef unsigned char *PUINT8;
typedef unsigned char __xdata *PUINT8X;
typedef const unsigned char __code *PUINT8C;
typedef unsigned char __xdata UINT8X;
typedef unsigned char  __data             UINT8D;

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "CH559.h"
#include "util.h"
#include "USBHost.h"
#include "uart.h"

SBIT(LED, 0x90, 6);

int delayForSend = 1;

void main()
{
    unsigned char s;
    initClock();
    initUART0(1000000, 1);
    DEBUG_OUT("Startup\n");
    resetRootHubPort((unsigned char)0);
	resetRootHubPort((unsigned char)1);
	resetHubDevices(0);
	resetHubDevices(1);
	delay(500);
	initUSB_Host();
    DEBUG_OUT("Ready\n");
	sendProtocolMSG(MSG_TYPE_STARTUP,0, 0x00, 0x00, 0x00, 0);
	int val = 1;
	unsigned char temp;
    while(1)
    {
        if(!(P4_IN & (1 << 6)))
            runBootloader();
		if ((val = processUart()) != -1)
		{
			if (val == 10)
			{
				temp = UHUB0_CTRL;
				UHUB0_CTRL = ((temp & 0xFB) | 0x2);
				delay(1000);
				UHUB0_CTRL = (temp & 0xFB);
				
				temp = UHUB1_CTRL;
				UHUB1_CTRL = ((temp & 0xFB) | 0x2);
				delay(1000);
				UHUB1_CTRL = (temp & 0xFB);

				// Reset the USB connected devices
				/*
				temp = USB_CTRL;
				USB_CTRL = ((temp & 0xCF) | 0x10);
				delay(1000);
				USB_CTRL = (temp & 0xCF);
				*/
				// Enter safemode
				/*
				SAFE_MOD = 0x55;
				SAFE_MOD = 0xAA;
				// SW Reset
				GLOBAL_CFG = bSW_RESET;
				*/
			}
			else
				delayForSend = val;
		}
			
        s = checkRootHubConnections();
        pollHIDdevice();
    }
}