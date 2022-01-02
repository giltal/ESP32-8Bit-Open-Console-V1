
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "CH559.h"
#include "util.h"
#include "uart.h"

uint8_t __xdata uartRxBuff[64];
uint8_t __xdata rxPos = 0;
uint8_t __xdata uartRxData;

int processUart()
{
	int val = -1;
	while(RI)
	{
            RI=0;
			uartRxData = SBUF;
			if (uartRxData == 'P')
			{
				runBootloader();
			}
			if (uartRxData == '0')
			{
				val = 0;
			}
			if (uartRxData == '1')
			{
				val = 1;
			}
			if (uartRxData == '2')
			{
				val = 2;
			}
			if (uartRxData == '3')
			{
				val = 3;
			}
			if (uartRxData == 'R')
			{
				val = 10;
			}

            /*uartRxBuff[rxPos] = SBUF;
            if (uartRxBuff[rxPos]=='\n' || rxPos >= 64)
			{
                for (uint8_t i = 0; i < rxPos; i ++ )
                {
                    printf( "0x%02X ",uartRxBuff[i]);
                }
                printf("\n");
                if(uartRxBuff[0]=='k'){
                //if(uartRxBuff[1]==0x61)LED=0;
                //if(uartRxBuff[1]==0x73)LED=1;
                if(uartRxBuff[1]=='b')runBootloader();
                }
            rxPos=0;
            }else{
            rxPos++;
            }*/
    }
	return val;
}

void sendProtocolMSG(unsigned char msgtype, unsigned short length, unsigned char type, unsigned char device, unsigned char endpoint, unsigned char __xdata *msgbuffer)
{
    unsigned short i;
    putchar(0xFE);	
	putchar(length);
	putchar((unsigned char)(length>>8));
	putchar(msgtype);
	putchar(type);
	putchar(device);
	putchar(endpoint);
	putchar(0);
	putchar(0);
	putchar(0);
	putchar(0);
	for (i = 0; i < length; i++)
	{
		putchar(msgbuffer[i]);
	}
	putchar('\n');
}

unsigned char lastMsgP0[128] = { 0 };
unsigned char lastMsgP1[128] = { 0 };
unsigned char lastMsgP2[128] = { 0 };

extern int delayForSend;
#define DELAY() delay(delayForSend)

void sendHidPollMSG(unsigned char msgtype, unsigned short length, unsigned char type, unsigned char device, unsigned char endpoint, unsigned char __xdata *msgbuffer,unsigned char idVendorL,unsigned char idVendorH,unsigned char idProductL,unsigned char idProductH)
{
	if ((type == MSG_TYPE_ERROR) || (type > 0x8))
	{
		return;
	}
	unsigned short i,hitCounter0 = 0, hitCounter1 = 0, hitCounter2 = 0;
	if ((device == 0) && (length < 128))
	{
		for (i = 0; i < length; i++)
		{
			if (lastMsgP0[i] != msgbuffer[i])
			{
				lastMsgP0[i] = msgbuffer[i];
				hitCounter0++;
			}
		}
		if (hitCounter0 == 0) // same message - do not send again
		{
			return;
		}
	}
	if ((device == 1) && (length < 128))
	{
		for (i = 0; i < length; i++)
		{
			if (lastMsgP1[i] != msgbuffer[i])
			{
				lastMsgP1[i] = msgbuffer[i];
				hitCounter1++;
			}
		}
		if (hitCounter1 == 0) // same message - do not send again
		{
			return;
		}
	}
	if ((device == 2) && (length < 128))
	{
		for (i = 0; i < length; i++)
		{
			if (lastMsgP2[i] != msgbuffer[i])
			{
				lastMsgP2[i] = msgbuffer[i];
				hitCounter2++;
			}
		}
		if (hitCounter2 == 0) // same message - do not send again
		{
			return;
		}
	}

    putchar(0xFE);
	DELAY();
	putchar(length);
	DELAY();
	putchar((unsigned char)(length>>8));
	DELAY();
	putchar(msgtype);
	DELAY();
	putchar(type);
	DELAY();
	putchar(device);
	DELAY();
	putchar(endpoint);
	DELAY();
	putchar(idVendorL);
	DELAY();
	putchar(idVendorH);
	DELAY();
	putchar(idProductL);
	DELAY();
	putchar(idProductH);
	DELAY();
	for (i = 0; i < length; i++)
	{
		putchar(msgbuffer[i]);
		DELAY();
	}
	putchar('\n');
}