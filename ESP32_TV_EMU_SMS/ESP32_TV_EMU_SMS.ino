/* Copyright (c) 2020, Peter Barrett
**
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#define USE_CH559

#ifdef USE_CH559
#include "driver/uart.h"
#include "CH559.h"
#endif

#include "SPI.h"
#include "esp_system.h"
#include "esp_int_wdt.h"
#include "esp_spiffs.h"
#include "src\config.h"
#include "SNEScont.h"
#include <EEPROM.h>
#include "Wiimote.h"

WII_KEYS wiiMoteKeys;

#define PERF  // some stats about where we spend our time
#include "src/emu.h"
#include "src/video_out.h"
#define noINTEL_MP

// esp_8_bit
// Atari 8 computers, NES and SMS game consoles on your TV with nothing more than a ESP32 and a sense of nostalgia
// Supports NTSC/PAL composite video, Bluetooth Classic keyboards and joysticks

//  Choose one of the video standards: PAL,NTSC
#define VIDEO_STANDARD NTSC

//  Choose one of the following emulators: EMU_NES,EMU_SMS,EMU_ATARI
//#define EMULATOR EMU_ATARI
//#define EMULATOR EMU_NES
//#define EMULATOR EMU_SMS
//  Many emus work fine on a single core (S2), file system access can cause a little flickering
//  #define SINGLE_CORE

// The filesystem should contain folders named for each of the emulators i.e.
//    atari800
//    nofrendo
//    smsplus
// Folders will be auto-populated on first launch with a built in selection of sample media.
// Use 'ESP32 Sketch Data Upload' from the 'Tools' menu to copy a prepared data folder to ESP32


using namespace std;

Emu* _emu = 0;            // emulator running on core 0
uint32_t _frame_time = 0;
uint32_t _drawn = 1;
bool _inited = false;

#ifndef USE_CH559
SNEScont SNEScont1((unsigned char)19, (unsigned char)23);
#else
// UART interface for the CH559
class ch559uartAcc : public UARTaccessor
{
public:
	ch559uartAcc() {};
	~ch559uartAcc() {};
	bool init()
	{
		Serial2.begin(1000000, SERIAL_8N1, 19, 23);
		Serial2.setRxBufferSize(256);
		return true;
	}
	int available()
	{
		return Serial2.available();
	}
	int read()
	{
		return Serial2.read();
	}
	void write(unsigned char data)
	{
		Serial2.write(&data, 1);
	}
};

HIDdevice hidDevice;
joyStick joystick;

joystickMapper SNESmapper;
joystickMapper SparkFunMapper;

ch559uartAcc ch559accessor;
CH559 ch559port(&ch559accessor);
#endif

SNESbuttons SNESbut;

void emu_init()
{
	std::string folder = "/" + _emu->name;
	gui_start(_emu, folder.c_str());
	_drawn = _frame_counter;
}

unsigned long resetCounter = 0, totalResetCounts;

bool getIntoMenu = true;
unsigned char emulatorType;
int secEmuType;

void emu_loop()
{
	// wait for blanking before drawing to avoid tearing
	video_sync();

	// Draw a frame, update sound, process hid events
	uint32_t t = xthal_get_ccount();
	gui_update();
	_frame_time = xthal_get_ccount() - t;
	_lines = _emu->video_buffer();
	_drawn++;

#ifndef USE_CH559
	SNEScont1.readCont(0, &SNESbut);
#endif

	if (getIntoMenu)
	{
		SNESbut.L = 1;
	}
	else
	{
		// Wait for menu to load
		Wiimote::handle();
		Wiimote::readKeys(&wiiMoteKeys);
	}

	if (SNESbut.R)
	{
		resetCounter = ESP.getCycleCount() - resetCounter;
		totalResetCounts += resetCounter;
		if (totalResetCounts > 240000000 * 3)
		{
			ESP.restart();
		}
		else
			resetCounter = ESP.getCycleCount();
	}
	else
	{
		resetCounter = ESP.getCycleCount();
		totalResetCounts = 0;
	}
}

// dual core mode runs emulator on comms core
void emu_task(void* arg)
{
	printf("emu_task %s running on core %d at %dmhz\n",
		_emu->name.c_str(), xPortGetCoreID(), rtc_clk_cpu_freq_value(rtc_clk_cpu_freq_get()));
	emu_init();
	for (;;)
		emu_loop();
}

#ifdef USE_CH559
int delayVal = 0;

void controllerTask(void* arg)
{
	unsigned char fire = 0, left = 0, right = 0, up = 0, down = 0, select = 0, start = 0, home = 0;
	while (1)
	{
		if (ch559accessor.available() > 20)
		{
			if (ch559port.update(&hidDevice))
			{
				if (hidDevice.ID == SNESmapper.deviceID)
				{
					ch559port.decodeJoystick(&hidDevice, &SNESmapper, &joystick);
					fire = SNESbut.A = joystick.a;
					SNESbut.B = joystick.b;
					home = SNESbut.L = joystick.leftS1;
					SNESbut.R = joystick.rightS1;
					SNESbut.X = joystick.x;
					SNESbut.Y = joystick.y;
					start = SNESbut.Start = joystick.start;
					select = SNESbut.Select = joystick.select;
					up = SNESbut.Up = (joystick.analogY2 == 0x00);
					down = SNESbut.Down = (joystick.analogY2 == 0xff);
					left = SNESbut.Left = (joystick.analogX2 == 0x00);
					right = SNESbut.Right = (joystick.analogX2 == 0xff);
				}
				if (SNESbut.R)
				{
					ch559accessor.write('1');
				}
				if (hidDevice.ID == SparkFunMapper.deviceID)
				{
					ch559port.decodeJoystick(&hidDevice, &SparkFunMapper, &joystick);
					fire = SNESbut.A = joystick.a;
					SNESbut.B = 0;
					home = SNESbut.L = joystick.leftS1; // Home
					SNESbut.R = joystick.rightS1;
					//SNESbut.X = joystick.x;
					//SNESbut.Y = joystick.y;
					start = SNESbut.Start = joystick.start;
					select = SNESbut.Select = joystick.select;
					up = SNESbut.Up = (joystick.analogY1 == 0x01);
					down = SNESbut.Down = (joystick.analogY1 == 0xff);
					left = SNESbut.Left = ((joystick.analogX1 == 0x01) || SNESbut.R);
					right = SNESbut.Right = (joystick.analogX1 == 0xff);
				}
			}
		}
		else
		{
			if (!fire)
			{
				SNESbut.A = !digitalRead(32);
			}
			if (!up)
			{
				SNESbut.Up = !digitalRead(22);
			}
			if (!down)
			{
				SNESbut.Down = !digitalRead(21);
			}
			if (!right)
			{
				SNESbut.Right = !digitalRead(27);
			}
			if (!left)
			{
				SNESbut.Left = !digitalRead(14);
			}
			if (!home)
			{
				SNESbut.L = !digitalRead(26);
			}
			if (!start)
			{
				SNESbut.Start = !digitalRead(34);
			}
			if (!select)
			{
				SNESbut.Select = !digitalRead(35);
			}
		}
		//AtariPot = readAveragedAnalog();
		delay(20);
	}
}
#endif

#ifdef USE_SD
#include "SD.h"
SPIClass SDspi(HSPI);
bool SDmountOK = false;
#endif

esp_err_t mount_filesystem()
{
	printf("\n\n\nesp_8_bit\n\nmounting spiffs (will take ~15 seconds if formatting for the first time)....\n");
	uint32_t t = millis();
	esp_vfs_spiffs_conf_t conf = {
	  .base_path = "",
	  .partition_label = NULL,
	  .max_files = 5,
	  .format_if_mount_failed = true  // force?
	};
	esp_err_t e = esp_vfs_spiffs_register(&conf);
	if (e != 0)
		printf("Failed to mount or format filesystem: %d. Use 'ESP32 Sketch Data Upload' from 'Tools' menu\n", e);
	vTaskDelay(1);
	printf("... mounted in %d ms\n", millis() - t);
	return e;
}

void setup()
{
	// Atari Joystick
#ifdef USE_CH559
	pinMode(14, INPUT);
	pinMode(21, INPUT);
	pinMode(22, INPUT);
	pinMode(26, INPUT);
	pinMode(27, INPUT);
	pinMode(32, INPUT);
	pinMode(34, INPUT);
	pinMode(35, INPUT);
#endif
#ifdef USE_PSARM
	if (psramInit())
	{
		printf("PSRAM init OK\n");
		Wiimote::init(true);
	}
	else
		Wiimote::init(false);
	printf("PSRAM size: %d\n", ESP.getFreePsram());
#endif
	rtc_clk_cpu_freq_set(RTC_CPU_FREQ_240M);
#ifdef USE_SD
	SDspi.begin(15, 18, 13, 4);
	if (!SD.begin(4, SDspi, 10000000, "/sd", 5))
	{
		printf("Failed to mount SD card!\n");
		SDmountOK = false;
	}
	else
	{
		printf("SD card mounted on VSPI.\n");
		SDmountOK = true;
	}

	if (SDmountOK)
	{
		uint8_t cardType = SD.cardType();
		if (cardType == CARD_NONE)
		{
			Serial.println("No SD card attached\n");
			SDmountOK = false;
		}
	}
#endif
#ifndef USE_SD
	mount_filesystem();                       // mount the filesystem!
#endif
#ifndef USE_CH559
	SNEScont1.addCont(36, 0);
#endif
#define MULTI_EMU
#define EEPROM_SIZE 128
#ifdef MULTI_EMU
	EEPROM.begin(EEPROM_SIZE);
	emulatorType = EEPROM[0];
	if (emulatorType > 2)
	{
		EEPROM[0] = 2;
		EEPROM.commit();
		emulatorType = 2;
	}
	switch (emulatorType)
	{
	case 0:
		_emu = NewSMSPlus(VIDEO_STANDARD);
		EEPROM[0] = 2;
		EEPROM.commit();
		emulatorType = 2;
		break;
	case 1:
		EEPROM[0] = 2;
		EEPROM.commit();
		emulatorType = 2;
		_emu = NewSMSPlus(VIDEO_STANDARD);
		break;
	case 2:
		emulatorType = 2;
		_emu = NewSMSPlus(VIDEO_STANDARD);
		break;
	}
#else
	_emu = NewEmulator();                     // create the emulator!
#endif
	//hid_init("emu32");                        // bluetooth hid on core 1!
	xTaskCreatePinnedToCore(emu_task, "emu_task", 8 * 1024, NULL, 0, NULL, 0); // nofrendo needs 5k word stack, start on core 0
#ifdef USE_CH559
	printf("Init CH559...\n");
	ch559port.init();
	delay(100);
	printf("CH559 initialized...\n");

	SparkFunMapper.deviceID = 0x4f1b0692;
	SparkFunMapper.analogX1index = 2;
	SparkFunMapper.analogY1index = 4;
	SparkFunMapper.analogX2index = 6;
	SparkFunMapper.analogY2index = 0;
	SparkFunMapper.selectIndex = 1;
	SparkFunMapper.selectMask = 0x08;
	SparkFunMapper.startIndex = 1;
	SparkFunMapper.startMask = 0x02;
	SparkFunMapper.aIndex = 1;
	SparkFunMapper.aMask = 0x01;
	SparkFunMapper.bIndex = 0;
	SparkFunMapper.bMask = 0xF0;
	SparkFunMapper.xIndex = 0;
	SparkFunMapper.xMask = 0xF0;
	SparkFunMapper.yIndex = 0;
	SparkFunMapper.yMask = 0xF0;
	SparkFunMapper.leftS1Index = 1; // Home
	SparkFunMapper.leftS1Mask = 0x04;
	SparkFunMapper.leftS2Index = 0;
	SparkFunMapper.leftS2Mask = 0xF0;
	SparkFunMapper.rightS1Index = 1;
	SparkFunMapper.rightS1Mask = 0x10;
	SparkFunMapper.rightS2Index = 0;
	SparkFunMapper.rightS2Mask = 0xF0;
	SparkFunMapper.dpadIndex = 0;
	SparkFunMapper.dpadMask = 0xF0;
	SparkFunMapper.dpadUpVal = 0xF0;
	SparkFunMapper.dpadDownVal = 0xF0;
	SparkFunMapper.dpadLeftVal = 0xF0;
	SparkFunMapper.dpadRightVal = 0xF0;

	SNESmapper.deviceID = 0x79001100;
	SNESmapper.analogX1index = 1;
	SNESmapper.analogY1index = 2;
	SNESmapper.analogX2index = 3;
	SNESmapper.analogY2index = 4;
	SNESmapper.selectIndex = 6;
	SNESmapper.selectMask = 0x10;
	SNESmapper.startIndex = 6;
	SNESmapper.startMask = 0x20;
	SNESmapper.aIndex = 5;
	SNESmapper.aMask = 0x20;
	SNESmapper.bIndex = 5;
	SNESmapper.bMask = 0x40;
	SNESmapper.xIndex = 5;
	SNESmapper.xMask = 0x10;
	SNESmapper.yIndex = 5;
	SNESmapper.yMask = 0x80;
	SNESmapper.leftS1Index = 6;
	SNESmapper.leftS1Mask = 0x01;
	SNESmapper.leftS2Index = 0;
	SNESmapper.leftS2Mask = 0xFF;
	SNESmapper.rightS1Index = 6;
	SNESmapper.rightS1Mask = 0x02;
	SNESmapper.rightS2Index = 0;
	SNESmapper.rightS2Mask = 0xFF;
	SNESmapper.dpadIndex = 0;
	SNESmapper.dpadMask = 0xFF;
	SNESmapper.dpadUpVal = 0xFF;
	SNESmapper.dpadDownVal = 0xFF;
	SNESmapper.dpadLeftVal = 0xFF;
	SNESmapper.dpadRightVal = 0xFF;

	xTaskCreatePinnedToCore(controllerTask, "cont_task", 1024, NULL, 5, NULL, 1);
#endif
}

// this loop always runs on app_core (1).
void loop()
{
	// start the video after emu has started
	if (!_inited) 
	{
		if (_lines) {
			printf("video_init\n");
			video_init(_emu->cc_width, _emu->flavor, _emu->composite_palette(), _emu->standard); // start the A/V pump
			_inited = true;
		}
		else {
			vTaskDelay(1);
		}
	}
	// update the bluetooth edr/hid stack
	//hid_update();
}