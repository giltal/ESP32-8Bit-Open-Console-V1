/*
 Name:		ESP32_GB_TVout.ino
 Created:	18-Aug-21 11:06:12 PM
 Author:	giltal
*/

#include "graphics.h"
#include "FS.h"
#include "SD.h"
#include "Wiimote.h"
#include "PCF8574.h"
#include "esp_partition.h"
#include "esp_task_wdt.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "soc/ledc_struct.h"

#define USE_CH559

#ifdef USE_CH559
#include "driver/uart.h"
#include "CH559.h"
#include "SNEScont.h"

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
//unsigned char AtariPot;
//SNESbuttons SNESbut;

joystickMapper SNESmapper;
joystickMapper SparkFunMapper;
joystickMapper RetroJoyMapper;

ch559uartAcc ch559accessor;
CH559 ch559port(&ch559accessor);
#endif
#ifdef USE_CH559
int delayVal = 0;
long mapJoy(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif

void feedTheWDog() 
{
	// feed dog 0
	TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // write enable
	TIMERG0.wdt_feed = 1;                       // feed dog
	TIMERG0.wdt_wprotect = 0;                   // write protect
	// feed dog 1
	TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // write enable
	TIMERG1.wdt_feed = 1;                       // feed dog
	TIMERG1.wdt_wprotect = 0;                   // write protect
}
WII_KEYS wiiMoteKeys;
bool wiiMoteConnected = false;
/////////
TVout tvOut;
uint8_t cartSave[8 * 1024];
const unsigned char * ROMtoLoad;

///////// Game Boy Related

#include "sound.h"
#include "peanut_gb.cpp"

#define WRITE_DELAY 5000
#define SOUNDPIN 5

volatile uint8_t soundFlag = 0;
static uint8_t previousSoundFlag = 1;

static struct gb_s gb;
enum gb_init_error_e ret;

uint8_t gb_rom_read(struct gb_s *gb, const uint32_t addr) 
{
	return ROMtoLoad[addr];
}

uint8_t ICACHE_RAM_ATTR gb_cart_ram_read(struct gb_s *gb, const uint32_t addr) 
{
	if (addr > 8*1024)
	{
		printf("Cart mem read address error!\n");
		return 0;
	}
	return cartSave[addr];
}

void ICACHE_RAM_ATTR gb_cart_ram_write(struct gb_s *gb, const uint32_t addr, const uint8_t val)
{
	if (addr > 8 * 1024)
	{
		printf("Cart mem read address error!\n");
		return;
	}
	cartSave[addr] = val;
}

void gb_error(struct gb_s *gb, const uint16_t gb_err, const uint16_t val)
{
	const char* gb_err_str[4] = {
		"UNKNOWN",
		"INVALID OPCODE",
		"INVALID READ",
		"INVALID WRITE"
	};

	Serial.print(F("Error "));
	Serial.print(gb_err);
	Serial.print(F(" occurred:  "));
	Serial.println(gb_err >= GB_INVALID_MAX ? gb_err_str[0] : gb_err_str[gb_err]);
	Serial.print(F("At Address "));
	Serial.println(val, HEX);
}

/* 
	Palette codes
	const static uint32_t palette0[4] = { 0x7FFF, 0x329F, 0x001F, 0x0000 }; // OBJ0
	const static uint32_t palette1[4] = { 0x7FFF, 0x3FE6, 0x0200, 0x0000 }; // OBJ1
	const static uint32_t palette2[4] = { 0x7FFF, 0x7EAC, 0x40C0, 0x0000 }; // BG
*/

void lcd_draw_line(struct gb_s *gb, const uint8_t *pixels, const uint_fast8_t line)
{
	unsigned short actLine = line;
	int x;
	if (line < 124)
	{
		unsigned short * tempLinePointer = (unsigned short *)tvOut.getLineBufferAddress(actLine * 2);
		unsigned short * tempLinePointer2 = (unsigned short *)tvOut.getLineBufferAddress(actLine * 2 + 1);

		unsigned short val;

		for (x = 0; x < 160; x++)
		{
			val = ((pixels[x] & 3) << 8) | (pixels[x] & 3);
			tempLinePointer[x] = val;
			tempLinePointer2[x] = val;
		}

	}
	else
	{
		unsigned short * tempLinePointer = (unsigned short *)tvOut.getLineBufferAddress(124 * 2 + (line - 124));

		unsigned short val;

		for (x = 0; x < 160; x++)
		{
			val = ((pixels[x] & 3) << 8) | (pixels[x] & 3);
			tempLinePointer[x] = val;
		}
	}
}

volatile uint8_t sound_dac;

void IRAM_ATTR sound_ISR()
{
	if (soundFlag) 
	{
		//sigmaDeltaWrite(0, sound_dac);
		LEDC.channel_group[0].channel[0].duty.duty = sound_dac << 4;// (sound_dac >> 8) << 4; // 25 bit (21.4)
		LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1; // This is the output enable control bit for channel
		LEDC.channel_group[0].channel[0].conf1.duty_start = 1; // When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware
		LEDC.channel_group[0].channel[0].conf0.clk_en = 1;

		sound_dac = audio_update();
	}
}

unsigned int contLeft = 0, contDown = 0, contUp = 0, contRight = 0, contStart = 0, contSelect = 0, contFire = 0, contHome = 0, contFireB = 0;

void controllerTask(void* arg)
{
	while (1)
	{
		if (ch559accessor.available() > 20)
		{
			if (ch559port.update(&hidDevice))
			{
				if (hidDevice.ID == SNESmapper.deviceID)
				{
					ch559port.decodeJoystick(&hidDevice, &SNESmapper, &joystick);
					contFire = joystick.a;
					contHome = joystick.leftS1;
					contStart = joystick.start;
					contSelect = joystick.select;
					contUp = (joystick.analogY2 == 0x00);
					contDown = (joystick.analogY2 == 0xff);
					contLeft = (joystick.analogX2 == 0x00);
					contRight = (joystick.analogX2 == 0xff);
				}
				if (hidDevice.ID == RetroJoyMapper.deviceID)
				{
					ch559port.decodeJoystick(&hidDevice, &RetroJoyMapper, &joystick);
					contFire = joystick.a;
					contHome = joystick.leftS1;
					contStart = joystick.start;
					contSelect = joystick.select;
					contUp = (joystick.analogY1 == 0x00);
					contDown = (joystick.analogY1 == 0xff);
					contLeft = (joystick.analogX1 == 0x00);
					contRight = (joystick.analogX1 == 0xff);
				}
				if (hidDevice.ID == SparkFunMapper.deviceID)
				{
					ch559port.decodeJoystick(&hidDevice, &SparkFunMapper, &joystick);
					contFire = joystick.a;
					contHome = joystick.leftS1; // Home
					contStart = joystick.start;
					contSelect = joystick.select;
					contUp = (joystick.analogY1 == 0x01);
					contDown = (joystick.analogY1 == 0xff);
					contLeft = ((joystick.analogX1 == 0x01));
					contRight = (joystick.analogX1 == 0xff);
					/*if (joystick.analogX2 > 5 && joystick.analogX2 < 229)
					{
						AtariPot = joystick.analogX2;
					}
					else
					{
						if (joystick.analogX2 > 228)
						{
							AtariPot = 228;
						}
						if (joystick.analogX2 < 6)
						{
							AtariPot = 6;
						}
					}*/
				}
			}
		}
		Wiimote::handle();
		Wiimote::readKeys(&wiiMoteKeys);
		if (Wiimote::isConnected())
			wiiMoteConnected = true;
		else
			wiiMoteConnected = false;

		contFireB = joystick.b | (wiiMoteKeys.b1 & wiiMoteConnected);
		if (!contLeft)
		{
			contLeft = (digitalRead(14) == LOW) | (wiiMoteKeys.aUp & wiiMoteConnected);
		}
		if (!contDown)
		{
			contDown = (digitalRead(21) == LOW) | (wiiMoteKeys.aLeft & wiiMoteConnected);
		}
		if (!contUp)
		{
			contUp = (digitalRead(22) == LOW) | (wiiMoteKeys.aRight & wiiMoteConnected);
		}
		if (!contRight)
		{
			contRight = (digitalRead(27) == LOW) | (wiiMoteKeys.aDown & wiiMoteConnected);
		}
		if (!contHome)
		{
			contHome = (digitalRead(26) == LOW) | (wiiMoteKeys.bHome & wiiMoteConnected);
		}
		if (!contStart)
		{
			contStart = (digitalRead(34) == LOW) | (wiiMoteKeys.bMinus & wiiMoteConnected);
		}
		if (!contFire)
		{
			contFire = (digitalRead(32) == LOW) | (wiiMoteKeys.b1 & wiiMoteConnected);
		}
		if (!contSelect)
		{
			contSelect = (digitalRead(35) == LOW) | (wiiMoteKeys.bPlus & wiiMoteConnected);
		}

		gb.direct.joypad_bits.a = !contFire;
		gb.direct.joypad_bits.b = !contFireB;
		gb.direct.joypad_bits.up = !contUp;
		gb.direct.joypad_bits.down = !contDown;
		gb.direct.joypad_bits.left = !contLeft;
		gb.direct.joypad_bits.right = !contRight;
		gb.direct.joypad_bits.start = !contStart;
		gb.direct.joypad_bits.select = !contSelect;

		delay(50);
	}
}

hw_timer_t * timer = NULL;
void loadROM();

void gameBoyTask(void* arg)
{
	// init game boy evulator
	while (1)
	{
		// Gray scale
		tvOut.updatePalette(0, 255, 255, 255);
		tvOut.updatePalette(1, 169, 169, 169);
		tvOut.updatePalette(2, 84, 84, 84);
		tvOut.updatePalette(3, 0x00, 0x00, 0x00);

		ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read, &gb_cart_ram_write, &gb_error, NULL);

		if (ret != GB_INIT_NO_ERROR) {
			Serial.print("Error: ");
			Serial.println(ret);
		}

		gb_init_lcd(&gb, &lcd_draw_line);
		gb.direct.interlace = 0;
		gb.direct.frame_skip = 1;
		//sigmaDeltaSetup(0, F_CPU / 256);
		//sigmaDeltaAttachPin(SOUNDPIN, 0);
		//sigmaDeltaEnable();

		sound_dac = 0;

		/*timer = timerBegin(0, 20, true); // Set to run @ 4MHz
		timerAttachInterrupt(timer, &sound_ISR, true);
		timerAlarmWrite(timer, 4000000 / SAMPLING_RATE, true); //AUDIO_SAMPLE_RATE
		timerAlarmEnable(timer);*/

		soundFlag = 1;
		tvOut.fillScr(0, 0, 0);
		while (1)
		{
			gb_run_frame(&gb);
			feedTheWDog();
			if (wiiMoteKeys.bHome | (digitalRead(26) == LOW))
			{
				/*timerAlarmDisable(timer);
				tvOut.generateRGB323palette();
				loadROM();*/
				ESP.restart();
				break;
			}
		}
	}
}


SPIClass SDspi(HSPI); // VSPI is used by LCD
bool SDmountOK;
PCF8574 pcf8574(0x21);

// the setup function runs once when you press reset or power the board
void setup() 
{
	pinMode(14, INPUT); // Left
	pinMode(21, INPUT); // Down
	pinMode(22, INPUT); // Up
	pinMode(26, INPUT); // Home
	pinMode(27, INPUT); // Right
	pinMode(32, INPUT); // Fire
	pinMode(34, INPUT); // Start
	pinMode(35, INPUT); // Sel
	
	if (!tvOut.init())
	{
		printf("Error initializing video engine!\n");
		while (1);
	}
	else
	{
		printf("TV Out engine is running...\n");
	}

	pinMode(SOUNDPIN, OUTPUT);

	ledcSetup(0, 2000000, 7);    // 625000 khz is as fast as we go w 7 bits
	ledcAttachPin(SOUNDPIN, 0);
	ledcWrite(0, 0);
	
	if (psramInit())
	{
		printf("PSRAM init OK\n");
		Wiimote::init(true);
	}
	else
		Wiimote::init(false);
	printf("PSRAM size: %d\n", ESP.getFreePsram());

	delay(250);

	xTaskCreatePinnedToCore(controllerTask, "cont_task", 1024, NULL, 5, NULL, 1);

	SDspi.begin(15, 18, 13, 4);

	delay(500);

	if (!SD.begin(4, SDspi, 1000000, "/sd", 5))
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
#ifdef USE_CH559
	printf("Init CH559...\n");
	ch559port.init();
	delay(100);
	printf("CH559 initialized...\n");

	// RetroJoyMapper
	RetroJoyMapper.deviceID = 0x79000600;
	RetroJoyMapper.analogX1index = 0;
	RetroJoyMapper.analogY1index = 1;
	RetroJoyMapper.analogX2index = 2;
	RetroJoyMapper.analogY2index = 3;
	RetroJoyMapper.selectIndex = 6;
	RetroJoyMapper.selectMask = 0x10;
	RetroJoyMapper.startIndex = 6;
	RetroJoyMapper.startMask = 0x20;
	RetroJoyMapper.aIndex = 6;
	RetroJoyMapper.aMask = 0x08;
	RetroJoyMapper.bIndex = 5;
	RetroJoyMapper.bMask = 0x80;
	RetroJoyMapper.xIndex = 6;
	RetroJoyMapper.xMask = 0x04;
	RetroJoyMapper.yIndex = 5;
	RetroJoyMapper.yMask = 0x40;
	RetroJoyMapper.leftS1Index = 6; // Home
	RetroJoyMapper.leftS1Mask = 0x01;
	RetroJoyMapper.leftS2Index = 4;
	RetroJoyMapper.leftS2Mask = 0x80;
	RetroJoyMapper.rightS1Index = 5;
	RetroJoyMapper.rightS1Mask = 0x10;
	RetroJoyMapper.rightS2Index = 4;
	RetroJoyMapper.rightS2Mask = 0x80;
	RetroJoyMapper.dpadIndex = 4;
	RetroJoyMapper.dpadMask = 0x80;
	RetroJoyMapper.dpadUpVal = 0x80;
	RetroJoyMapper.dpadDownVal = 0x80;
	RetroJoyMapper.dpadLeftVal = 0x80;
	RetroJoyMapper.dpadRightVal = 0x80;

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
#endif

	//interrupts();
}

#include "Solar_Striker__World.h"
bool listDirectoryAndGetSelectedFile(char * StartFolder, String * selectedFile);
void loadROM();

// the loop function runs over and over again until power down or reset
void loop() 
{
	tvOut.generateRGB323palette();
	tvOut.fillScr(0, 0, 0);
	tvOut.loadFonts(ORBITRON_LIGHT24);
	tvOut.setColor(255, 255, 255);
	tvOut.print("GAME Boy", 0, 110, true);
	loadROM();
	// ROMtoLoad is updated with the game
	printf("Free heap: %d\n", ESP.getFreeHeap());
	// update the GB palette

	xTaskCreatePinnedToCore(gameBoyTask, "gameBoy_task", 1024*4, NULL, 5, NULL, 0);
	
	timer = timerBegin(0, 20, true); // Set to run @ 4MHz
	timerAttachInterrupt(timer, &sound_ISR, true);
	timerAlarmWrite(timer, 4000000 / SAMPLING_RATE, true); //AUDIO_SAMPLE_RATE
	timerAlarmEnable(timer);

	while (1)
	{
		delay(1000);
	}
}

unsigned int ROMsize;
#define ESP32_FLASH_SECTOR_SIZE 4096
unsigned char tempFileBuffer[ESP32_FLASH_SECTOR_SIZE];

void loadROM()
{
	//ROMtoLoad = Asteroids; // Default ROM
	ROMsize = 0;
	unsigned char * tempFileBuffer;

	printf("Load ROM...\n");

	String pickedFile;
	unsigned int fileSize, loadedFileSize;
	tvOut.loadFonts(ORBITRON_LIGHT24);
	tvOut.fillScr(0, 0, 0);
	tvOut.setColor(255, 100, 100);
	tvOut.print("GameBoy emulator", 0, 30, true);
	tvOut.loadFonts(MONO_BOLD18);
	tvOut.setColor(100, 255, 100);
	tvOut.print("Please connect a WIImote", 0, 120, true);
	//while (!Wiimote::isConnected()) delay(50);
	tvOut.print("Connected", 0, 140, true);
	delay(1000);
	tvOut.setColor(100, 100, 255);
	tvOut.print("Loadind SD card content", 0, 200, true);
	if (SDmountOK)
	{
		tvOut.loadFonts(OBLIQUE18);
		bool exitMenu = false;
		do
		{
			if (listDirectoryAndGetSelectedFile("/GB", &pickedFile))
			{
				printf("Selected file: %s\n", pickedFile.c_str());
				File fileToLoad;
				fileToLoad = SD.open(pickedFile, "r");
				if (!fileToLoad)
				{
					printf("Cannot open file!\n");
				}
				else
				{
					printf("File Size: %d\n", fileSize = fileToLoad.size());
					ROMsize = fileSize;
					unsigned char * gameRomPointer;
					gameRomPointer = (unsigned char *)ps_malloc(fileSize); 
					if (gameRomPointer)
					{
						printf("Loading file...\n");
						loadedFileSize = fileToLoad.read(gameRomPointer, fileSize);
						if (loadedFileSize == fileSize)
						{
							ROMtoLoad = gameRomPointer;
							ROMsize = fileSize;
							exitMenu = true;
							printf("Loaded...\n");
						}
					}
					else
					{
						printf("Malloc failed - use SPIFFS partition\n");
						const esp_partition_t* part;
						spi_flash_mmap_handle_t handle;
						void* data;
						esp_err_t err;
						part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
						err = esp_partition_mmap(part, 0, part->size, SPI_FLASH_MMAP_DATA, (const void**)&data, &handle);
						if (err == ESP_OK)
						{
							printf("MAP flash OK\n");
							tempFileBuffer = (unsigned char *)malloc(4096);
							unsigned int numOfBlocks = fileSize / ESP32_FLASH_SECTOR_SIZE + 1;
							unsigned int readBytesSize; // ESP32_FLASH_SECTOR_SIZE;
							err = esp_partition_erase_range(part, 0, numOfBlocks*ESP32_FLASH_SECTOR_SIZE);
							if (err == ESP_OK)
							{
								for (size_t n = 0; n < numOfBlocks; n++)
								{
									readBytesSize = fileToLoad.read(tempFileBuffer, ESP32_FLASH_SECTOR_SIZE);
									err = esp_partition_write(part, 0 + n * ESP32_FLASH_SECTOR_SIZE, (void *)tempFileBuffer, readBytesSize);
									if (err != ESP_OK)
									{
										tvOut.loadFonts(ORBITRON_LIGHT24);
										tvOut.fillScr(0, 0, 0);
										tvOut.setColor(255, 100, 100);
										tvOut.print("Flash write failed!", 0, 120, true);
										delay(2000);
										continue;
									}
									printf(".");
								}
								printf("\n");
								ROMtoLoad = (unsigned char *)(data);
								exitMenu = true;
							}
							else
							{
								tvOut.loadFonts(ORBITRON_LIGHT24);
								tvOut.fillScr(0, 0, 0);
								tvOut.setColor(255, 100, 100);
								tvOut.print("Flash erase failed!", 0, 120, true);
								delay(2000);
							}
							free(tempFileBuffer);
						}
						else
						{
							tvOut.loadFonts(ORBITRON_LIGHT24);
							tvOut.fillScr(0, 0, 0);
							tvOut.setColor(255, 100, 100);
							tvOut.print("Cannot allocate flash memory", 0, 120, true);
							delay(2000);
						}
						if (!exitMenu)
						{
							tvOut.fillScr(0, 0, 0);
							tvOut.setColor(100, 255, 100);
							tvOut.loadFonts(OBLIQUE18);
							tvOut.print("Cannot allocate Mem", 0, 120, true);
							tvOut.setColor(100, 100, 255);
							tvOut.print("Press 1 to load default game", 0, 160, true);
							while (!wiiMoteKeys.b1) delay(50);
							exitMenu = true;
						}
					}
				}
			}
			else
			{
				tvOut.fillScr(0, 0, 0);
				tvOut.setColor(100, 255, 100);
				tvOut.loadFonts(OBLIQUE18);
				tvOut.print("No NES directory", 0, 120, true);
				tvOut.setColor(100, 100, 255);
				tvOut.print("Press 1 to load default game", 0, 160, true);
				while (!wiiMoteKeys.b1) delay(50);
				exitMenu = true;
			}
		} while (!exitMenu);
	}
	else
	{
		tvOut.fillScr(0, 0, 0);
		tvOut.setColor(100, 255, 100);
		tvOut.loadFonts(OBLIQUE18);
		tvOut.print("No SD Card", 0, 120, true);
		tvOut.setColor(255, 100, 100);
		tvOut.print("Press 1 to load default game", 0, 160, true);
		while (!wiiMoteKeys.b1) delay(50);
	}
}

bool listDirectoryAndGetSelectedFile(char * StartFolder, String * selectedFile)
{
	String *		filesList;
	unsigned int *	fileSizes;
	unsigned int *	indexArray, numOfFiles, totalEntries;

	File root;
	File entry;
	root = SD.open(StartFolder);
	numOfFiles = 0;
	totalEntries = 0;
	// First: get the number of files within the directory
	if (!root)
	{
		printf("Error - no such directory...\n");
		return false;
	}

	do
	{
		entry = root.openNextFile();
		if (!entry.isDirectory())
			numOfFiles++;
		totalEntries++;
	} while (entry);

	if (numOfFiles == 0)
	{
		printf("No files in directory\n");
		return false;
	}
	numOfFiles--;
	totalEntries--;

	printf("Number of files: %d\n", numOfFiles);
	printf("Number of entries: %d\n", totalEntries);

	root = SD.open(StartFolder);

	filesList = new String[numOfFiles];

	if (!filesList)
	{
		printf("Cannot allocate memory!\n");
		return false;
	}

	fileSizes = new unsigned int[numOfFiles];

	if (!fileSizes)
	{
		delete[] filesList;
		printf("Cannot allocate memory!\n");
		return false;
	}

	root = SD.open(StartFolder);
	if (!root)
	{
		printf("Error - no such directory...\n");
		return false;
	}
	else
	{
		numOfFiles = 0;
		for (size_t i = 0; i < totalEntries; i++)
		{
			File entry = root.openNextFile();
			if (!entry)
			{
				// no more files
				break;
			}
			if (!entry.isDirectory())
			{
				filesList[numOfFiles] = entry.name();
				filesList[numOfFiles].remove(0, strlen(StartFolder) + 1);
				fileSizes[numOfFiles] = entry.size();
				numOfFiles++;
			}
		}
	}
	for (size_t i = 0; i < numOfFiles; i++)
	{
		printf("%s - %d\n", filesList[i].c_str(), fileSizes[i]);
	}

	unsigned int	markedFileIndex = 0, w, maxFilesPerScreen, startIndex = 0, endIndex, h, fileIndex = 0;
	char			tempStr[50];

	short			jx, jy, y, addVal;

	h = tvOut.getFontHieght();
	maxFilesPerScreen = tvOut.getYSize() / h;
	if (numOfFiles < maxFilesPerScreen)
	{
		endIndex = numOfFiles;
	}
	else
	{
		endIndex = maxFilesPerScreen;
	}

	printf("maxFilesPerScreen = %d\n", maxFilesPerScreen);

	bool updateDisplay = true, fullScreenUpdate = true;

	tvOut.fillScr(0, 0, 0);
	int offset = 0, delta = 0, dir = 1;
	while (1)
	{
		// Display relevant files
		if (contDown)
		{
			addVal = -1;
			while (contDown) delay(1);
			markedFileIndex++;
			if (!(markedFileIndex >= maxFilesPerScreen || markedFileIndex >= endIndex))
			{
				fileIndex++;
				updateDisplay = true;
			}
			else
			{
				if (markedFileIndex == maxFilesPerScreen && endIndex < numOfFiles)
				{
					startIndex++;
					endIndex++;
					fileIndex++;
					updateDisplay = true;
				}
				markedFileIndex--;
			}
		}
		if (contUp)
		{
			addVal = 1;
			while (contUp) delay(1);

			if (markedFileIndex == 0 && startIndex > 0)
			{
				startIndex--;
				endIndex--;
				fileIndex--;
				updateDisplay = true;
			}
			else if (!(markedFileIndex == 0 && startIndex == 0))
			{
				markedFileIndex--;
				fileIndex--;
				updateDisplay = true;
			}
		}

		if (contRight)
		{
			while (contRight) delay(1);
			addVal = -1;
			if ((endIndex + maxFilesPerScreen) < numOfFiles)
			{
				startIndex += (maxFilesPerScreen);
				endIndex += (maxFilesPerScreen);
				fileIndex += (maxFilesPerScreen);
				updateDisplay = true;
				fullScreenUpdate = true;
			}
			else if((startIndex + maxFilesPerScreen) < numOfFiles)
			{
				startIndex = numOfFiles - maxFilesPerScreen;
				endIndex = numOfFiles;
				markedFileIndex = 0;
				fileIndex = numOfFiles - maxFilesPerScreen;
				updateDisplay = true;
				fullScreenUpdate = true;
			}
		}
		if (contLeft)
		{
			while (contLeft) delay(1);
			addVal = 1;
			if (startIndex > maxFilesPerScreen)
			{
				startIndex -= (maxFilesPerScreen);
				endIndex -= (maxFilesPerScreen);
				fileIndex -= (maxFilesPerScreen);
				updateDisplay = true;
				fullScreenUpdate = true;
			}
			else if (startIndex != 0)
			{
				startIndex = 0;
				endIndex = maxFilesPerScreen;
				fileIndex = 0;
				updateDisplay = true;
				fullScreenUpdate = true;
				markedFileIndex = 0;
			}
		}

		if (updateDisplay)
		{
			if (markedFileIndex == 0 || markedFileIndex == (maxFilesPerScreen - 1) || fullScreenUpdate)
			{
				y = 0;
				fullScreenUpdate = false;
				for (size_t j = startIndex; j < endIndex; j++)
				{
					tvOut.setColor(0, 0, 0);
					tvOut.drawRect(0, h*y + 4, tvOut.getXSize() - 1, h*y + h - 1 + 4, true);
					tvOut.setColor(200, 200, 200);
					tvOut.print((char *)filesList[j].c_str(), 2, y * h);
					tvOut.setColor(100, 255, 100);
					sprintf(tempStr, "(%d)", fileSizes[j]);
					w = tvOut.getPrintWidth((char*)filesList[j].c_str());
					tvOut.print(tempStr, 2 + w, y * h);
					y++;
				}
			}
			else
			{
				tvOut.setColor(0, 0, 0);
				tvOut.drawRect(0, h*(markedFileIndex + addVal) + 4, tvOut.getXSize() - 1, h*(markedFileIndex + addVal) + h - 1 + 4, true);
				tvOut.setColor(200, 200, 200);
				tvOut.print((char *)filesList[fileIndex + addVal].c_str(), 2, (markedFileIndex + addVal) * h);
				tvOut.setColor(100, 255, 100);
				sprintf(tempStr, "(%d)", fileSizes[fileIndex + addVal]);
				w = tvOut.getPrintWidth((char*)filesList[fileIndex + addVal].c_str());
				tvOut.print(tempStr, 2 + w, (markedFileIndex + addVal) * h);
			}
			tvOut.setColor(200, 200, 200);
			tvOut.drawRect(0, h*markedFileIndex + 4, tvOut.getXSize() - 1, h*markedFileIndex + h - 1 + 4, true);
			tvOut.setColor(0, 0, 0);
			tvOut.print((char *)filesList[fileIndex].c_str(), 2, markedFileIndex * h);
			sprintf(tempStr, "(%d)", fileSizes[fileIndex]);
			w = tvOut.getPrintWidth((char*)filesList[fileIndex].c_str());
			delta = w + tvOut.getPrintWidth(tempStr) - tvOut.getXSize();
			offset = 0;
			dir = 1;
			tvOut.setColor(100, 255, 100);
			tvOut.print(tempStr, 2 + w, markedFileIndex * h);
			updateDisplay = false;
		}
		if (delta > 2)
		{
			offset += dir;
			tvOut.setColor(200, 200, 200);
			tvOut.drawRect(0, h*markedFileIndex + 4, tvOut.getXSize() - 1, h*markedFileIndex + h - 1 + 4, true);
			tvOut.setColor(0, 0, 0);
			tvOut.print((char *)filesList[fileIndex].c_str(), 2 - offset, markedFileIndex * h);
			sprintf(tempStr, "(%d)", fileSizes[fileIndex]);
			tvOut.setColor(100, 255, 100);
			tvOut.print(tempStr, 2 + w - offset, markedFileIndex * h);
			delay(25);
			if (offset == delta || offset == 0)
			{
				dir *= -1;
			}
		}

		if (contFire)
		{
			*selectedFile = (const char*)StartFolder;
			*selectedFile += '/';
			*selectedFile += filesList[fileIndex];
			delete[] filesList;
			delete[] fileSizes;

			return true;
		}
	}
}
