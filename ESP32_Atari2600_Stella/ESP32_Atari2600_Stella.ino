/*
 Name:		ESP32_Atari2600_Odroid.ino
 Created:	31-Aug-21 4:04:10 PM
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

unsigned int ROMsize;
unsigned char * ROMtoLoad;
unsigned char * frameBuffer; // For emulator

#define SOUNDPIN 5
hw_timer_t * timer = NULL;
#define AUDIO_SAMPLE_RATE (31400)
static unsigned int soundBufferIndex = AUDIO_SAMPLE_RATE / 60;
static int16_t* sampleBuffer;

void IRAM_ATTR sound_ISR()
{
	if (soundBufferIndex < (AUDIO_SAMPLE_RATE / 60))
	{
		//sigmaDeltaWrite(0, (sampleBuffer[soundBufferIndex++] >> 8));
		LEDC.channel_group[0].channel[0].duty.duty = (sampleBuffer[soundBufferIndex++] >> 8) << 4; // 25 bit (21.4)
		LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1; // This is the output enable control bit for channel
		LEDC.channel_group[0].channel[0].conf1.duty_start = 1; // When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware
		LEDC.channel_group[0].channel[0].conf0.clk_en = 1;
	}
}

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
PCF8574 pcf8574(0x21);

unsigned int contLeft = 1, contDown = 1, contUp = 1, contRight = 1, contStart = 1, contSelect = 1, contFire = 1, contHome = 1;

void controllerTask(void* arg)
{
	uint16_t bClick;
	unsigned char IOreads = 0;
	while (1)
	{
		Wiimote::handle();
		Wiimote::readKeys(&wiiMoteKeys);
		contLeft = (digitalRead(14) == LOW) | (wiiMoteKeys.aUp & wiiMoteConnected);
		contDown = (digitalRead(21) == LOW) | (wiiMoteKeys.aLeft & wiiMoteConnected);
		contUp = (digitalRead(22) == LOW) | (wiiMoteKeys.aRight & wiiMoteConnected);
		contRight = (digitalRead(27) == LOW) | (wiiMoteKeys.aDown & wiiMoteConnected);
		contHome = (digitalRead(26) == LOW) | (wiiMoteKeys.bHome & wiiMoteConnected);
		contStart = (digitalRead(34) == LOW) | (wiiMoteKeys.bMinus & wiiMoteConnected);
		contFire = (digitalRead(32) == LOW) | (wiiMoteKeys.b1 & wiiMoteConnected);
		contSelect = (digitalRead(35) == LOW) | (wiiMoteKeys.bPlus & wiiMoteConnected);

		if (Wiimote::isConnected())
			wiiMoteConnected = true;
		else
			wiiMoteConnected = false;
		if (contHome)
		{
			ESP.restart();
		}
		delay(50);
	}
}

//hw_timer_t * timer = NULL;

void loadROM();

SPIClass SDspi(HSPI); // VSPI is used by LCD
bool SDmountOK;

#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_ota_ops.h"

// Stella
#include "src/Console.h"
#include "src/Cart.h"
#include "src/Props.h"
#include "src/MD5.h"
#include "src/Sound.h"
#include "src/OSystem.h"
#include "src/TIA.h"
#include "src/PropsSet.h"
#include "src/Switches.h"
#include "src/SoundSDL.h"

#include <dirent.h>
#include <string.h>
#include <ctype.h>

#define ESP32_PSRAM (0x3f800000)

QueueHandle_t vidQueue;

#define STELLA_WIDTH 160
#define STELLA_HEIGHT 250
//uint8_t * framebuffer;// [STELLA_WIDTH * STELLA_HEIGHT];

int videoWidth;
int videoHeight;
int renderFrames = 0;

static Console *console = 0;
static Cartridge *cartridge = 0;
static Settings *settings = 0;
static OSystem* osystem;
static uint32_t tiaSamplesPerFrame;


void stella_init(unsigned char* rom, unsigned int romSize)
{
	unsigned int size = romSize;

	void* data = (void *)rom;

	printf("Stella init...\n");

	string cartMD5 = MD5((uInt8*)data, (uInt32)size);

	osystem = new OSystem();

	printf("Stella init...new OSystem()\n");
	Properties props;
	osystem->propSet().getMD5(cartMD5, props);

	// Load the cart
	string cartType = props.get(Cartridge_Type);
	string cartId;//, romType("AUTO-DETECT");
	printf("cartType = %s\n", cartType.c_str());
	printf("%s: HEAP:0x%x (%#08x)\n",
		__func__,
		esp_get_free_heap_size(),
		heap_caps_get_free_size(MALLOC_CAP_DMA));
	settings = new Settings(osystem);
	settings->setValue("romloadcount", false);
	printf("Stella init: Cartridge::create\n");
	cartridge = Cartridge::create((const uInt8*)data, (uInt32)size, cartMD5, cartType, cartId, *osystem, *settings);
	printf("Stella init: Cartridge::create - done\n");
	if (cartridge == 0)
	{
		printf("Stella: Failed to load cartridge.\n");
		abort();
	}
	else
	{
		printf("Stella: cartridge loaded\n");
	}

	// Create the console
	console = new Console(osystem, cartridge, props);
	osystem->myConsole = console;

	printf("Stella: Console loaded\n");

	// Init sound and video
	console->initializeVideo();
	console->initializeAudio();

	// Get the ROM's width and height
	TIA& tia = console->tia();
	videoWidth = tia.width();
	videoHeight = tia.height();

	printf("videoWidth = %d, videoHeight = %d\n", videoWidth, videoHeight);

	const uint32_t *palette = console->getPalette(0);
	for (int i = 0; i < 256; ++i)
	{
		uint32_t color = palette[i];

		uint16_t r = (color >> 16) & 0xff;
		uint16_t g = (color >> 8) & 0xff;
		uint16_t b = (color >> 0) & 0xff;
		tvOut.updatePalette(i, r, g, b);
	}

	tiaSamplesPerFrame = (uint32_t)(31400.0f / console->getFramerate());
	printf("Console targeted FPS: %f\n", console->getFramerate());
}

/*void stella_step()
{
	// Process input
	static Event &ev = osystem->eventHandler().event();

	ev.set(Event::Type(Event::JoystickZeroUp), contUp);// gamepad->values[ODROID_INPUT_UP]);
	ev.set(Event::Type(Event::JoystickZeroDown), contDown);//gamepad->values[ODROID_INPUT_DOWN]);
	ev.set(Event::Type(Event::JoystickZeroLeft), contLeft);//gamepad->values[ODROID_INPUT_LEFT]);
	ev.set(Event::Type(Event::JoystickZeroRight), contRight);//gamepad->values[ODROID_INPUT_RIGHT]);
	ev.set(Event::Type(Event::JoystickZeroFire), contFire);//gamepad->values[ODROID_INPUT_A]);
	ev.set(Event::Type(Event::ConsoleSelect), contSelect);//gamepad->values[ODROID_INPUT_SELECT]);
	ev.set(Event::Type(Event::ConsoleReset), contStart);//gamepad->values[ODROID_INPUT_START]);
	//ev.set(Event::Type(Event::PaddleZeroFire), wiiMoteKeys.b2);//gamepad->values[ODROID_INPUT_START]);

	//Tell all input devices to read their state from the event structure
	console->controller(Controller::Left).update();
	console->controller(Controller::Right).update();
	console->switches().update();

	// Emulate
	static TIA& tia = console->tia();
	tia.update();

	static SoundSDL *sound = (SoundSDL*)&osystem->sound();
	sound->processFragment(sampleBuffer, AUDIO_SAMPLE_RATE / 60);
	soundBufferIndex = 0;
}*/
#define UPSCALE
#define THROTTLE_60FPS
#define TICKS_PER_FRAME (240000000/75) /* CPU freq / FPS */
bool RenderFlag;
unsigned int renderCounter = 0;
unsigned int * fb;
void stellaTask(void *arg)
{
	printf("Stella task started...\n");
	stella_init(ROMtoLoad, ROMsize);

	int steps = 0;
	unsigned long time = ESP.getCycleCount(), throttle, cycles;
	int frame = 0;
	//char tmpStr[10] = { "0.0" };

	static TIA& tia = console->tia();
	static Event &ev = osystem->eventHandler().event();
	static SoundSDL *sound = (SoundSDL*)&osystem->sound();

	while (1)
	{
		renderCounter++;
		if (renderCounter > 3)
		{
			renderCounter = 0;
		}
		throttle = ESP.getCycleCount();

		if (renderCounter <= 1)
		{
			RenderFlag = true;
		}
		else
			RenderFlag = false;
		// stella_step();
		//static Event &ev = osystem->eventHandler().event();

		ev.set(Event::Type(Event::JoystickZeroUp), contUp);// gamepad->values[ODROID_INPUT_UP]);
		ev.set(Event::Type(Event::JoystickZeroDown), contDown);//gamepad->values[ODROID_INPUT_DOWN]);
		ev.set(Event::Type(Event::JoystickZeroLeft), contLeft);//gamepad->values[ODROID_INPUT_LEFT]);
		ev.set(Event::Type(Event::JoystickZeroRight), contRight);//gamepad->values[ODROID_INPUT_RIGHT]);
		ev.set(Event::Type(Event::JoystickZeroFire), contFire);//gamepad->values[ODROID_INPUT_A]);
		ev.set(Event::Type(Event::ConsoleSelect), contSelect);//gamepad->values[ODROID_INPUT_SELECT]);
		ev.set(Event::Type(Event::ConsoleReset), contStart);//gamepad->values[ODROID_INPUT_START]);

		//Tell all input devices to read their state from the event structure
		console->controller(Controller::Left).update();
		//console->controller(Controller::Right).update();
		console->switches().update();

		// Emulate
		//static TIA& tia = console->tia();
		tia.update();

		//static SoundSDL *sound = (SoundSDL*)&osystem->sound();
		sound->processFragment(sampleBuffer, AUDIO_SAMPLE_RATE / 60);
		soundBufferIndex = 0;
		// Stella step
		steps++;
		if (RenderFlag)
		{
			//static TIA& tia = console->tia();
			fb = (unsigned int *)tia.currentFrameBuffer();
			xQueueSend(vidQueue, &fb, portMAX_DELAY);
#ifdef noTHROTTLE_60FPS
			while (soundBufferIndex != (AUDIO_SAMPLE_RATE / 60))
				delay(1);
#endif
		}

		if (steps == 59)
		{
			steps = 0;
			time = ESP.getCycleCount() - time;
			printf("FPS: %.02f\n", (240000000.0 / time) * 60.0);
			//sprintf(tmpStr, "FPS: %.02f\n", (240000000.0 / time) * 60.0);
			time = ESP.getCycleCount();
			//printf("Buffer address %X\n", fb);
		}
		feedTheWDog();
		//tvOut.print(tmpStr, 10, 240);
	}
}

void videoTask(void *arg)
{
	unsigned int * lineBuffer, *lineBuffer2, color, pixelData0, pixelData1, scaleX, scaleYcounter, lineCounter = 0;

	while (1)
	{
		// memcpy(framebuffer, param, sizeof(framebuffer));
		uint8_t* param;
		xQueueReceive(vidQueue, &param, portMAX_DELAY);
		scaleYcounter = 0;
		lineCounter = 0;
		if (videoHeight == 210)
		{
			for (size_t line = 0; line < videoHeight; line++)
			{
				lineBuffer = (unsigned int *)tvOut.getLineBufferAddress(lineCounter);
				lineBuffer2 = (unsigned int *)tvOut.getLineBufferAddress(lineCounter + 1);
				scaleX = 0;
				for (size_t x = 0; x < STELLA_WIDTH / 4; x++)
				{
					color = fb[line * STELLA_WIDTH / 4 + x];
					pixelData0 = (color & 0xff000000) | ((color >> 8) & 0x00ffff00) | ((color >> 16) & 0x000000ff);
					pixelData1 = (color & 0x000000ff) | ((color << 8) & 0x00ffff00) | ((color << 16) & 0xff000000);
					if (scaleYcounter == 3)
					{
						lineBuffer[scaleX] = pixelData1;
						lineBuffer2[scaleX++] = pixelData1;
						lineBuffer[scaleX] = pixelData0;
						lineBuffer2[scaleX++] = pixelData0;
					}
					else
					{
						lineBuffer[scaleX++] = pixelData1;
						lineBuffer[scaleX++] = pixelData0;
					}
				}
				if (scaleYcounter == 3)
				{
					scaleYcounter = 0;
					lineCounter += 2;
				}
				else
				{
					lineCounter++;
					scaleYcounter++;
				}
			}
		}
		else
		{
			for (size_t line = 0; line < videoHeight; line++)
			{
				lineBuffer = (unsigned int *)tvOut.getLineBufferAddress(line);
				scaleX = 0;
				for (size_t x = 0; x < STELLA_WIDTH / 4; x++)
				{
					color = fb[line * STELLA_WIDTH / 4 + x];
					pixelData0 = (color & 0xff000000) | ((color >> 8) & 0x00ffff00) | ((color >> 16) & 0x000000ff);
					pixelData1 = (color & 0x000000ff) | ((color << 8) & 0x00ffff00) | ((color << 16) & 0xff000000);
					lineBuffer[scaleX++] = pixelData1;
					lineBuffer[scaleX++] = pixelData0;
				}
			}
		}
	}
	feedTheWDog();
}

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

	printf("Starting emulator\n");
	if (!tvOut.init())
	{
		printf("Error initializing video engine!\n");
		while (1);
	}
	else
	{
		printf("TV Out engine is running...\n");
	}

	frameBuffer = /*new uInt8[160 * 286];//*/(unsigned char*)heap_caps_malloc(160 * 276, MALLOC_CAP_DMA);
	
	if (!frameBuffer)
	{
		printf("Cannot allocate frame buffer for emulator!\n");
		while (1);
	}

	if (psramInit())
	{
		printf("PSRAM init OK\n");
		Wiimote::init(true);
	}
	else
		Wiimote::init(false);
	printf("PSRAM size: %d\n", ESP.getFreePsram());

	delay(250);

	//pcf8574.begin();

	//pcf8574.pinMode(P0, OUTPUT); // SD CS

	SDspi.begin(15, 18, 13, 4);
	//pcf8574.digitalWrite(P0, LOW); // Set SD card chip select to low (enabled)
	delay(500);

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

	xTaskCreatePinnedToCore(controllerTask, "cont_task", 1024 * 2, NULL, 5, NULL, 1);
	// Audio Init - handled by main CPU
	pinMode(SOUNDPIN, OUTPUT);

	ledcSetup(0, 2000000, 7);    // 625000 khz is as fast as we go w 7 bits
	ledcAttachPin(SOUNDPIN, 0);
	ledcWrite(0, 0);

	timer = timerBegin(1, 20, true); // Set to run @ 4MHz
	timerAttachInterrupt(timer, &sound_ISR, true);
	timerAlarmWrite(timer, 4000000 / AUDIO_SAMPLE_RATE, true); //AUDIO_SAMPLE_RATE
	timerAlarmEnable(timer);
}

void loop()
{
	printf("stella-go started.\n");

	loadROM();
	tvOut.fillScr(0);

	printf("HEAP:0x%x (%#08x)\n",
		esp_get_free_heap_size(),
		heap_caps_get_free_size(MALLOC_CAP_DMA));
	heap_caps_malloc_extmem_enable(1024 * 1024 * 4);

	// Audio
	sampleBuffer = (int16_t*)ps_malloc(/*(AUDIO_SAMPLE_RATE / 60)*/1024 * sizeof(int16_t));//malloc(tiaSamplesPerFrame * sizeof(int16_t));
	if (!sampleBuffer)
		abort();

	//odroid_audio_init(AUDIO_SAMPLE_RATE);

	vidQueue = xQueueCreate(1, sizeof(uint16_t*));
	xTaskCreatePinnedToCore(&videoTask, "videoTask", 1024 * 1, NULL, 5, NULL, 1);
	//stellaTask
	xTaskCreatePinnedToCore(&stellaTask, "stellaTask", 1024 * 6, NULL, 1, NULL, 0);
	/*
	unsigned char * lineBuffer;
	int steps = 0;
	unsigned long time = ESP.getCycleCount();
	stella_init(ROMtoLoad, ROMsize);

	while (1)
	{
		stella_step();
		TIA& tia = console->tia();
		uint8_t* fb = tia.currentFrameBuffer();
		for (int line = 0; line < videoHeight; line++)
		{
			lineBuffer = tvOut.getLineBufferAddress(line);
			for (int x = 0; x < STELLA_WIDTH; x++)
			{
				lineBuffer[x] = fb[line * STELLA_WIDTH + x];
			}
		}
		//feedTheWDog();
		//vTaskDelay(1);
		steps++;
		if (steps == 60)
		{
			steps = 0;
			time = ESP.getCycleCount() - time;
			printf("FPS = %f\n", 240000000 / time);
			printf("Buffer address %X\n", fb);
		}
	}
	*/
#if 0
	stella_init(ROMtoLoad, ROMsize);

	int steps = 0;
	unsigned char * lineBuffer;
	unsigned long time = ESP.getCycleCount();
	int frame = 0;

	static const bool renderTable[8] = {
		true, false,
		false, true,
		true, false,
		false, true };
	uint8_t* fb;
	while (1)
	{
		RenderFlag = renderTable[frame & 7];
		stella_step();
		steps++;

		//if (RenderFlag)
		//{
		TIA& tia = console->tia();
		fb = tia.currentFrameBuffer();
		for (size_t line = 0; line < videoHeight; line++)
		{
			lineBuffer = tvOut.getLineBufferAddress(line);
			for (size_t x = 0; x < STELLA_WIDTH; x++)
			{
				lineBuffer[x] = fb[line * STELLA_WIDTH + x];
			}
		}
		//xQueueSend(vidQueue, &fb, portMAX_DELAY);
	//}
		if (steps == 60)
		{
			steps = 0;
			time = ESP.getCycleCount() - time;
			printf("FPS = %f\n", 240000000 / time);
			printf("Buffer address %X\n", fb);
		}
	}
#else
	while (1)
		delay(1000);
#endif
}

#define ESP32_FLASH_SECTOR_SIZE (4096)

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
	tvOut.print("Atari 2600 emulator", 0, 30, true);
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
			if (listDirectoryAndGetSelectedFile("/A2600", &pickedFile))
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
					gameRomPointer = (unsigned char *)ps_malloc(fileSize); //A2600 Kart max size
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

	bool updateDisplay = true, fullScreenUpdate = false;

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
