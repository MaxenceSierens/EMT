main.c

/*
	Joulemeter - Joulemetre
	2018 - 2019
	Louis MASSUCCI - Guillaume SAPRANI
	
	Compiled with CCS C - Compile avec CCS C
*/

#include <24HJ128GP306A.h>
#build (stack = 512)
#device ADC = 10
#device ICD = TRUE, ICD = 2
#device NESTED_INTERRUPTS = FALSE

#use DELAY(clock = 20MHz, crystal = 8MHz)
#use RS232(STREAM = GPS_STREAM, UART1, BAUD = 9600, BITS = 8, PARITY = N, DISABLE_INTS)
#use RS232(STREAM = BL_STREAM, UART2, BAUD = 115200, BITS = 8, PARITY = N, DISABLE_INTS)
#use SPI(MASTER, BAUD = 115200, SPI2, FORCE_HW, BITS = 8, SAMPLE_RISE)
#use I2C(MASTER, SLOW, FORCE_HW, I2C1)

#include <stdlib.h>

/* Language Settings - Parametre de langue */
/* 0 : English - Anglais / 1 : French - Francais */
#define L10N 1

// #define USE_HF

/* The current sensor is an hall-effect based sensor capable of measuring positive and negative current. */
/* For a zero amps input the output is actually not zero but VREF / 2 (3.3 / 2 ~ 1.65V). */
/* Le capteur de courant est un capteur a effet Hall capable de mesurer des courants positifs et negatifs. */
/* Pour un courant nul en entree, la sortie ne vaut pas zero mais VREF / 2 (3.3 / 2 ~ 1.65V). */
#define HALL_MID_POINT 0x01FC
/* Scale values for voltage and current - Valeurs pour la conversion de la tension et du courant */
#define ADC_V_SCALE 0.037
#define ADC_C_SCALE 0.105

/* S25FL256 */
#define MEM_WRR 0x01 /* Write Register */
#define MEM_PP 0x02 /* Page Program */
#define MEM_READ 0x03 /* Read */
#define MEM_RDSR1 0x05 /* Read Status Register 1 */
#define MEM_WREN 0x06 /* Write Enable */
#define MEM_BRAC 0xB9 /* Bank Register Access */
#define MEM_SE 0xD8 /* Sector Erase (256kB) */
#define MEM_BE 0xC7 /* Bulk Erase */
#define MEM_CS PIN_G12 /* Chip Select pin - Broche de selection de la memoire */
#define MEM_PAGE_LENGTH 256 /* Page length - Taille d'une page memoire */
#if (L10N == 1)
#define MEM_HEADER_LENGTH 95 /* Header length - Taille de la ligne l'entete */
#define MEM_UNITS_LENGTH 65 /* Units length - Taille de la ligne d'unite */
#else
#define MEM_HEADER_LENGTH 96
#define MEM_UNITS_LENGTH 65
#endif

#define GPS_LEN_FRAME 100 /* GPS frame length - Taille d'une trame GPS */

#define HF_CS PIN_G13 /* Chip Select pin - Broche de selection de l'emetteur HF */
#define HF_CE PIN_G14 /* Chip Enable pin - Broche d'activation de l'emetteur HF */

/* HF transmitter registers - Registres de l'emetteur HF */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define W_TX_PAYLOAD 0xA0

#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define TX_ADDR 0x10

#if (L10N == 1)
#define BL_SHELL_LENGTH 196 /* Shell text length - Taille du texte de l'invite de commante */
#define BL_ERASE_LENGTH 29 /* Erase text length - Taille du texte du message d'effacement */
#define BL_DONE_LENGTH 6 /* Done text length - Taille du texte du message de confirmation */
#else
#define BL_SHELL_LENGTH 173
#define BL_ERASE_LENGTH 19  
#define BL_DONE_LENGTH 6
#endif

#define LCD_I2C_ADDR 0x4E /* PCF8574A : 0x7E | PCF8574 : 0x4E */
#if (L10N == 1)
#define LCD_ENERGY_STR "ENERGIE%7lu J" /* LCD Energy text - Texte "Energie" sur l'ecran LCD */
#define LCD_VOLTAGE_STR "TENSION   %s V" /* LCD Voltage text - Texte "Tension" sur l'ecran LCD */
#define LCD_CURRENT_STR "COURANT   %s A" /* LCD Current text - Texte "Courant" sur l'ecran LCD */
#define LCD_TIME_STR "TEMPS      %2d:%2d" /* LCD Time text - Texte "Temps" sur l'ecran LCD */
#else
#define LCD_ENERGY_STR "ENERGY %7lu J"
#define LCD_VOLTAGE_STR "VOLTAGE   %s V"
#define LCD_CURRENT_STR "CURRENT   %s A"
#define LCD_TIME_STR "TIME       %2d:%2d"
#endif

unsigned short lcdColdStart;

/*
	Bit-0: RUN/STOP (0: STOP / 1: RUN)
	Bit-1: ADC conversion completed (1: conversion completed)
	Bit-2: GPS GPRMC frame (Reserved do not use!)
	Bit-3: GPS GPGGA frame (Reserved do not use!)
	Bit-4: GPS frame validity (Reserved do not use!)
	Bit-5: GPS end of reception
	Bit-6: Bluetooth end of reception
	Bit-7: Unused
*/
unsigned short systemFlags = 0x00;

float voltageCalc = 0.0;
float voltage;
char voltageString[5];
float currentCalc = 0.0;
float current;
char currentString[5];
float energyCalc;
unsigned long energy = 0;

unsigned short timeSeconds = 0;
unsigned short timeMinutes = 0;


volatile unsigned int adcVoltage;
volatile unsigned int adcCurrent;

unsigned long memStartAddress;
unsigned long memCursorAddress;
unsigned long memStopAddress;
char memBuffer[MEM_PAGE_LENGTH];
#if (L10N == 1)
const char memHeader[] = "Trame Date Heure Latitude Nord/Sud Longitude Est/Ouest Altitude Vitesse Tension Courant Energie";
const char memUnits[] = "- JJMMAA HHMMSS.dd DDmm.mm - DDmm.mm - m km/h V A J";
#else
const char memHeader[] = "Frame Date Hours Latitude North/South Longitude East/West Elevation Speed Voltage Current Energy";
const char memUnits[] = "- DDMMYY HHMMSS.dd DDmm.mm - DDmm.mm - m km/h V A J";
#endif
const char newLine[] = "\n\r";

const char gpsFrameUnavailable[18] = " - - - - - - - - ";
unsigned int gpsFrameNumber = 0;
volatile unsigned short gpsCursor;
volatile char gpsFrameGPRMC[GPS_LEN_FRAME];
volatile char gpsFrameGPGGA[GPS_LEN_FRAME];
char gpsBuffer[GPS_LEN_FRAME];

char hfMessage[32];

unsigned short lcdSwitch = 0;
char lcdLine1[16];
char lcdLine2[16];
unsigned short i2cLCDPort;

#if (L10N == 1)
const char blShellText[] = "Joulemetre - Polytech Nancy (2019)\n\rListe des commandes :\n\r  - 'h' : Affiche ce menu\n\r  - 'r' : Lecture des donnees de course\n\r  - 'E' : Efface la memoire\n\r  - 'A' : Mode automatique\n\r";
const char blEraseText[] = "Effacement de la memoire ... ";
const char blDoneText[] = "Fait\n\r";
#else
const char blShellText[] = "Joulemeter - Polytech Nancy (2019)\n\rList of availables commands:\n\r  - 'h': Display this menu\n\r  - 'r': read data of the last run\n\r  - 'E': erase memory\n\r  - 'A': Automatic\n\r";
const char blEraseText[] = "Erasing memory ... ";
const char blDoneText[] = "done\n\r";
#endif
volatile char blDataReceived;

unsigned short automaticMode = 0xFF;

/* ################ Prototypes ################ */
void adc_compute_float(char* str, float value);
void mem_init(void);
void mem_enable_write(void);
void mem_save(void);
void mem_send_command(unsigned char command, unsigned long address);
unsigned short mem_read_byte(unsigned long address);
void mem_read_page(unsigned long address);
void mem_wait(void);
void mem_enable_write(void);
void mem_erase(void);
void gps_parse(void);
void lcd_init(void);
void lcd_cmd(unsigned short cmd);
void lcd_data(unsigned short data);
void lcd_clear(void);
void nrf_init(void);
void nrf_write_s(unsigned short reg, unsigned short data);
void nrf_write_m(unsigned short reg, unsigned short* data, unsigned short length);
void nrf_send(void);
void blReadMem(void);
void blEraseMem(void);

/* ################ BEGIN ################ */
/* ################ MAIN ################ */
void main(void)
{
	unsigned int cnt;


	setup_adc(ADC_CLOCK_DIV_2 | ADC_TAD_MUL_2);
	setup_adc_ports(sAN0 | sAN3);
	setup_timer1(TMR_INTERNAL | TMR_DIV_BY_1);
	voltageString[4] = '\0';
	currentString[4] = '\0';

	/* Buttons pins as inputs */
	set_tris_d(0xFFFF);

	mem_init();
	lcd_init();
	#ifdef USE_HF
	nrf_init();
	#endif

	enable_interrupts(INT_TIMER1);
	clear_interrupt(INT_TIMER1);
	enable_interrupts(INTR_GLOBAL);

	while(1)
	{
		if(bit_test(systemFlags, 1) && bit_test(systemFlags, 0)) /* ADC conversion complete && "RUN" state */
		{
			bit_clear(systemFlags, 1);
			voltageCalc = (0.03125 * (float)(adcVoltage)) + (0.96875 * voltageCalc);
			voltage = voltageCalc * ADC_V_SCALE;
			if(adcCurrent < HALL_MID_POINT) adcCurrent = HALL_MID_POINT; /* No negative current ! */
			currentCalc = (0.03125 * (float)(adcCurrent - HALL_MID_POINT) + (0.96875 * currentCalc));
			current = currentCalc * ADC_C_SCALE;
			adc_compute_float(voltageString, voltage);
			adc_compute_float(currentString, current);
			energyCalc += voltage * current * 0.0065625;
		}

		if(bit_test(systemFlags, 5)) /* GPS reception complete (every second) */
		{
			/* Buttons */
			if(input_state(pin_D8) == 0) /* START/STOP button */
			{
				bit_clear(systemFlags, 5);
				systemFlags ^= 0x01;
			}
			if(input_state(pin_D9) == 0) /* SELECT button */
			{
				lcdSwitch++;
				if(lcdSwitch > 2) lcdSwitch = 0;
			}
			if(input_state(pin_D10) == 0) #asm goto 0 #endasm; /* RESET button */
		}

		if(bit_test(systemFlags, 5) && bit_test(systemFlags, 0)) /* GPS reception complete and "RUN" state */
		{
			bit_clear(systemFlags, 5);
			energy = (long)(energyCalc + 0.5);
			gps_parse();
			sprintf(memBuffer, "%s %s %s %7lu", gpsBuffer, voltageString, currentString, (long)(energy));
			if(automaticMode == 0xFF) fputs(memBuffer, BL_STREAM); 
			mem_save();
			timeSeconds++;
			if(timeSeconds > 59)
			{
				timeSeconds = 0;
				timeMinutes++;
			}
			if(lcdSwitch == 0) sprintf(lcdLine1, LCD_ENERGY_STR, energy);
			if(lcdSwitch == 1) sprintf(lcdLine1, LCD_VOLTAGE_STR, voltageString);
    		if(lcdSwitch == 2) sprintf(lcdLine1, LCD_CURRENT_STR, currentString);
			sprintf(lcdLine2, LCD_TIME_STR, timeMinutes, timeSeconds);
			lcd_cmd(0x80); /* Cursor set to (0, 0) */
			lcd_cmd(0x00);
			for(cnt = 0; cnt < 16; cnt++) lcd_data(lcdLine1[cnt]);
			lcd_cmd(0xC0); /* Cursor set to (0, 1) */
			lcd_cmd(0x00);
			for(cnt = 0; cnt < 16; cnt++) lcd_data(lcdLine2[cnt]);
			#ifdef USE_HF
			for(cnt = 0; cnt < 16; cnt++) hfMessage[cnt] = lcdLine1[cnt];
			for(cnt = 16; cnt < 32; cnt++) hfMessage[cnt] = lcdLine2[cnt - 16];
			nrf_send();
			#endif
		}

		if(bit_test(systemFlags, 6)) /* Bluetooth reception complete */
		{
			bit_clear(systemFlags, 6);
			switch(blDataReceived)
			{
				case 'h': /* Display help */
				fputs(blShellText, BL_STREAM);
				break;

				case 'r': /* Read last run data */
				blReadMem();
				break;

				case 'E': /* Erase memory */
				blEraseMem();
				break;
				
				case 'A': /* Automatic mode */
				automaticMode = ~automaticMode;
				break;

				default:
				break;
			}
		}

		/* GPS UART1 */
		if(kbhit(GPS_STREAM))
		{
			char gpsReceived;

			gpsReceived = fgetc(GPS_STREAM);
	
			if(gpsReceived == '$') gpsCursor = 0;
			if(gpsReceived == 'C' && gpsCursor == 5)
			{
				bit_set(systemFlags, 2);
				bit_clear(systemFlags, 3);
			}
			if(gpsReceived == 'G' && gpsCursor == 4)
			{
				bit_set(systemFlags, 3);
				bit_clear(systemFlags, 2);
			}
	
			if(bit_test(systemFlags, 2)) gpsFrameGPRMC[gpsCursor] = gpsReceived;
			if(bit_test(systemFlags, 3)) gpsFrameGPGGA[gpsCursor] = gpsReceived;

			gpsCursor++;
	
			if(gpsReceived == '*')
			{
				if(bit_test(systemFlags, 2))
				{
					gpsFrameGPRMC[gpsCursor] = '\0';
					bit_clear(systemFlags, 2);
					bit_clear(systemFlags, 3);
				}
				if(bit_test(systemFlags, 3))
				{
					gpsFrameGPGGA[gpsCursor] = '\0';
					bit_clear(systemFlags, 2);
					bit_clear(systemFlags, 3);
					bit_set(systemFlags, 5);
				}
				gpsCursor = 0;
			}
		}

		/* Bluetooth UART2 */
		if(kbhit(BL_STREAM))
		{
			blDataReceived = fgetc(BL_STREAM);
			bit_set(systemFlags, 6);
		}
	}
}

/* ################ END ################ */

/* ################ Subroutines ################ */
/* ######## ADC ######## */
#INT_TIMER1 level = 6
void timer1_isr(void)
{
	clear_interrupt(INT_TIMER1);

	set_adc_channel(0);
	adcVoltage = read_adc(ADC_START_AND_READ);
	set_adc_channel(3);
	adcCurrent = read_adc(ADC_START_AND_READ);
	bit_set(systemFlags, 1);
}

/*
	adc_compute_float()
	IN: [char*] str
		[float] value
	OUT: NULL
	Comments:
	Convert 'value' to "XX.X" format and store it in variable pointed by '*str'
*/
void adc_compute_float(char* str, float value)
{
	value += 0.05;
	*(str + 3) = ((unsigned short)(value * 10) % 10) + '0';
	*(str + 2) = '.';
	*(str + 1) = ((unsigned short)(value) % 10) + '0';
	*(str) = (((unsigned short)(value) / 10) % 10) + '0';
}

/* ######## Memory ######## */
/*
	mem_init()
	IN: NULL
	OUT: NULL
	Comments:
	mem_init() is responsible of the initialization of memory peripheral, finding free start addresse and writing header
*/
void mem_init(void)
{
	unsigned short cnt;


	/* Wait for memory POR to finish */
	delay_us(200);

	/* Find empty space */
	output_low(MEM_CS);
	while(mem_read_byte(memStartAddress) != 0xFF) memStartAddress += MEM_PAGE_LENGTH;
	memCursorAddress = memStartAddress;
	output_high(MEM_CS);

	/* Enable write */
	mem_enable_write();

	/* Write headers and units */
	output_low(MEM_CS);
	mem_send_command(MEM_PP, memCursorAddress);
	for(cnt = 0; cnt < MEM_HEADER_LENGTH; cnt++) spi_write2(memHeader[cnt]);
	for(cnt = MEM_HEADER_LENGTH; cnt < (MEM_PAGE_LENGTH - 2); cnt++) spi_write2(0x00);
	for(cnt = 0; cnt < 2; cnt++) spi_write2(newLine[cnt]);
	for(cnt = 0; cnt < MEM_UNITS_LENGTH; cnt++) spi_write2(memUnits[cnt]);
	for(cnt = MEM_UNITS_LENGTH; cnt < (MEM_PAGE_LENGTH - 2); cnt++) spi_write2(0x00);
	for(cnt = 0; cnt < 2; cnt++) spi_write2(newLine[cnt]);
	output_high(MEM_CS);
	mem_wait();
	memCursorAddress += (2 * MEM_PAGE_LENGTH);
}

/*
	mem_enable_write()
	IN: NULL
	OUT: NULL
	Comments:
	The write enable (WREN) command must be sent prior to any command listed after : RESET, MEM_PP, MEM_SE, MEM_BE, MEM_WRDI, MEM_WRR, MEM_QPP and MEM_OTPP
*/
void mem_enable_write(void)
{
	output_low(MEM_CS);
	spi_write2(MEM_WREN);
	output_high(MEM_CS);
	mem_wait();
}

/*
	mem_save()
	IN: NULL
	OUT: NULL
	Comments:
	This function save the content of 'memBuffer' to the next free location in memory
*/
void mem_save(void)
{
	unsigned short a;

	mem_enable_write();
	delay_cycles(8);
	output_low(MEM_CS);
	mem_send_command(MEM_PP, memCursorAddress);
	for(a = 0; a < (MEM_PAGE_LENGTH - 2); a++) spi_write2(memBuffer[a]);
	for(a = 0; a < 2; a++) spi_write2(newLine[a]);
	output_high(MEM_CS);
	mem_wait();
	memCursorAddress += MEM_PAGE_LENGTH;
}

/*
	mem_send_command()
	IN: [char] command
		[long] address
	OUT: NULL
*/
void mem_send_command(unsigned char command, unsigned long address)
{
	spi_write2(command);
	spi_write2(make8(address, 2));
	spi_write2(make8(address, 1));
	spi_write2(make8(address, 0));
}

/*
	mem_read_byte()
	IN: [long] address
	OUT: [short] data
*/
unsigned short mem_read_byte(unsigned long address)
{
	unsigned short data;


	output_low(MEM_CS);
	mem_send_command(MEM_READ, address);
	data = spi_read2(0);
	output_high(MEM_CS);


	return(data);
}

/*
	mem_read_page()
	IN: [long] address
	OUT: NULL
	Comments:
	The content of memory pointed by 'address' is read and store into 'memBuffer'
*/
void mem_read_page(unsigned long address)
{
	unsigned int cnt;


	output_low(MEM_CS);
	mem_send_command(MEM_READ, address);
	for(cnt = 0; cnt < MEM_PAGE_LENGTH; cnt++) memBuffer[cnt] = spi_read2(0);
	output_high(MEM_CS);
}

/*
	mem_wait()
	IN: NULL
	OUT: NULL
	Comments:
	Wait until memory is ready to accept commands
*/
void mem_wait(void)
{
	unsigned short memStatus;

	delay_cycles(8);
	output_low(MEM_CS);
	spi_write2(MEM_RDSR1);
	do
	{
		memStatus = spi_read2(0);
	}
	while(bit_test(memStatus, 0));
	output_high(MEM_CS);
}

/*
	mem_erase()
	IN: NULL
	OUT: NULL
	Comments:
	/!\ Erase the entire memory /!\
*/
void mem_erase(void)
{
	mem_enable_write();
	delay_cycles(8);
	output_low(MEM_CS);
	spi_write2(MEM_BE);
	output_high(MEM_CS);
	mem_wait();
}

/* ######## GPS ######## */

/*
	gps_parse()
	IN: NULL
	OUT: NULL
	Comments:
	Get informations (Time, data, speed, ...) from GPS frame
*/
void gps_parse(void)
{
  unsigned short cursor = 0;
  unsigned short commaPos = 0;
  unsigned short counter;
  unsigned short cnt;
  
  do
  {
    if(gpsFrameGPRMC[cursor] == ',')
    {
      cursor++;
      commaPos++;
      switch(commaPos)
      {
        case 1: /* Time stamp */
        if(bit_test(systemFlags, 4))
        {
          counter = 0;
          while(gpsFrameGPRMC[cursor + counter] != ',')
          {
          	gpsBuffer[12 + counter] = gpsFrameGPRMC[cursor + counter];
            counter++;
          }
          cursor += (counter - 1);
        }
        break;
          
        case 2: /* Validity */
        if(!(bit_test(systemFlags, 4)))
        {
          if(gpsFrameGPRMC[cursor] == 'A')
          {
            commaPos = 0;
            cursor = 0;
            bit_set(systemFlags, 4);
          }
          else
          {
            gpsFrameNumber++;
            gpsBuffer[0] = ((gpsFrameNumber / 1000) % 10) + '0';
  			gpsBuffer[1] = ((gpsFrameNumber / 100) % 10) + '0';
  			gpsBuffer[2] = ((gpsFrameNumber / 10) % 10) + '0';
  			gpsBuffer[3] = (gpsFrameNumber % 10) + '0';
  			gpsBuffer[4] = ' ';
  			for(cnt = 0; cnt < 18; cnt++) gpsBuffer[5 + cnt] = gpsFrameUnavailable[cnt];
  			
            return;
          }
        }
        break;
        
        case 3: /* Latitude */
        if(bit_test(systemFlags, 4))
        {     
          counter = 0;
          while(gpsFrameGPRMC[cursor + counter] != ',')
          {
          	gpsBuffer[22 + counter] = gpsFrameGPRMC[cursor + counter];
            counter++;
          }
          cursor += (counter - 1);
        }
        break;
          
        case 4: /* Orientation (North / South) */
        if(bit_test(systemFlags, 4))
        {
          gpsBuffer[34] = gpsFrameGPRMC[cursor];
          cursor++;
        }
        break;
            
        case 5: /* Longitude */
        if(bit_test(systemFlags, 4))
        {
          counter = 0;
          while(gpsFrameGPRMC[cursor + counter] != ',')
          {
            gpsBuffer[36 + counter] = gpsFrameGPRMC[cursor + counter];
            counter++;
          }
          cursor += (counter - 1);
        }
        break;
          
        case 6: /* Orientation (East / West) */
        if(bit_test(systemFlags, 4))
        {
          gpsBuffer[48] = gpsFrameGPRMC[cursor];
          cursor++;
        }
        break;
        
        case 7: /* Speed */
        if(bit_test(systemFlags, 4))
        {
	      counter = 0;
	      do
	      {
	        gpsBuffer[57 + counter] = gpsFrameGPRMC[cursor + counter];
	        counter++;
          }
	      while(gpsFrameGPRMC[cursor + counter] != ',');
	      cursor += (counter - 1);
	    }
	    break;

        case 9: /* Date stamp */
        if(bit_test(systemFlags, 4))
        {
          counter = 0;
          do
          {
          	gpsBuffer[5 + counter] = gpsFrameGPRMC[cursor + counter];
            counter++;
          }
          while(gpsFrameGPRMC[cursor + counter] != ',');
          cursor += (counter - 1);
        }
        break;
          
        default:
        break;
      }
    }
    else cursor++;
  }
  while((gpsFrameGPRMC[cursor] != '*')  && (cursor < GPS_LEN_FRAME));

  /* GPGGA Frame */
  counter = 0;
  cursor = 0;
  
  for(cnt = 0; cnt < 9; cnt++)
  {
    do
    {
      cursor++;
    }
    while(gpsFrameGPGGA[cursor] != ',');
  } 
  cursor++;
  do
  {
    gpsBuffer[50 + counter] = gpsFrameGPGGA[cursor + counter];
    counter++;
  }
  while(gpsFrameGPGGA[cursor + counter] != ',');
  
  bit_clear(systemFlags, 4);
  gpsFrameNumber++;

  gpsBuffer[0] = ((gpsFrameNumber / 1000) % 10) + '0';
  gpsBuffer[1] = ((gpsFrameNumber / 100) % 10) + '0';
  gpsBuffer[2] = ((gpsFrameNumber / 10) % 10) + '0';
  gpsBuffer[3] = (gpsFrameNumber % 10) + '0';
  
  gpsBuffer[4] = ' ';
  gpsBuffer[11] = ' ';
  gpsBuffer[21] = ' ';
  gpsBuffer[33] = ' ';
  gpsBuffer[35] = ' ';
  gpsBuffer[47] = ' ';
  gpsBuffer[49] = ' ';
  gpsBuffer[56] = ' ';
  gpsBuffer[66] = ' ';
  gpsBuffer[67] = '\0';
}


/* ######## LCD ######## */
/*
	lcd_init()
	IN: NULL
	OUT: NULL
	Comments:
	Initialize LCD over I2C
*/
void lcd_init(void)
{
	i2cLCDPort = 0x08;
	
	if(lcdColdStart == 0x00)
	{
		/* Power ON */
		delay_ms(20);
		lcd_cmd(0x80);
		delay_ms(5);
		lcd_cmd(0x30);
		delay_us(200);
		lcd_cmd(0x30);
	
		/* Configuration */
		lcd_cmd(0x20); /* 4-bit mode */
	}
	lcd_cmd(0x20); /* Number of display line / Character font */
	lcd_cmd(0x80);
	lcd_cmd(0x00); /* Display ON / Cursor OFF / Cursor blink OFF */
	lcd_cmd(0xC0);
	lcd_cmd(0x00); /* Clear display */
	lcd_cmd(0x10);
	lcd_cmd(0x00); /* Entry mode set */
	lcd_cmd(0x60);

	lcdColdStart = 0xFF;
}

/*
	lcd_cmd()
	IN: [short] cmd
	OUT: NULL
	Comments:
	Send 'cmd' to LCD over I2C
*/
void lcd_cmd(unsigned short cmd)
{
	bit_clear(i2cLCDPort, 0); /* RS = 0 */
	bit_set(i2cLCDPort, 3);
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	i2cLCDPort = (i2cLCDPort & 0x0F) | (cmd & 0xF0);
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	bit_set(i2cLCDPort, 2); /* EN = 1 */
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	delay_ms(4);
	bit_clear(i2cLCDPort, 2); /* EN = 0 */
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
}

/*
	lcd_data()
	IN: [short] data
	OUT: NULL
	Comments:
	Send 'data' to LCD over I2C
*/
void lcd_data(unsigned short data)
{
	bit_set(i2cLCDPort, 0); /* RS = 1 */
	bit_set(i2cLCDPort, 3);
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	i2cLCDPort = (i2cLCDPort & 0x0F) | (data & 0xF0); /* LSB */
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	bit_set(i2cLCDPort, 2); /* EN = 1 */
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	delay_us(40);
	bit_clear(i2cLCDPort, 2); /* EN = 0 */
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	i2cLCDPort = (i2cLCDPort & 0x0F) | ((data << 4) & 0xF0); /* MSB */
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	bit_set(i2cLCDPort, 2); /* EN = 1 */
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
	delay_us(40);
	bit_clear(i2cLCDPort, 2); /* EN = 0 */
	i2c_start();
	i2c_write(LCD_I2C_ADDR);
	i2c_write(i2cLCDPort);
	i2c_stop();
}

/*
	lcd_clear()
	IN: NULL
	OUT: NULL
	Comments:
	Clear screen and set onscreen cursor to (0, 0)
*/
void lcd_clear(void)
{
	/* Remise en (0,0) du curseur */
	lcd_cmd(0x80);
	lcd_cmd(0x00);

	/* Effacement de l'\E9cran */
	lcd_cmd(0x00);
	lcd_cmd(0x10);
}

/* ######## HF ######## */
#ifdef USE_HF
void nrf_init(void)
{
	nrf_write_s(EN_AA, 0x00); /* Disable auto-ackowledgment */
	nrf_write_s(EN_RXADDR, 0x00); /* Disable RX pipe */
	nrf_write_s(SETUP_RETR, 0x00); /* Disable auto retransmit */
	nrf_write_s(RF_CH, 0x04); /* Set TX on channel 4 */
	nrf_write_s(RF_SETUP, 0x26); /* Data rate : 250kps / Output power : 0dBm */
	nrf_write_m(TX_ADDR, (byte*)("EMT19"), 5);
	nrf_write_s(CONFIG, 0x02); /* Power UP and set TX mode */
}

void nrf_write_s(unsigned short reg, unsigned short data)
{
	output_low(HF_CS);
	spi_write2(W_REGISTER | reg);
	spi_write2(data);
	output_high(HF_CS);
}

void nrf_write_m(unsigned short reg, unsigned short* data, unsigned short length)
{
	unsigned short cnt;


	output_low(HF_CS);
	spi_write2(W_REGISTER | reg);
	for(cnt = 0; cnt < length; cnt++) spi_write2(*(data + cnt));
	output_high(HF_CS);
}

void nrf_send(void)
{
	unsigned short cnt;


	output_low(HF_CS);
	spi_write2(W_TX_PAYLOAD);
	for(cnt = 0; cnt < 32; cnt++) spi_write2(hfMessage[cnt]);
	output_high(HF_CS);
}
#endif

/* ######## Bluetooth ######## */
void blReadMem(void)
{
	memStopAddress = memCursorAddress;
	memCursorAddress = memStartAddress;
	while(memCursorAddress < memStopAddress)
	{
		mem_read_page(memCursorAddress);
		fputs(memBuffer, BL_STREAM);
		memCursorAddress += MEM_PAGE_LENGTH;
	}
}

void blEraseMem(void)
{
	fputs(blEraseText, BL_STREAM);
	mem_erase();
	fputs(blDoneText, BL_STREAM);
}
