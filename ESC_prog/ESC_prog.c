/*
 * Created: 21.11.2012 18:01:26
 * Author: VladaS
 
 * using lib. display.h and bitops.h - by Petr Kaèer
 
 * Program for Electric-Scooter controller with 8x2 char LCD,
 measure: ESC current, battery voltage, speed, and control input 1-4V
 
 PORT configuration:
 
 * portB0-7 - display data (display_port) + programing input (MISO,MOSI,SCK)
 * portD0 - SF output PWM for fan
		1 - SW output PWM for ESC 50Hz, positive pulse 1-2ms
		2 - SV impulse input - speed (interrupt 0)
		3 - OFF signal - (interrupt 1)
		4 
		5 - R/S - display
		6 - R/W - display
		7 - E - display
 * portC0 - button 1
		1 - button 2
		2 - SI analog input - current
		3
		4 - SU analog input - voltage
		5 - SA analog input - acceleration (control)
		6 - reset - only for programing 

 */ 


//includes
/*----------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "bitops.h"
#include "display.h"

//define ports
/*----------------------------------------------*/
#define OUTPUT PORTD
#define INPUT PORTC

//OUTPUT pins
#define SF 0
#define SW 1

//INPUT pins
#define BTN1 0
#define BTN2 1
#define SI 2
#define SU 4
#define SA 5


/*----------------------------------
Global variables definition
-----------------------------------*/

//sums for PII regulator
int sum1 = 0;
unsigned int sum2 = 0;

//measured values
unsigned char wantedSpeed = 0;			//output for the engine controller 0-255

unsigned char wantedAcceleration = 0;	//wanted input 0-255
unsigned char actualCurrent = 0;		//0-50A 0-255
unsigned char actualVoltage = 0;		//12.8-16.8V 80-180

unsigned char CycleTime = 0;			//time of actual cycle			x2 ms
unsigned char lastCyclePeriod = 0;		//last measured cycle period	x2 ms
unsigned char actualSpeed = 0;			//last speed info				1/4 km/h

unsigned long distance=0;				//travel distance 			cycles 
unsigned long totalDistance = 0;		//total travel distance    (saving to eeprom)

unsigned long consumedCapacity=0;		//consumed						256 mAs
unsigned long totalConsumedCapacity = 0; //total consumed capacity      (saving to eeprom) in mAh

/*---------------------------
display line mode 
1 total distance (cycles)
2 total consumed capacity (mAh)
3 distance (cycles)
4 consumed capacity (256 = 1 mAh)
5 rest capacity (%)
6 voltage
7 current (255 = 50 A)
8 speed
------------------------------*/

unsigned char LineMode; //mode of 1st and 2nd line on display (4b/4b)

unsigned char displayPaused = 0;		// 1 -> display is paused
unsigned char displayPausedCounter = 0; // decrement every 20ms -> 50 = 1s, if is 0 -> display is unpaused

unsigned char lastButtonState = 0;		//pressed buttons


//conversion tables:
const unsigned char tabA[194] PROGMEM = {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,8,8,9,9,9,10,10,11,11,12,12,13,13,14,14,15,16,16,17,17,18,19,20,20,21,22,23,24,24,25,26,27,28,29,30,31,32,33,34,35,36,38,39,40,41,43,44,45,47,48,49,51,52,54,55,57,58,60,62,63,65,67,69,71,72,74,76,78,80,82,84,86,89,91,93,95,97,100,102,104,107,109,112,114,117,119,122,125,127,130,133,136,139,141,144,147,150,153,156,160,163,166,169,172,176,179,182,186,189,192,196,199,203,206,210,214,217,221,225,228,232,236,240,244,248,252,255};
const unsigned char tabI[109] PROGMEM = {0,3,5,8,10,12,15,17,19,22,24,26,29,31,34,36,38,41,43,45,48,50,52,55,57,59,62,64,67,69,71,74,76,78,81,83,85,88,90,93,95,97,100,102,104,107,109,111,114,116,118,121,123,126,128,130,133,135,137,140,142,144,147,149,152,154,156,159,161,163,166,168,170,173,175,177,180,182,185,187,189,192,194,196,199,201,203,206,208,211,213,215,218,220,222,225,227,229,232,234,236,239,241,244,246,248,251,253,255};
//const unsigned char tabU[147] PROGMEM = {0,2,4,6,7,9,11,13,14,16,18,20,21,23,25,27,28,30,32,34,35,37,39,41,42,44,46,47,49,51,53,54,56,58,60,61,63,65,67,68,70,72,74,75,77,79,81,82,84,86,87,89,91,93,94,96,98,100,101,103,105,107,108,110,112,114,115,117,119,121,122,124,126,128,129,131,133,134,136,138,140,141,143,145,147,148,150,152,154,155,157,159,161,162,164,166,168,169,171,173,174,176,178,180,181,183,185,187,188,190,192,194,195,197,199,201,202,204,206,208,209,211,213,215,216,218,220,221,223,225,227,228,230,232,234,235,237,239,241,242,244,246,248,249,251,253,255};

const unsigned char tabSpeed[245] PROGMEM = {247,227,209,194,181,170,160,151,143,136,130,124,119,114,109,105,101,97,94,91,88,85,83,80,78,76,74,72,70,68,67,65,64,62,61,60,58,57,56,55,54,53,52,51,50,49,48,47,47,46,45,44,44,43,42,42,41,40,40,39,39,38,38,37,37,36,36,35,35,34,34,34,33,33,32,32,32,31,31,31,30,30,30,29,29,29,28,28,28,28,27,27,27,27,26,26,26,26,25,25,25,25,25,24,24,24,24,24,23,23,23,23,23,22,22,22,22,22,22,21,21,21,21,21,21,20,20,20,20,20,20,20,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,18,17,17,17,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,0};



/*----------------------------------
	Functions:
----------------------------------*/

//PII regulation !!!ZKONTROLOVAT!!!
inline unsigned char regulator()
{	
	unsigned int realCurrent;//0 - 65 535 (0 - 12 800A), 255 = 50A
	int delta;//-32768 - +32767
	
	if (wantedAcceleration == 0)//no acceleration wanted -> wanted speed = 0
	{
		sum2=0;
		sum1=0;
		return 0;		
	}	
	
	if (wantedSpeed < 25 || actualCurrent < 25)//if wanted speed < 10% or current < 5A
	{
		sum2 = wantedAcceleration * 64;
		sum1=0;
		return wantedAcceleration;
	}
	
	realCurrent = (actualCurrent*(unsigned int)256)/wantedSpeed;
	
	if (realCurrent > 256) // real current is 50A - no more accelerate
	{
		return(sum2 >> 6);
	} 
	else if(realCurrent > 512) // real current is 100A !!! -> slow down
	{
		wantedAcceleration = 0;
	}
	
	delta = (int)wantedAcceleration - realCurrent;
	
	//acceleration - positive delta
	if (delta >= 0)
	{	
		if ( sum1 + delta > 255) sum1 = 255;
		else sum1 += delta;
		
		if (sum2 + sum1 + delta < 16383) sum2 += (sum1 + delta); //16383 = max -> 255 to output
		else sum2 = 16383;
	} 
	// slow down - negative delta
	else
	{		
		if ( sum1 + delta < -255) sum1 = -255;
		else sum1 += delta;
		
		if (sum2 > -(sum1 + delta)) sum2 += (sum1 + delta); //0 = min - 0 to output
		else sum2 = 0;	
	}
	
	return(sum2 >> 6);	
}
//function for analog measure, return measured value
//255=4.7V; 0 = 0V
inline unsigned char Measure(unsigned char input_pin, unsigned char min, unsigned char max)
{
	//REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
	ADMUX = 0x20 + input_pin;		
	
	//ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA = 0xC6;		//start conversion, divide clk 64
	
	
	while(readBit(ADCSRA, 6)); //wait until conversion is complete
	
	//unsigned char measured = 0;
	
	if (ADCH < min)
	{
		return 0;
	} 
	else if (ADCH > max)
	{
		return (max-min);		
	}
	else
	{
		return (ADCH - min);
	}

	//return measured;	
}

// convert unsigned int to char array 
void toCharArray(char *array, unsigned int number)
{
	char i = 0;
	if (number==0)
	{
		array[i] = '0';
		i++;
	}
	while (number>0 && i<8)
	{
		array[i] = (number % 10) + '0';
		number/=10;
		i++;
	}
	array[i]='\0';
	
	for (char j=0;j<(i/2);j++)
	{
		char pom = array[j];
		array[j] = array[i-j-1];
		array[i-j-1] = pom;
	}
}

//show on display which value is selected
inline void displayShowMode(char mode)
{
			displayPausedCounter = 50;
			displayPaused = 1;

			switch(mode)
			{
				case 1:
					displayWriteDataArray("Tot.dist");
					break;
				case 2:
					displayWriteDataArray("Tot.cons");
					break;
				case 3:
					displayWriteDataArray("Distance");
					break;
				case 4:
					displayWriteDataArray("Consumed");
					break;
				case 5:
					displayWriteDataArray("Rest cap");
					break;
				case 6:
					displayWriteDataArray("Accu.   ");
					break;
				case 7:
					displayWriteDataArray("Current ");
					break;
				case 8:
					displayWriteDataArray("Speed   ");
					break;
					
			}
}

//check if button pressed, change display line mode or clear distance and consumed capacity
inline void checkButton()
{

	if(lastButtonState != 0x03-(PINC & 0x03))
	{
		lastButtonState = 0x03-(PINC & 0x03);
		
		switch(lastButtonState)//3 = both buttons pressed, 1,2 - button 1,2 pressed 
		{
			case 1:	//button 1 pressed - lineMode + 1
				do 
				{
					if (LineMode % 16 < 8)
					{
					LineMode++;
					}
					else
					{
					LineMode -= 7;			
					}
				} while (LineMode % 16 == LineMode / 16);
				
				//show line mode			
				displaySetAddressDDRAM(0x00);
				displayShowMode(LineMode % 16);	
			break;
		
			case 2: //button 2 pressed - lineMode + 16
				do 
				{
					if (LineMode / 16 < 8)
					{
					LineMode += 16;
					}
					else
					{
					LineMode -= 112;				
					}
				} while (LineMode % 16 == LineMode /16);
				
				//show line mode
				displaySetAddressDDRAM(0x40);
				displayShowMode(LineMode / 16);
			break;
		
			case 3://booth button pressed - reset distance and consumed capacity
				distance=0;
				totalConsumedCapacity += (consumedCapacity/922);
				consumedCapacity=0;
			break;
		
		}
	}	
	
}

//function for refresh display 
inline void displayRedraw()
{
	char array[8];
	unsigned char xlineMode = 0;
	
	if(displayPaused == 0){
	for (int line=0;line<2;line++) //for booth lines
	{
		if (line)// line 2
		{
			displaySetAddressDDRAM(0x40);
			xlineMode = (LineMode / 16);
		} 
		else //line 1
		{
			displaySetAddressDDRAM(0x00);			
			xlineMode = (LineMode % 16);
		}
			
		switch(xlineMode)
		{
			case 1://1 total distance
				toCharArray(&array,(totalDistance*395)>>20); //distance*0.000377 = dist in kilometers	
				//toCharArray(&array,(totalDistance*0.000377));
				displayWriteDataArray(array);
				displayWriteDataArray(" km");		
			break;
		
			case 2://2 total consumed capacity
				toCharArray(&array,(totalConsumedCapacity)/1000 + consumedCapacity/921600);//show in Ah		
				displayWriteDataArray(array);
				displayWriteDataArray(" Ah");
			break;
	
			case 3://3 distance
				toCharArray(&array,(distance*386)>>10); //distance*0.377 = dist in meters
				//toCharArray(&array,(distance*0.377));
				displayWriteDataArray(array);
				displayWriteDataArray(" m");
			break;
	
			case 4://4 consumed capacity (mAh)
				toCharArray(&array,consumedCapacity/922);	//consumed capacity/256/3600 = mAh
				displayWriteDataArray(array);
				displayWriteDataArray(" mAh");
			break;
	
			case 5://5 rest capacity (%)
				if (actualVoltage>=180)	displayWriteDataArray("100 %");
				
				else if (actualVoltage>=80)
				{
					toCharArray(&array,(actualVoltage-80));
					displayWriteDataArray(array);
					displayWriteDataArray(" %");
				} 
				
				else displayWriteDataArray("!  0 % !");									
				
			break;
	
			case 6://6 voltage
				toCharArray(&array,(10 + actualVoltage/25));	//10V + 1/25V 100 = 4V	
				displayWriteDataArray(array);
				displayWriteDataArray(" V");
			break;
	
			case 7://7 current
				toCharArray(&array,actualCurrent/5);		
				displayWriteDataArray(array);
				displayWriteDataArray(" A");
			break;
	
			case 8://8 speed
				toCharArray(&array,actualSpeed/4);		
				displayWriteDataArray(array);
				displayWriteData('.');
				toCharArray(&array,(actualSpeed % 4)*10/4);		
				displayWriteDataArray(array);
				displayWriteDataArray("km/h");
			break;
			
			default:
				displayWriteDataArray("Err     ");
			break;
		}
		displayWriteDataArray("     ");
	}
	}	
}



/*---------------------------------------------
Interrupt routines
----------------------------------------------*/

// interrupt timer 1 - compare match A - every 20ms
ISR(TIMER1_COMPA_vect)			//auto reload OCR1A - CTC mode
{
	setBit(OUTPUT,SW);			//start PWM pulse for controller
	OCR1BL = 128 + (wantedSpeed >> 1);//sets PWM impulse width 1-2ms (0-127)
	
	/*	CURRENT
		0A - min 0.6V = 33
		50A - max 2.6V = 141 */
	actualCurrent = pgm_read_byte(&tabI[Measure(SI,33,141)]);
	
	/*	ACELERATION
		min 0.86V = 51
		max 4.5V = 244 */
	wantedAcceleration = pgm_read_byte(&tabA[Measure(SA,51,244)]);
	
	//display pause timer decrement
	if(displayPaused == 1)
		{
			if (displayPausedCounter <= 0)
			{
				displayPaused = 0;
			}
			else
			{
				displayPausedCounter--;
			}
		}
}

// interrupt timer 1 - compare match B - after 1-2ms
ISR(TIMER1_COMPB_vect)
{
	clearBit(OUTPUT,SW);		//end of PWM impulse	
	
	wantedSpeed = regulator();
	
	consumedCapacity += actualCurrent+1;	//increment of consumed capacity
		
	if (wantedAcceleration > 0)
	{
		setBit(OUTPUT,SF); //fan on 
	} 
	else
	{
		clearBit(OUTPUT,SF); //fan off
	}
	
	if (actualCurrent < 10)//voltage measure if current is low (I<2A), fan is off
	{
		/*	VOLTAGE
			11.4V = 38
			12.8V - min 1.4V = 76
			16.8V - max 3.4V = 184 */
		actualVoltage = Measure(SU,0,180);
	}
	
}

// interrupt timer 2 - compare match, auto reload - every 2ms
ISR(TIMER2_COMP_vect) //cycle time counter, count up to 255
{
	if (CycleTime<255)
	{
		CycleTime++;
	}
	else
	{
		lastCyclePeriod=255;
		actualSpeed=0;
	}
}

// external interrupt 0 - cycle time measure, distance increment
ISR(INT0_vect)
{
	lastCyclePeriod=CycleTime;
	CycleTime=0;
	
	distance++;
	totalDistance++;
	
	//actual speed 
	if (lastCyclePeriod>10)
	{
		actualSpeed=pgm_read_byte(&tabSpeed[lastCyclePeriod-11]);
	} 
	else
	{
		actualSpeed=255;
	}
	
}

// external interrupt 1 - OFF signal
ISR(INT1_vect)
{
	cli();//disable global interrupt
	
	totalConsumedCapacity += (consumedCapacity/922);
	
	//save data to EEPROM
	eeprom_update_dword(15,totalDistance);
	eeprom_update_dword(25,totalConsumedCapacity);
	eeprom_update_byte(35,LineMode);
	
	
	displaySetAddressDDRAM(0x00);
	displayWriteDataArray("  GOOD  ");
	displaySetAddressDDRAM(0x40);
	displayWriteDataArray("  BYE   ");
	
	while(1){};
	//wait to power down
} 


int main(void)
{
	
	/*-----------------------
	Default PORT settings
	------------------------*/
	//portB is configured by display.h
	
	//portD: 0,1,5,6,7 = output, 2,3 = input (interrupts), 4 - unused
	DDRD = 0xE3;
	PORTD = 0x1C; // enable pull-up for unused pin and for interrupt pins
	
	//port C is only input port
	DDRC = 0x00;
	PORTC = 0xCB; //pull-up for btn 1-2, reset and unused pin,
		
	clearBit(OUTPUT,SF); //FAN is OFF
	
	/*-----------------------
	Restore data from eeprom
	------------------------*/	
	totalDistance = eeprom_read_dword(15);
	totalConsumedCapacity = eeprom_read_dword(25);
	LineMode = eeprom_read_byte(35);
	
	
	/*-------------------------------------------------------------
	TIMER1 configuration 
	generate of "servo" control PWM (1-2ms impulse, 20ms period)
	--------------------------------------------------------------*/
	//COM1A1 COM1A0 COM1B1 COM1B0 FOC1A FOC1B WGM11 WGM10
	TCCR1A = 0x00; //Normal port operation, OC1A/OC1B disconnected, normal mode
	
	//ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
	TCCR1B = 0x0B; //CTC mode, prescaler = 64 (250 clk = 2ms, 2500 = 20ms)
	
	//compare register A - period 20ms
	OCR1A = 0x09C4;
	
	// compare register B - impulse 1-2ms (OCR1AL=0x7F-0xFF)
	OCR1B = 0x007F;//1ms
	
	/*-------------------------------------------------------------
	TIMER2 configuration 
	generate time for speed measuring - period 2ms
	-------------------------------------------------------------*/
	
	//compare register - 2ms period
	OCR2 = 250;
	
	//FOC2 WGM20 COM21 COM20 WGM21 CS22 CS21 CS20
	TCCR2 = 0x0C;//TCT mode, OC2 disconnected, prescaler = 64	
	
	/*-------------------------------------------------------------
	external interrupt configuration 
	0 - measure cycle time
	1 - OFF signal
	-------------------------------------------------------------*/
	
	//SE SM2 SM1 SM0 ISC11 ISC10 ISC01 ISC00 
	setBit(MCUCR,1);// both interrupts on falling edge
	setBit(MCUCR,3);
	
	//INT1 INT0 – – – – IVSEL IVCE
	setBit(GICR,7); // both external interrupts enabled
	setBit(GICR,6);		
		
	//OCIE2 TOIE2 TICIE1 OCIE1A OCIE1B TOIE1 – TOIE0
	TIMSK = 0x98; //interrupts on compare match
	
	sei();//global interrupt enable	
	
	// display initialization

	displayPortsInit(); 
	_delay_ms(2);	
	displayFunctionSet(1,1,0);
	displayOnOffControl(1,0,0);
	displayClear();	
	
	displayEntryModeSet(1,0);
	displayCursorShift(0,1);	
	
	
	//show text for 2 seconds
	displayPausedCounter = 100;
	displayPaused = 1;
	
	displaySetAddressDDRAM(0x00);	
	displayWriteDataArray(" HELLO  ");
	displaySetAddressDDRAM(0x40);	
	displayWriteDataArray("ver. 1.6");
			
    while(1)
    {       
		_delay_ms(7);
		checkButton();
		displayRedraw();
		
		//pause redrawing for 0.5s
		if (displayPaused == 0)
		{
			displayPausedCounter = 25;
			displayPaused = 1;	
		}
				
    }
}