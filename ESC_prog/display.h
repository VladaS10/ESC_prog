#ifndef DISPLAY_H
#define DISPLAY_H

#include "bitops.h"

#define F_CPU 8000000UL
#include <util/delay.h>

/**
 * Ports definition.
 */
#define DISPLAY_PORT PORTB
#define DISPLAY_DDR DDRB
#define DISPLAY_PIN PIND

#define E_PORT PORTD
#define E_DDR DDRD
#define E_BIT 7

#define RW_PORT PORTD
#define RW_DDR DDRD
#define RW_BIT 6

#define RS_PORT PORTD
#define RS_DDR DDRD
#define RS_BIT 5

/**
 * Set / clear macros.
 */
#define SET_E() (setBit((E_PORT),(E_BIT)))
#define CLEAR_E() (clearBit((E_PORT),(E_BIT)))

#define SET_RW() (setBit((RW_PORT),(RW_BIT)))
#define CLEAR_RW() (clearBit((RW_PORT),(RW_BIT)))

#define SET_RS() (setBit((RS_PORT),(RS_BIT)))
#define CLEAR_RS() (clearBit((RS_PORT),(RS_BIT)))

/**
 * Generates 5us pulse at E_PORT and waits 35us for display to finish command (function lasts cca 40us). 
 */
#define DISPLAY_EXECUTE() SET_E(); _delay_us(5); CLEAR_E(); _delay_us(35);

/**
 * Generates 5us pulse at E_PORT and waits 1.59ms for display
 * to finish command (function lasts cca 1.64ms). 
 */
#define DISPLAY_EXECUTE2() SET_E();	_delay_us(5); CLEAR_E(); _delay_ms(2);	//_delay_ms(1.59);							

/**
 * Initializes display ports.
 */
void displayPortsInit(void){
	DISPLAY_PORT = 0;
	DISPLAY_DDR = 0xFF;
	setBit(E_DDR, E_BIT);
	setBit(RW_DDR, RW_BIT);
	setBit(RS_DDR, RS_BIT);
	CLEAR_E();
	CLEAR_RW();
	CLEAR_RS();
}

/**
 * Clear all display and returns the cursor to the home position (Address 0).
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   0    0    0    0    0    0    0    0    1
 */
void displayClear(void){
	CLEAR_RW();
	CLEAR_RS();
	DISPLAY_PORT = 1;
	DISPLAY_EXECUTE2();
}

/**
 * Returns the cursor to the home position (Address 0).  
 * Also returns the display being shifted to the original  
 * position DDRAM contents remain unchanged.
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   0    0    0    0    0    0    0    1    -
 */
void displayCursorAtHome(void){
	CLEAR_RW();
	CLEAR_RS();
	DISPLAY_PORT = 0b00000010;
	DISPLAY_EXECUTE2();
}

/**
 * Sets the cursor move direction and specifies or not to  
 * shift the display. These operations are performed  
 * during data write and read.
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   0    0    0    0    0    0    1    I/D  S  
 *
 * @param ID 0(address decrement) / 1(address increment)
 * @param S 1(with display cursor shift) / 0(without)
 */
void displayEntryModeSet(unsigned char ID, unsigned char S){
	CLEAR_RW();
	CLEAR_RS();
	DISPLAY_PORT = 0b00000100;
	if(ID) setBit(DISPLAY_PORT, 1);
	if(S) setBit(DISPLAY_PORT, 0);
	DISPLAY_EXECUTE();
}

/**
 * Sets the ON/OFF of all display (D) cursor ON/OFF (C), 
 * and blink of cursor position character (B).
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   0    0    0    0    0    1    D    C    B
 *
 * @param D sets the ON/OFF of all display
 * @param C cursor ON/OFF
 * @param B blink of cursor position character
 */
void displayOnOffControl(unsigned char D, unsigned char C, unsigned char B){
	CLEAR_RW();
	CLEAR_RS();
	DISPLAY_PORT = 0b00001000;
	if(D) setBit(DISPLAY_PORT, 2);
	if(C) setBit(DISPLAY_PORT, 1);
	if(B) setBit(DISPLAY_PORT, 0);
	DISPLAY_EXECUTE();
}

/**
 * Sets interface data length (DL), number of display  
 * lines(N) and character font (F).
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   0    0    0    0    1   SC    RL   -    -
 *
 * @param S/C=1: Display shift, S/C=0: Cursor  movement
 * @param R/L=1:Shift to the right, R/L=0:Shift to the left 
 */
void displayCursorShift(unsigned char SC, unsigned char RL){
	CLEAR_RW();
	CLEAR_RS();
	DISPLAY_PORT = 0b00010000;
	if(SC) setBit(DISPLAY_PORT, 3);
	if(RL) setBit(DISPLAY_PORT, 2);
	DISPLAY_EXECUTE();
}
  
/**
 * Sets interface data length (DL), number of display  
 * lines(N) and character font (F).
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   0    0    0    1    DL   N    F    -    -
 *
 * @param DL DL=1:8-bit, DL=0:4-bit
 * @param N N=1:2 lines, N=0:1 line  
 * @param F F=1:5x10 dots, F=0:5x7 dots  
 */
void displayFunctionSet(unsigned char DL, unsigned char N, unsigned char F){
	CLEAR_RW();
	CLEAR_RS();
	DISPLAY_PORT = 0b00100000;
	if(DL) setBit(DISPLAY_PORT, 4);
	if(N) setBit(DISPLAY_PORT, 3);
	if(F) setBit(DISPLAY_PORT, 2);
	DISPLAY_EXECUTE();
}

/**
 * Sets the CGRAM, data is sent and received after this setting.
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   0    0    1    Acg5 Acg4 Acg3 Acg2 Acg1 Acg0
 *
 * @param CGRAM address
 */
void displaySetAddressCGRAM(unsigned char address){
	CLEAR_RW();
	CLEAR_RS();
	DISPLAY_PORT = 0b01000000 | (address & 0b00111111);
	DISPLAY_EXECUTE();
}

/**
 * Sets the DDRAM, data is sent and received after this setting.
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   0    1    Add6 Add5 Add4 Add3 Add2 Add1 Add0
 *
 * @param DDRAM address
 */
void displaySetAddressDDRAM(unsigned char address){
	CLEAR_RW();
	CLEAR_RS();
	DISPLAY_PORT = 0b10000000 | (address & 0b01111111);
	DISPLAY_EXECUTE();
}	

/**
 * Reads Busy flag (BF) indicating internal operation is  
 * being performed and reads address counter contents.
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   1    BF   AC6  AC5  AC4  AC3  AC2  AC1  AC0
 *
 * @return 1b:BF + 7b:Address
 */
unsigned char displayBussyFlagAddressRead(void){
	SET_RW();
	CLEAR_RS();
	DISPLAY_DDR = 0x00;
	SET_E();
	_delay_us(5);
	unsigned char data = DISPLAY_PIN;
	CLEAR_E();
	DISPLAY_DDR = 0xFF;
	return data;
}

/**
 * Writes data into DDRAM or CGRAM.
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 0   1    DATA ...  ...  ...  ...  ...  ...  ...
 *
 * @param data character to write
 */
void displayWriteData(unsigned char data){
	DISPLAY_PORT = data;
	SET_RS();
	CLEAR_RW();
	DISPLAY_EXECUTE();
}

/**
 * Reads data into DDRAM or CGRAM.
 *
 * RS  R/W  DB7  DB6  DB5  DB4  DB3  DB2  DB1  DB0
 * 1   1    DATA ...  ...  ...  ...  ...  ...  ...
 *
 * @return data from DDRAM or CGRAM
 */
unsigned char displayReadData(void){
	SET_RW();
	SET_RS();
	DISPLAY_DDR = 0x00;
	SET_E();
	_delay_us(5);
	unsigned char data = DISPLAY_PIN;
	CLEAR_E();
	DISPLAY_DDR = 0xFF;
	return data;
}

/**
 * Writes char array to display.
 *
 * @param char array
 */
void displayWriteDataArray(char* data){
	SET_RS();
	CLEAR_RW();
	for(int i = 0; data[i] != '\0'; i++){
		DISPLAY_PORT = data[i];
		DISPLAY_EXECUTE();
	}
}

#endif