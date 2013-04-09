#ifndef BITOPS_H
#define BITOPS_H

/**
 * Reads bit at given position in given byte.
 */ 
#define readBit(byte, position) (((byte)&(1 <<(position)))>>(position))

/**
 * Sets bit at given position in given byte.
 */ 
#define setBit(byte, position) ((byte)=((byte)|(1<<(position))))

/**
 * Clears bit at given position in given byte.
 */ 
#define clearBit(byte, position) ((byte)=((byte)&~(1 << (position))))

/**
 * Toggles bit at given position in given byte.
 */ 
#define toggleBit(byte, position) ((byte)=((byte)^(1<<(position))))

#endif