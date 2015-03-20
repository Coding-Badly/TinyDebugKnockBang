/*==============================================================================

  TinyDebugKnockBang - Serial like debugging for processors with an 
  uncalibrated / inaccurate clock.

  Copyright 2012 Rowdy Dog Software.

  This file is part of Arduino-Tiny.

  Arduino-Tiny is free software: you can redistribute it and/or modify it 
  under the terms of the GNU Lesser General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  Arduino-Tiny is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License 
  along with Arduino-Tiny.  If not, see <http://www.gnu.org/licenses/>.

==============================================================================*/
#ifndef TinyDebugKnockBang_h
#define TinyDebugKnockBang_h

#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif


/*=============================================================================
  Define MISO macros for the target (fix: move to a pin definitions header 
  file)
=============================================================================*/

#define MISO_PASTE(lft,rgt)       lft ## rgt
#define MISO_MAKE_REG(lft,rgt)    MISO_PASTE(lft,rgt)

#if defined( __AVR_ATtiny13__ ) || defined( __AVR_ATtiny25__ ) || defined( __AVR_ATtiny45__ ) || defined( __AVR_ATtiny85__ )

  #define MISO_DDR    DDRB
  #define MISO_PORT   PORTB
  #define MISO_PIN    PINB
  #define MISO_BIT    1

  #define KNOCKBANG_SENDER_AVAILABLE 1

  #if TC_VERSION < 200
    typedef fstr_t __FlashStringHelper;
  #endif

#elif defined( __AVR_ATtiny24__ ) || defined( __AVR_ATtiny44__ ) || defined( __AVR_ATtiny84__ )

  #define MISO_DDR    DDRA
  #define MISO_PORT   PORTA
  #define MISO_PIN    PINA
  #define MISO_BIT    5

  #define KNOCKBANG_SENDER_AVAILABLE 1

  #if TC_VERSION < 200
    typedef fstr_t __FlashStringHelper;
  #endif

#elif defined( __AVR_ATtiny2313__ ) || defined( __AVR_ATtiny4313__ )

  #define MISO_DDR    DDRB
  #define MISO_PORT   PORTB
  #define MISO_PIN    PINB
  #define MISO_BIT    6

  #define KNOCKBANG_SENDER_AVAILABLE 1

  #if TC_VERSION < 200
    typedef fstr_t __FlashStringHelper;
  #endif

#elif defined( __AVR_ATmega328P__ ) || defined( __AVR_ATmega328__ ) || defined( __AVR_ATmega168__ )

  #define MISO_DDR    DDRB
  #define MISO_PORT   PORTB
  #define MISO_PIN    PINB
  #define MISO_BIT    4

  #define KNOCKBANG_SENDER_AVAILABLE 1

#elif defined( __AVR_ATtiny261__ ) || defined( __AVR_ATtiny461__ ) || defined( __AVR_ATtiny861__ )

  #define MISO_DDR    DDRB
  #define MISO_PORT   PORTB
  #define MISO_PIN    PINB
  #define MISO_BIT    1

  #define KNOCKBANG_SENDER_AVAILABLE 1

  #if TC_VERSION < 200
    typedef fstr_t __FlashStringHelper;
  #endif

#else
  #warning Missing MISO_* definitions.  KnockBang send functions are not available.
  #define KNOCKBANG_SENDER_AVAILABLE 0
#endif


/*=============================================================================
  So the user can potentially override the pin used for output
=============================================================================*/

#ifndef KBS_DDR
#define KBS_DDR   MISO_DDR
#endif

#ifndef KBS_PORT
#define KBS_PORT  MISO_PORT
#endif

#ifndef KBS_PIN
#define KBS_PIN   MISO_PIN
#endif

#ifndef KBS_BIT
#define KBS_BIT   MISO_BIT
#endif

#define KBS_MASK  (1 << KBS_BIT)


/*=============================================================================
  TinyDebugKnockBangClass and the Debug singleton
=============================================================================*/

#if KNOCKBANG_SENDER_AVAILABLE

class TinyDebugKnockBangClass
{
public:

  static const uint8_t CMD_WRITE_CRLF    =  1;
  static const uint8_t CMD_WRITE_BYTE    =  2;
  static const uint8_t CMD_WRITE_BLOCK   =  3;

  static const uint8_t CMD_PRINT_STRING  =  4;
  static const uint8_t CMD_PRINT_UINT8   =  5;
  static const uint8_t CMD_PRINT_SINT16  =  6;
  static const uint8_t CMD_PRINT_UINT16  =  7;
  static const uint8_t CMD_PRINT_SINT32  =  8;
  static const uint8_t CMD_PRINT_UINT32  =  9;
  static const uint8_t CMD_PRINT_DOUBLE  = 10;

public:

  static void begin( unsigned long )
  {
    // Configure MISO as an input with the internal pullup enabled
    KBS_DDR &= ~ KBS_MASK;
    KBS_PORT |= KBS_MASK;
  }

  static void end()
  {
    // Ensure MISO is an input with the internal pullup disabled
    KBS_PORT &= ~ KBS_MASK;
    KBS_DDR &= ~ KBS_MASK;
  }

  static bool lock( void );
  static void unlock( void );

  // Note: Arduino 1.0 returns a size_t for all these...

  static void write( uint8_t v );
  static void write( const char *str ) { print( str ); }
  static void write( const uint8_t *buffer, uint8_t size );

  static void print( const __FlashStringHelper * v );
//static void print( const String & );
  static void print( const char v[] );
  static void print( char v ) { write( v ); }
  static void print( unsigned char v, uint8_t b = DEC );
  static void print( int v, uint8_t b = DEC ) { print16( CMD_PRINT_SINT16, (uint16_t)(v), b ); }
  static void print( unsigned int v, uint8_t b = DEC ) { print16( CMD_PRINT_UINT16, (uint16_t)(v), b ); }
  static void print( long v, uint8_t b = DEC ) { print32( CMD_PRINT_SINT32, (uint32_t)(v), b ); }
  static void print( unsigned long v, uint8_t b = DEC ) { print32( CMD_PRINT_UINT32, (uint32_t)(v), b ); }
  static void print( double v, uint8_t d = 2 );
//static void print( const Printable& );

  static void println( const __FlashStringHelper * v ) { print(v); println(); }
//static void println( const String & );
  static void println( const char v[] ) { print(v); println(); }
  static void println( char v ) { print(v); println(); }
  static void println( unsigned char v, uint8_t b = DEC ) { print(v,b); println(); }
  static void println( int v, uint8_t b = DEC ) { print(v,b); println(); }
  static void println( unsigned int v, uint8_t b = DEC ) { print(v,b); println(); }
  static void println( long v, uint8_t b = DEC ) { print(v,b); println(); }
  static void println( unsigned long v, uint8_t b = DEC ) { print(v,b); println(); }
  static void println( double v, uint8_t d = 2 ) { print(v,d); println(); }
//static void println( const Printable& );

  static void println( void );

  static void flush( void ) { }

private:

  static uint8_t _locks;

  static bool knockHello( void );
  static void sendByte( uint8_t v );
  static void sendCommand( uint8_t c );

  static void print16( uint8_t c, uint16_t v, uint8_t b );
  static void print32( uint8_t c, uint32_t v, uint8_t b );

  friend void loop( void );  // rmv: solely for debugging
};

extern TinyDebugKnockBangClass Debug;

#endif


#endif
