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
#include <Arduino.h>
#include <util/atomic.h>

#include "TinyDebugKnockBang.h"


/*==============================================================================
  Calculate a loop and nop count for use by DELAY_FINE.  Closest match less 
  than or equal to the target is returned (floor).
==============================================================================*/

static uint16_t MicrosecondsToLoopsAndNops( long us, long cc ) __attribute__ (( const, always_inline ));

static uint16_t MicrosecondsToLoopsAndNops( long us, long cc )
{
  long goal_cycles;
  long loops_raw;
  long loops_cooked;
  long loops_cycles;
  long leftover;
  long nops_cooked;
  
  goal_cycles   = us * F_CPU;
  
  loops_raw     = (goal_cycles - (1000000L * (0L + cc))) / (1000000L * 3L);
  
  loops_cooked  = loops_raw < 0 ? 0 : loops_raw;
  
  loops_cycles  = (loops_cooked * 3L) + 0L + cc;
  
  leftover      = (goal_cycles - (loops_cycles * 1000000L)) / 1000000L;
  
  nops_cooked   = leftover < 0 ? 0 : leftover;

  return( (loops_cooked << 2) | nops_cooked );
}


/*==============================================================================
  Reduce some tedious repetition
==============================================================================*/
void TinyDebugKnockBangClass_AssemblyMacros( void )
{
  // Macro definitions

  asm volatile 
  (
    ".macro DELAY_ROUGH Rx, LoopsOrNops"              "\n\t"

    ".if \\LoopsOrNops != 0"                          "\n\t"

    ".if \\LoopsOrNops < 128"                         "\n\t"

    "ldi   \\Rx, \\LoopsOrNops"                       "\n\t"
  "1:"                                                "\n\t"
    "dec   \\Rx"                                      "\n\t"
    "brne  1b"                                        "\n\t"

    ".else"                                           "\n\t"

    ".if \\LoopsOrNops >= 0x80 + 1"                   "\n\t"
    "nop"                                             "\n\t"
    ".endif"                                          "\n\t"

    ".if \\LoopsOrNops >= 0x80 + 2"                   "\n\t"
    "nop"                                             "\n\t"
    ".endif"                                          "\n\t"

    ".if \\LoopsOrNops >= 0x80 + 3"                   "\n\t"
    "nop"                                             "\n\t"
    ".endif"                                          "\n\t"

    ".endif"                                          "\n\t"

    ".endif"                                          "\n\t"

    ".endm"                                           "\n\t"
		: 
		: 
    :
  );

  asm volatile 
  (
    ".macro DELAY_FINE Rx, LoopsAndNops"              "\n\t"

    ".if ( \\LoopsAndNops >> 2 ) != 0"                "\n\t"
      "ldi   \\Rx, ( \\LoopsAndNops >> 2 )"           "\n\t"
    "1:"                                              "\n\t"
      "dec   \\Rx"                                    "\n\t"
      "brne  1b"                                      "\n\t"
    ".endif"                                          "\n\t"

    ".if ( \\LoopsAndNops & 3 ) >= 1"                 "\n\t"
      "nop"                                           "\n\t"
    ".endif"                                          "\n\t"

    ".if ( \\LoopsAndNops & 3 ) >= 2"                 "\n\t"
      "nop"                                           "\n\t"
    ".endif"                                          "\n\t"

    ".if ( \\LoopsAndNops & 3 ) >= 3"                 "\n\t"
      "nop"                                           "\n\t"
    ".endif"                                          "\n\t"

    ".endm"                                           "\n\t"
		: 
		: 
    :
  );
}


/*==============================================================================
  Knock on the receiver's door just once for a given transaction
==============================================================================*/
__attribute__((noinline))
bool TinyDebugKnockBangClass::lock( void )
{
  bool rv;

  if ( _locks == 0 )
  {
    rv = knockHello();
  }
  else
  {
    rv = true;
  }

  if ( rv )
  {
    ++_locks;
  }

  return( rv );
}


/*==============================================================================
  When the transaction is closed say goodbye to the receiver
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::unlock( void )
{
  if ( _locks > 0 )
  {
    if ( --_locks == 0 )
    {
      sendByte( 0 );

      // High output...
      KBS_PORT |= KBS_MASK;
      // ...input w/ pullup
      KBS_DDR &= ~ KBS_MASK;
    }
  }
}


/*==============================================================================
  Ask the receiver to write a single byte
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::write( uint8_t v )
{
  if ( lock() )
  {
    sendCommand( CMD_WRITE_BYTE );

    sendByte( 1 );  // sizeof(v)
    sendByte( v );

    unlock();
  }
}


/*==============================================================================
  Ask the receiver to write a block of data.  WARNING: size cannot be zero or
  255 and this method does not validate the parameter.
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::write( const uint8_t *buffer, uint8_t size )
{
  if ( lock() )
  {
    sendCommand( CMD_WRITE_BLOCK );

    sendByte( 1 );
    sendByte( size );

    sendByte( size );
    while ( size-- > 0 )
      sendByte( *buffer++ );

    unlock();
  }
}


/*==============================================================================
  Ask the receiver to print a text string read from Flash
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::print( const __FlashStringHelper * v )
{
  if ( lock() )
  {
    sendCommand( CMD_PRINT_STRING );

    uint8_t b;

    sendByte( 255 );

    do
    {
      b = pgm_read_byte( (const char*)(v) );
      v = (const __FlashStringHelper *)((const char*)(v) + 1);
      sendByte( b );
    }
    while ( b != 0 );

    unlock();
  }
}


/*==============================================================================
  Ask the receiver to print a text string read from SRAM
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::print( const char v[] )
{
  if ( lock() )
  {
    sendCommand( CMD_PRINT_STRING );

    sendByte( 255 );

    do
    {
      sendByte( *v );
    }
    while ( *v++ != 0 );

    unlock();
  }
}


/*==============================================================================
  Ask the receiver to print an 8 bit unsigned number
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::print( unsigned char v, uint8_t b )
{
  if ( lock() )
  {
    sendCommand( CMD_PRINT_UINT8 );

    sendByte( 2 );  // sizeof(v) + sizeof(b)
    sendByte( v );
    sendByte( b );

    unlock();
  }
}


/*==============================================================================
  Ask the receiver to print a floating-point number
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::print( double v, uint8_t d )
{
  union { uint32_t i; double f; uint8_t b[4]; } cracked;

  if ( sizeof(v) == 4 )
  {
    if ( lock() )
    {
      sendCommand( CMD_PRINT_DOUBLE );

      sendByte( 5 );  // sizeof(v) + sizeof(d)

      cracked.f = v;
      sendByte( cracked.b[3] );
      sendByte( cracked.b[2] );
      sendByte( cracked.b[1] );
      sendByte( cracked.b[0] );

      sendByte( d );

      unlock();
    }
  }
}


/*==============================================================================
  Ask the receiver to print a 16 bit signed or unsigned number
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::print16( uint8_t c, uint16_t v, uint8_t b )
{
  union { uint16_t i; uint8_t b[2]; } cracked;

  if ( lock() )
  {
    sendCommand( c );

    sendByte( 3 );  // sizeof(v) + sizeof(b)

    cracked.i = v;
    sendByte( cracked.b[1] );
    sendByte( cracked.b[0] );

    sendByte( b );

    unlock();
  }
}


/*==============================================================================
  Ask the receiver to print a 32 bit signed or unsigned number
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::print32( uint8_t c, uint32_t v, uint8_t b )
{
  union { uint32_t i; uint8_t b[4]; } cracked;

  if ( lock() )
  {
    sendCommand( c );

    sendByte( 5 );  // sizeof(v) + sizeof(b)

    cracked.i = v;
    sendByte( cracked.b[3] );
    sendByte( cracked.b[2] );
    sendByte( cracked.b[1] );
    sendByte( cracked.b[0] );

    sendByte( b );

    unlock();
  }
}


/*==============================================================================
  Ask the receiver to output CrLf
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::println( void )
{
  if ( lock() )
  {
    sendCommand( CMD_WRITE_CRLF );
    unlock();
  }
}


/*==============================================================================
  Knock on the receiver's door and wait for an optional hello
==============================================================================*/

static const int32_t OurClockInKHz = F_CPU / 1000L;  // processor speed in KHz
static const int32_t TheirClockInMHz = 16000000L / 1000000L;

// rmv static const int32_t OurClockInKHz = 1200000 / 1000L;  // processor speed in KHz

#define _cycles_to_cycles( cy )           (((cy * OurClockInKHz) + (TheirClockInMHz - 1)) / TheirClockInMHz)

#define _whatever_to_loops( w, m, c, l )  ((((w - c * 1000L) + ((m * 1000L) - 1 )) / (m * 1000L)) + l)

#define _cycles_to_loops( cy, m, c, l )   _whatever_to_loops( _cycles_to_cycles(cy), m, c, l )

#define _delay_to_loops( us, m, c, l )    (((((us * F_CPU) - c * 1000000L) + ((m * 1000000L) - 1 )) / (m * 1000000L)) + l)


/* rmv: c=7 (2+3+2) is too much.  At 1 MHz the Knock is too short.  At c=6 the loop count goes from 2 to 3 which apparently is still not enough to overcome the problem.  At c=3 the loop count goes from 3 to 4 which...
static const int32_t  _10us_s             = _delay_to_loops( 10, 3, 7, 1 );
static const int32_t  _10us_s             = _delay_to_loops( 10, 3, 6, 1 ); */
static const int32_t  _13us_s             = _delay_to_loops( 13, 3, 3, 1 );
static const uint16_t _13us               = _13us_s < 1 ? 1 : (_13us_s > 255 ? 0 : _13us_s);

static const int32_t  _2us_l_s            = _delay_to_loops(  2, 3, 3, 1 );
static const int32_t  _2us_n_s            = _delay_to_loops(  2, 1, 0, 0 );
static const uint16_t _2us                = _2us_l_s <= 1 ? ( _2us_n_s >= 0 ? _2us_n_s | 0x80 : 0 ) : ( _2us_l_s > 255 ? 0 : _2us_l_s);

static const int32_t  _6us_hst_s          = _delay_to_loops(  6, 5, 4, 1 );
static const uint16_t _6us_hst            =  _6us_hst_s < 1 ? 1 : ( _6us_hst_s > 255 ? 255 :  _6us_hst_s);

static const int32_t  _6us_hen_s          = _delay_to_loops(  6, 5, 7, 1 );
static const uint16_t _6us_hen            =  _6us_hen_s < 1 ? 1 : ( _6us_hen_s > 255 ? 255 :  _6us_hen_s);

static const int32_t  _24cy_l_s           = _cycles_to_loops(  24, 3, 10, 1 );
static const int32_t  _24cy_n_s           = _cycles_to_loops(  24, 1,  7, 0 );
static const uint16_t _24cy               = _24cy_l_s <= 1 ? ( _24cy_n_s >= 0 ? _24cy_n_s | 0x80 : 0 ) : ( _24cy_l_s > 255 ? 0 : _24cy_l_s);

// rmv static const int32_t  min_sp_l_s          = _cycles_to_loops(  21, 3, 5, 1 );
// rmv static const int32_t  min_sp_n_s          = _cycles_to_loops(  21, 1, 2, 0 );
// rmv static const uint16_t min_sp              = min_sp_l_s <= 1 ? ( min_sp_n_s >= 0 ? min_sp_n_s | 0x80 : 0 ) : ( min_sp_l_s > 255 ? 0 : min_sp_l_s);

static const int32_t  sp_to_bit0_l_s      = _cycles_to_loops(  20 /*16*/, 3, 3, 1 );
static const int32_t  sp_to_bit0_n_s      = _cycles_to_loops(  20 /*16*/, 1, 0, 0 );
static const uint16_t sp_to_bit0          = sp_to_bit0_l_s <= 1 ? ( sp_to_bit0_n_s >= 0 ? sp_to_bit0_n_s | 0x80 : 0 ) : ( sp_to_bit0_l_s > 255 ? 0 : sp_to_bit0_l_s);

// rmv static const int32_t  tog_to_bit_l_s      = _cycles_to_loops(  19 /*10*/ /*5*/, 3, 4, 1 );
// rmv static const int32_t  tog_to_bit_n_s      = _cycles_to_loops(  19 /*10*/ /*5*/, 1, 1, 0 );
// rmv static const uint16_t tog_to_bit          = tog_to_bit_l_s <= 1 ? ( tog_to_bit_n_s >= 0 ? tog_to_bit_n_s | 0x80 : 0 ) : ( tog_to_bit_l_s > 255 ? 0 : tog_to_bit_l_s);

// fix: Adjust this so c = overhead in sendByte
static const int32_t  dat_to_tog_l_s      = _cycles_to_loops(  20 /*9*/, 3, 4, 1 );
static const int32_t  dat_to_tog_n_s      = _cycles_to_loops(  20 /*9*/, 1, 1, 0 );
static const uint16_t dat_to_tog          = dat_to_tog_l_s <= 1 ? ( dat_to_tog_n_s >= 0 ? dat_to_tog_n_s | 0x80 : 0 ) : ( dat_to_tog_l_s > 255 ? 0 : dat_to_tog_l_s);


bool TinyDebugKnockBangClass::knockHello( void )
{
  uint8_t SaveSREG;
  uint8_t d;
  bool rv;

  asm volatile 
  (
    // Disable interrupts
    "in    %[SaveSREG], %[sreg]"                      "\n\t"
    "cli"                                             "\n\t"

    // Ensure MISO is floating high
    "sbis  %[pin], %[bit]"                            "\n\t"
    "rjmp  L%=bbailout1"                              "\n\t"

    // Pull MISO low (sinking output)
    "cbi   %[port], %[bit]"                           "\n\t"
    "sbi   %[ddr], %[bit]"                            "\n\t"

    // Delay for 13 us (a knock)
    "DELAY_ROUGH %[d], %[_13us]"                      "\n\t"

    // Then MISO back high (input pullup enabled)
    "cbi   %[ddr], %[bit]"                            "\n\t"
    "sbi   %[port], %[bit]"                           "\n\t"

    // Wait for MISO to float high (about two microseconds seems reliable)
    "DELAY_ROUGH %[d], %[_2us]"                       "\n\t"

    // For six microseconds check for Hello
  ".if %[_6us_hst] < 3"                               "\n\t"
    "sbis  %[pin], %[bit]"                            "\n\t"
    "rjmp  L%=bgothello"                              "\n\t"
    "sbis  %[pin], %[bit]"                            "\n\t"
    "rjmp  L%=bgothello"                              "\n\t"
    "sbic  %[pin], %[bit]"                            "\n\t"
    "rjmp  L%=bbailout2"                              "\n\t"
  ".else"                                             "\n\t"
    "ldi   %[d], %[_6us_hst]"                         "\n\t"  //  1  1
  "L%=bwaitforhello:"  "\n\t"
    "sbis  %[pin], %[bit]"                            "\n\t"  //  1  2 2 1
    "rjmp  L%=bgothello"                              "\n\t"  //  2  - - 2
    "dec   %[d]"                                      "\n\t"  //     1 1
    "brne  L%=bwaitforhello"                          "\n\t"  //     2 2
    "rjmp  L%=bbailout2"                              "\n\t"
  ".endif"                                            "\n\t"

  "L%=bgothello:"                                     "\n\t"
    // Putting bbailout2 here instead of with bbailout1 makes the Hello optional
  "L%=bbailout2:"                                     "\n\t"

    // For six microseconds check for Hello to finish
  ".if %[_6us_hen] < 3"                               "\n\t"
    "sbic  %[pin], %[bit]"                            "\n\t"
    "rjmp  L%=bready"                                 "\n\t"
    "sbic  %[pin], %[bit]"                            "\n\t"
    "rjmp  L%=bready"                                 "\n\t"
    "sbis  %[pin], %[bit]"                            "\n\t"
    "rjmp  L%=bbailout3"                              "\n\t"
  ".else"                                             "\n\t"
    "ldi   %[d], %[_6us_hen]"                         "\n\t"  //  1  1
  "L%=bwaitforready:"  "\n\t"
    "sbic  %[pin], %[bit]"                            "\n\t"  //  1  2 2 1
    "rjmp  L%=bready"                                 "\n\t"  //  2  - - 2
    "dec   %[d]"                                      "\n\t"  //     1 1
    "brne  L%=bwaitforready"                          "\n\t"  //     2 2
    "rjmp  L%=bbailout3"                              "\n\t"
  ".endif"                                            "\n\t"

  "L%=bready:"                                        "\n\t"
    // Putting bbailout3 here instead of with bbailout1 makes the Hello optional
  "L%=bbailout3:"                                     "\n\t"

    // Wait for the receiver to get everything in order then output the sample point pulse
    "DELAY_ROUGH %[d], %[_24cy]"                      "\n\t"

    // Make MISO an output (high)
    "sbi   %[ddr], %[bit]"                            "\n\t"

// rmv?...
    // Sample point
    "cbi   %[port], %[bit]"                           "\n\t"
    "DELAY_FINE %[d], %[dat_to_tog]"                  "\n\t"
    "sbi   %[port], %[bit]"                           "\n\t"
// ...rmv?

    // Receiver needs time to prepare for the first bit
    "DELAY_ROUGH %[d], %[sp_to_bit0]"                 "\n\t"

    // return true
    "ldi   %[rv], 0x01"                               "\n\t"
    "rjmp  L%=bfini"                                  "\n\t"

  "L%=bbailout1:"                                     "\n\t"
    // return false
    "ldi   %[rv], 0x00"                               "\n\t"

    // Restore interrupts
  "L%=bfini:"                                         "\n\t"
    "out   %[sreg], %[SaveSREG]"                      "\n\t"

		: 
      // Outputs
      [SaveSREG] "=&r" ( SaveSREG ),
      [d] "=&r" ( d ),
      [rv] "=r" ( rv )
		: 
      // Inputs
      [sreg] "I" ( _SFR_IO_ADDR( SREG ) ),
      [_13us] "M" ( _13us ),
      [_2us] "M" ( _2us ),
      [_6us_hst] "M" ( _6us_hst ),
      [_6us_hen] "M" ( _6us_hen ),
      [_24cy] "M" ( _24cy ),
// rmv       [min_sp] "M" ( min_sp ),
      [dat_to_tog] "M" ( MicrosecondsToLoopsAndNops( 2, 2 ) ),
      [sp_to_bit0] "M" ( sp_to_bit0 ),
      [ddr] "I" ( _SFR_IO_ADDR( KBS_DDR ) ),
      [port] "I" ( _SFR_IO_ADDR( KBS_PORT ) ),
      [pin] "I" ( _SFR_IO_ADDR( KBS_PIN ) ),
      [bit] "I" ( KBS_BIT )
    :
      // Clobbers
  );

  return( rv );
}


/*==============================================================================
  Send one raw byte to the receiver
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::sendByte( uint8_t v )
{
  uint8_t SaveSREG;
  uint8_t p;
  uint8_t i;
  uint8_t t;
  uint8_t x;
  uint8_t d;

  asm volatile
  (
    "in    %[SaveSREG], %[sreg]"                      "\n\t"
    "cli"                                             "\n\t"

    "in    %[p], %[port]"                             "\n\t"
    "mov   %[t], %[p]"                                "\n\t"

    "ldi   %[i], 8"                                   "\n\t"
    "ldi   %[x], %[mask]"                             "\n\t"
  "L%=bnextbit:"  "\n\t"
    "lsr   %[v]"                                      "\n\t"
    "brcc  L%=bdata0"                                 "\n\t"
  "L%=bdata1:"  "\n\t"
//  "mov   %[t], %[p]"                                "\n\t"  // rmv
    "ori   %[t], %[mask]"                             "\n\t"
    "rjmp  L%=bdataX"                                 "\n\t"
  "L%=bdata0:"  "\n\t"
//  "mov   %[t], %[p]"                                "\n\t"  // rmv
    "andi  %[t], ~ %[mask]"                           "\n\t"
    "nop"                                             "\n\t"
  "L%=bdataX:"  "\n\t"
    "eor   %[p], %[x]"                                "\n\t"

    "out   %[port], %[p]"                             "\n\t"
    "DELAY_FINE %[d], %[tog_to_bit]"                  "\n\t"
    "out   %[port], %[t]"                             "\n\t"
    "DELAY_FINE %[d], %[dat_to_tog]"                  "\n\t"

    "mov   %[p], %[t]"                                "\n\t"
    "subi  %[i], 1"                                   "\n\t"
    "brne  L%=bnextbit"                               "\n\t"

    "out   %[sreg], %[SaveSREG]"                      "\n\t"

/* 
  Note: There is enough overhead that this delay is not necessary up to 16 MHz.
  At 18 MHz it may become necessary.  The tricky part is that 
  MicrosecondsToLoopsAndNops takes the number of cycles as the constant.  The
  number of cycles may vary because of the other two delays.  Ideally, a 
  variation of MicrosecondsToLoopsAndNops would be added that takes the time 
  (in microseconds) as the constant.

    "DELAY_FINE %[d], %[inter_byte]"                  "\n\t"
*/
    : 
      // Outputs
      [SaveSREG] "=&r" ( SaveSREG ),
      [p] "=&r" ( p ),
      [i] "=&r" ( i ),
      [t] "=&r" ( t ),
      [x] "=&r" ( x ),
      [d] "=&r" ( d )
    : 
      // Inputs
      [v] "r" ( v ),
      [sreg] "I" ( _SFR_IO_ADDR( SREG ) ),
      [port] "I" ( _SFR_IO_ADDR( KBS_PORT ) ),
      [mask] "M" ( KBS_MASK ),
      [tog_to_bit] "M" ( MicrosecondsToLoopsAndNops( 1, 1 ) ),
      [dat_to_tog] "M" ( MicrosecondsToLoopsAndNops( 2, 11 ) ),
      [inter_byte] "M" ( MicrosecondsToLoopsAndNops( 2, 36 ) /*MicrosecondsToLoopsAndNops( 3, 45 )*/ )
    :
      // Clobbers
  );
}


/*==============================================================================
  Send a command byte to the receiver
==============================================================================*/
__attribute__((noinline))
void TinyDebugKnockBangClass::sendCommand( uint8_t c )
{
  sendByte( 1 );
  sendByte( c );
}


TinyDebugKnockBangClass Debug;
uint8_t TinyDebugKnockBangClass::_locks;

