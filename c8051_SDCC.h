//-----------------------------------------------------------------------------
// This file is for use in Embedded Control when using the SDCC compiler
//
// Directions:
//
// This file should be saved to the following directory on your laptop:
// C:\Program Files\SDCC\include\mcs51
// Save as c8051_SDCC.h
//
// In your program, you need to include this header file as #include <c8051_SDCC.h>
// and in the main() program, call the function Sys_Init();
//
// Another approach is to put this file in the working directory and call it as
// #include "c8051_SDCC.h" and in the main() program, call the function Sys_Init();
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <c8051f020.h>                 // Special Function Register (SFR) declarations

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSCLK       22118400          // SYSCLK frequency in Hz
#define BAUDRATE     38400             // Baud rate of UART in bps


//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SYSCLK_Init()
//-----------------------------------------------------------------------------
//
// This routine initializes the system clock to use an 22.1184MHz crystal
// as its clock source.
//
void SYSCLK_Init(void)
{
   int i;                              // delay counter

   OSCXCN = 0x67;                      // start external oscillator with
                                       // 22.1184MHz crystal

   for (i=0; i < 256; i++);            // wait for oscillator to start

   while (!(OSCXCN & 0x80));           // Wait for crystal osc. to settle

   OSCICN = 0x88;                      // select external oscillator as SYSCLK
                                       // source and enable missing clock
                                       // detector
}

//-----------------------------------------------------------------------------
// UART0_Init()
//-----------------------------------------------------------------------------
//
// Configure the UART0 using Timer1, for <baudrate> and 8-N-1.
//
void UART0_Init(void)
{
   SCON0  = 0x50;                      // SCON0: mode 1, 8-bit UART, enable RX
   TMOD   = 0x20;                      // TMOD: timer 1, mode 2, 8-bit reload
   TH1    = 0xFF&-(SYSCLK/BAUDRATE/16);     // set Timer1 reload value for baudrate
   TR1    = 1;                         // start Timer1
   CKCON |= 0x10;                      // Timer1 uses SYSCLK as time base
   PCON  |= 0x80;                      // SMOD00 = 1 (disable baud rate 
                                       // divide-by-two)
   TI0    = 1;                         // Indicate TX0 ready
   P0MDOUT |= 0x01;                    // Set TX0 to push/pull
}

//-----------------------------------------------------------------------------
// Sys_Init()
//-----------------------------------------------------------------------------
//
// Disable watchdog timer and call other Init functions.
//
void Sys_Init(void)
{
   WDTCN = 0xde;			// disable watchdog timer
   WDTCN = 0xad;

   SYSCLK_Init();			// initialize oscillator
   UART0_Init();			// initialize UART0

   XBR0 |= 0x04;
   XBR2 |= 0x40;                    	// Enable crossbar and weak pull-ups
}

//-------------------------------------------------------------------------------------------------
// putchar()
//-------------------------------------------------------------------------------------------------
void putchar(char c)
{
   while (!TI0); 
   TI0 = 0;
   SBUF0 = c;
}

//-------------------------------------------------------------------------------------------------
// getchar()
//-------------------------------------------------------------------------------------------------
char getchar(void)
{
   char c;
   while (!RI0);
   RI0 = 0;
   c = SBUF0;
   putchar(c);                          // echo to terminal
   return SBUF0;
}

//-------------------------------------------------------------------------------------------------
// getchar_nw()
//-------------------------------------------------------------------------------------------------
char getchar_nw(void)
{
   char c;
   if (!RI0) return 0xFF;
   else
   {
      RI0 = 0;
      c = SBUF0;
      putchar(c);                          // echo to terminal
      return SBUF0;
   }
}
