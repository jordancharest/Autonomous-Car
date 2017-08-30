/* Names: Jordan Charest, John Mata, Evan St. Angelo
Section: 1
Side: A
Description: Lab 4; Autonomous Car Lab; this program commands the car
to drive toward a desired heading. If an obstacle is encountered,
the car will turn one direction, if the obstacle is still visible, the
car will reverse to the previous location and turn the opposite direction
 */
#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void Interrupt_Init(void);
void XBR0_Init();
void PCA_Init (void);
void PCA_ISR ( void ) __interrupt 9;
void SMB_Init(void);
void ADC_Init(void);

void calibrate(void);
void ranger_ping(void);
void car_control(void);
void steer(int current_heading);
void wait(void);
void drive_wait(void);
void long_wait(void);
void pause(void);

unsigned int ranger(void);
unsigned int compass(void);
int choose_heading(void);
float choose_gain(void);
int keypad_input(void);
unsigned char read_AD_input(unsigned char pin_number);
//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
__sbit __at 0xB5 SS_calibrate;	// SS for calibration at P3.5
__sbit __at 0xB6 SS_R;	// SS for the ranger at P3.6
__sbit __at 0xB7 SS_C;	// SS for the compass at P3.7

unsigned int PW_steer_CENTER = 2805;
unsigned int PW_steer_MIN = 2185;			// PW drive extreme left
unsigned int PW_steer_MAX = 3275;			// PW drive extreme right
unsigned int PW_steer = 0;	
unsigned int PW_drive_CENTER = 2765;							
unsigned int PW_drive_MIN = 2028;			// PW of 1.1ms reverse full throttle
unsigned int PW_drive_MAX = 3400;			// PW of 1.9ms forward full throttle
unsigned int PW_drive = 2765;				// PW of 1.5ms to stand still

unsigned int nCounts;
unsigned int range = 0;
unsigned int heading = 0;
volatile unsigned char new_range = 0;
unsigned char r_count = 0;
volatile unsigned char new_heading = 0;
unsigned char h_count = 0;
unsigned char cycles = 0;
unsigned char Data[2];
int desired_heading = 1800;
float gain;
unsigned char input;
char drive_flag = 0;
int voltage;

//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    // initialize board
    Sys_Init();
    putchar(' '); //the quotes in this line may not format correctly
    Port_Init();
	Interrupt_Init();
    XBR0_Init();
    PCA_Init();
	SMB_Init();
	ADC_Init();

	printf("\r\nEmbedded Control Autonomous Car\n\n\n");
	wait();

	PW_steer = PW_steer_CENTER;
	PCA0CP0 = 0xFFFF - PW_steer;	// set initial pulse width
	PCA0CP2 = 0xFFFF - PW_drive_CENTER;	// set initial pulse width

	/* If using a new car, flip the calibration SS in order to run
	the steering and drive calibration; otherwise, the proper values are set
	*/
	if (!SS_calibrate) {
		calibrate();
	}

	printf("\r\nThe terminal must be used to input the gain and desired heading");
	gain = choose_gain();
	printf("\r\nThe gain has been chosen, please choose the heading\r\n");
	desired_heading = choose_heading();	// users inputs desired heading

    while(1)
	{
		if (new_range == 1)			// if new range flag
		{
			range = ranger();		// read the ranger
			ranger_ping();			// start a new ping
			new_range = 0;			// clear new ranger reading flag
			printf("\r\n%d,%d,%d", heading, range, PW_steer);
			car_control();			// control the car
		}
		if (new_heading == 1)		// if new heading flag
		{
			heading = compass();	// read the compass
			new_heading = 0;		// clear new heading flag
			/*cycles++;
			if (cycles > 20)		// perform every once in awhile
			{
				voltage = read_AD_input(1) * 58.8;	// read voltage and convert to millivolts
				printf("\r\nHeading is: %d", heading);
				printf("\r\nBattery Voltage (millivolts): %d", voltage);
				cycles = 0;
			}*/
		}
	}
}

//-----------------------------------------------------------------------------
// Port_Init - DONE
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Port_Init()
{	/*set output pin for CEX0 or CEX2 in push-pull mode; 
	Port 1.0 for CEX0 and P1.2 for CEX2*/
    P1MDOUT = 0x05; 
	//P1MDIN &= ~0x02;
	
	P3MDOUT &= ~0xE0;	// SS_R at P3.6, SS_C at P3.7, SS_calibrate at P3.5
	P3 |= 0xE0;			// high impedance on inputs
}

//-----------------------------------------------------------------------------
// Interrupt_Init - DONE
//-----------------------------------------------------------------------------
void Interrupt_Init(void)
{
	EA = 1;			// enable global interrupts
	EIE1 |= 0x08;	// enable PCA interrupts
}

//-----------------------------------------------------------------------------
// XBR0_Init - DONE
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init()
{
    XBR0 = 0x27;  //configure crossbar as necessary
}

//-----------------------------------------------------------------------------
// PCA_Init - DONE
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
	PCA0MD = 0x81;		// suspend PCA when processor is idle, enable CF interrupt, use SYSCLK/12
	PCA0CPM0 = 0xC2;	// Set CEX0 for 16-bit PWM, enable comparator, enable PWM
	PCA0CPM2 = 0xC2;	// Set CEX2 for 16-bit PWM, enable comparator, enable PWM
	PCA0CN |= 0x40;		// enable PCA
}

//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void ) __interrupt 9
{
	nCounts++;
	if (CF)
	{
		CF = 0;			// clear the interrupt service flag
		PCA0 = 28672;	// Preset PCA0 to achieve a 20ms period
		r_count++;
		h_count++;
	}

	if (r_count >= 5)	// if 100ms have passed
	{
		new_range = 1;	// set new range flag
		r_count = 0;	// reset 100 ms counter
	}

	if (h_count >= 2)	// if 40ms have passed
	{
		new_heading = 1;// set new heading flag
		h_count = 0;	// reset 40ms counter
	}

	PCA0CN &= 0xC0;		// handle other interrupt sources
}

//-----------------------------------------------------------------------------
// SMB_Init
//-----------------------------------------------------------------------------
void SMB_Init(void) 
{
	SMB0CR = 0x93;	//Set SCL to 100KHz
	ENSMB = 1;		//bit 6 of SMB0CN, enables the SMBus
}

//-----------------------------------------------------------------------------
// ADC_Init
//-----------------------------------------------------------------------------
void ADC_Init(void)
{
	REF0CN |= 0x03;						// set Vref to use internal reference voltage and enable inernal bias generator
	ADC1CN = 0x80;						// enable AD converter
	ADC1CF |= 0x01;						// set gain to 1
}

//-----------------------------------------------------------------------------
// CALIBRATION
//-----------------------------------------------------------------------------
void calibrate(void)
{
	printf("\r\nNew car needs to be calibrated. Follow the directions below");
	while(input !='z')
	{
		printf("\r\n\nCenter the steering. Press 'l' to turn left and 'r' to turn right. Press 'z' when finished");
		input = getchar();
		if (input == 'l')			// if user commands left turn
		{
			PW_steer -= 10;			// decrement PW by 10
		}

		if (input == 'r')			// if user commands right turn
		{
			PW_steer +=10;			// increment PW by 10
		}
	
		PCA0CP0 = 65536 - PW_steer;			// set pulsewidth
		printf("\r\nPulsewidth is: %u", PW_steer);
	}

	if (input == 'z') PW_steer_CENTER = PW_steer;	// define center PW for steering

	while (input != 'y')
	{
		printf("\r\n\nAdjust the steering until it is fully left by pressing 'l'");
		printf("\r\nPress 'y' when finished");
		input = getchar();
		if (input == 'l')			// if user commands left turn
		{
			PW_steer -= 10;			// decrement PW by 10
		}

		if (input == 'r')			// if user commands right turn
		{
			PW_steer +=10;			// increment PW by 10
		}
		PCA0CP0 = 65536 - PW_steer;		// set pulsewidth
		printf("\r\nPulsewidth is: %u", PW_steer);
	}

	if (input == 'y') PW_steer_MIN = PW_steer;	// define min PW for steering

	while (input != 'x')
	{
		printf("\r\n\nAdjust the steering until it is fully right py pressing 'r'");
		printf("\r\nPress 'x' when finished");
		input = getchar();
		if (input == 'l')			// if user commands left turn
		{
			PW_steer -= 10;			// decrement PW by 10
		}

		if (input == 'r')			// if user commands right turn
		{
			PW_steer +=10;			// increment PW by 10
		}
		PCA0CP0 = 65536 - PW_steer;		// set new PW
		printf("\r\nPulsewidth is: %u", PW_steer);
	}

	if (input == 'x') PW_steer_MAX = PW_steer;	// define max PW for steering

	PCA0CP2 = 0xFFFF - PW_drive;	// initialize PW for drive motor
}
//-----------------------------------------------------------------------------
// CHOOSE DESIRED HEADING
//-----------------------------------------------------------------------------
int choose_heading()
{
	int sum=0;
	char key, i;

	printf("Input a 2-digit degree value");
	for (i=0; i<2; i++)
	{
		key = getchar();		// input a key
		sum = (sum*10) + ((key - '0')*100);	// get the sum from 2 inputs
	}

	printf("\r\nYour desired heading is: %d", sum);

	return sum;
}

//-----------------------------------------------------------------------------
// CHOOSE DESIRED GAIN
//-----------------------------------------------------------------------------
float choose_gain(void)
{
	char digit1;
	float digit2;

	printf("\r\nMust use the terminal to choose the gain");
	printf("\r\nEnter a 2 digit gain value, in the form #.#");
	digit1 = getchar();	// get first digit
	digit1 -= 48;		// convert to decimal
	digit2 = getchar();	// get second digit
	digit2 -= 48;		// convert to decimal
	gain = digit1 + digit2/10;	// add together to get gain

	printf_fast_f("\r\n\nYour chosen gain is: %f", gain);
	
	return gain;
}
//-----------------------------------------------------------------------------
// READ KEYPAD - this code remains in case we want to add the keypad again
//-----------------------------------------------------------------------------
/*
int keypad_input(void)
{
	int sum;
	char key, i;

	sum = 0;

	lcd_print("Type a 2 digit degree value,\n then #");
	wait();

	for (i=0; i<3; i++)
	{
		while(((key=read_keypad()) == 0xFF) || (key == '*')) pause();
		if (key == '#')
		{
			while(read_keypad() == '#') pause();
			lcd_clear();
			lcd_print("Calculating...");
			wait();
			return sum;
		}
		else
		{	
			lcd_clear();
			lcd_print("\n%c", key);
			sum = (sum*10) + ((key - '0')*100);
			printf("\r\nSum is: %d", sum);
			while(read_keypad() == key) pause();
		}
	}
	return sum;	
}
*/
//-----------------------------------------------------------------------------
// READ THE RANGER
//-----------------------------------------------------------------------------
unsigned int ranger()
{
	unsigned int range = 0;
	unsigned char addr = 0xE0;			// address for the ranger

	i2c_read_data(addr, 2, Data, 2);	// read two bytes starting at reg 2
	return (((unsigned int)Data[0] << 8) | Data[1]);
}

//----------------------------------------------------------------------------
// RANGER PING
//-----------------------------------------------------------------------------
void ranger_ping(void)
{
	unsigned char addr = 0xE0;			// address for the ranger
	Data[0] = 0x51;						// return value in cm 
	i2c_write_data(addr, 0, Data, 1);	// write 1 byte starting at reg 0
}
	
//----------------------------------------------------------------------------
// READ THE COMPASS
//-----------------------------------------------------------------------------
unsigned int compass()
{
	unsigned char addr = 0xC0;			// address for the compass
	unsigned char Data_C[2];
	unsigned int heading;
	
	i2c_read_data(addr, 2, Data_C, 2);	// read 2 bytes starting at reg 2
	heading = (((unsigned int)Data_C[0] << 8) | Data_C[1]);	// combine bytes

	return heading;						// return heading value
}

//----------------------------------------------------------------------------
// WAIT
// ---------------------------------------------------------------------------
void wait(void)
{
    nCounts = 0;
    while (nCounts < 50);    // 50 counts -> 50 x 20ms = 1000ms
}

//----------------------------------------------------------------------------
// DRIVE WAIT
// ---------------------------------------------------------------------------
void drive_wait(void)
{
	char i;
	for (i=0; i<55; i++)
	{
		pause();		// drive for a little more than a second
	}
	ranger_ping();
	nCounts = 0;
    while (nCounts < 10);    // 15 counts -> 10 x 20ms = 200ms
}
//----------------------------------------------------------------------------
// PAUSE
// ---------------------------------------------------------------------------
void pause(void)
{
    nCounts = 0;
    while (nCounts < 1);// 1 count -> (65536-PCA_START) x 12/22118400 = 20ms
} 
//----------------------------------------------------------------------------
// CAR CONTROL
// ---------------------------------------------------------------------------
void car_control(void)
{
	if (range < 50 && drive_flag == 0)	// if object within 50cm
	{
		if (range < 15) 
			PCA0CP2 = 0xFFFF - PW_drive_CENTER;
		PCA0CP2 = 0xFFFF - 3200;// partial throttle forward
		PCA0CP0 = 0xFFFF - PW_steer_MAX;// turn hard right
		PW_steer = PW_steer_MAX;
		drive_wait();						// drive 1 second
		drive_flag = 1;
	}
	else if( range < 50 && drive_flag == 1)	// if obstacle still there
	{
		PCA0CP2 = 0xFFFF - PW_drive_MIN;	// full throttle reverse
		long_wait();					 	// for a little bit
		PCA0CP2 = 0xFFFF - PW_drive_MAX;// full throttle forward
		PCA0CP0 = 0xFFFF - PW_steer_MIN; // turn hard left
		PW_steer = PW_steer_MIN;
		wait();					 		// for one second
		drive_flag = 2;
	}
	else if(range < 50 && drive_flag == 2)	// if obstacle still there
		PCA0CP2 = 0xFFFF - PW_drive_CENTER;	// stop the car

	else if (range < 20)
		PCA0CP2 = 0xFFFF - PW_drive_CENTER;	// stop the car

	else
	{
		PCA0CP2 = 0xFFFF - PW_drive_MAX;	// full throttle forward
		drive_flag = 0;						// reset drive flag
		steer(heading);				// steer toward the desired heading
	}
}

//----------------------------------------------------------------------------
// STEER
// ---------------------------------------------------------------------------
void steer(int current_heading)
{
	int error = 0;	// error function

	error = desired_heading - current_heading;  // degrees off course
	if (error > 1800)			// flip error if greater than 1800
		error -= 3600;
	if (error < -1800)			// flip error if less than -1800
		error += 3600;

	PW_steer = error * gain + PW_steer_CENTER;	// change PW proportionally to error

	if(PW_steer < PW_steer_MIN)  	// check if less than pulsewidth minimum
    	PW_steer = PW_steer_MIN;    // set PW_steer to a minimum value

	if(PW_steer > PW_steer_MAX)  	// check if pulsewidth maximum exceeded
    	PW_steer = PW_steer_MAX;    // set PW to a maximum value
	
	PCA0CP0 = 0xFFFF - PW_steer;	// define new PW
}

//----------------------------------------------------------------------------
// LONG WAIT
// ---------------------------------------------------------------------------
void long_wait(void)
{
	char i;

	for(i=0; i<3; i++)	// wait 1 second, three times
	{
		wait();		
	}
}

//----------------------------------------------------------------------------
// READ AD INPUT
// ---------------------------------------------------------------------------
unsigned char read_AD_input(unsigned char pin_number)
{
	AMX1SL = pin_number;				// use P1.1 for input to ADC
	ADC1CN &= ~0x20;					// enable AD converter
	ADC1CN |= 0x10;						// begin conversion

	while ((ADC1CN & 0x20) == 0x00);  	// wait for conversion to complete

	return ADC1;
}
	