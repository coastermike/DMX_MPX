#include <p24FV32KA304.h>

void setup(void);
void sendDAC(int value);
void display(char character);
void threeDigits(char first, unsigned int number);
void twoDigits(char first, char second, unsigned int number);

_FDS(DSWDTEN_OFF & DSBOREN_OFF)	//Deep Sleep Watchdog disabled, Deep sleep Zero-Power BOR Disabled
_FICD(ICS_PGx2)					//ICD Pin on PGC2/PGD2
_FPOR(BOREN_BOR3 & I2C1SEL_PRI & BORV_V30 & MCLRE_ON) //Brown-out reset enabled in hardware, Default I2C1 pins, Brownout set to 3V, MCLR pin enabled
_FWDT(FWDTEN_OFF & WINDIS_OFF)	//Watchdog disabled
_FOSC(OSCIOFNC_OFF & POSCMOD_NONE) 	//CLKO disable, primary osc diabled
_FOSCSEL(FNOSC_FRCPLL & SOSCSRC_DIG)	//Fast RC oscillator
_FGS(GWRP_OFF & GSS0_OFF)			//Flash write protection disabled, no protection
_FBS(BWRP_OFF & BSS_OFF)			//boot protects disabled

//Define pins
//RC4 Status
#define LED_Status PORTCbits.RC4

//LED display
#define APOS PORTAbits.RA9
#define COL PORTAbits.RA8
#define A PORTBbits.RB5
#define B PORTBbits.RB3
#define C PORTBbits.RB7
#define D PORTBbits.RB2
#define E PORTBbits.RB1
#define F PORTBbits.RB6
#define G PORTBbits.RB4
#define DP PORTBbits.RB0

//Switches
#define menu PORTCbits.RC0	//SW2
#define up PORTCbits.RC1	//SW3
#define down PORTCbits.RC2	//SW4

//Define constant values
#define DAC_W_ADDRESS 0b11000000//Address + write bit for I2C DAC
#define SYNC 0b0000000000		//Results in -10V sync pulse
#define ZERO 0b0111111111		//DAC value for 0V output
#define FULL 0b1111111111		//DAC value for 10V output
#define MAX_CHANNEL 16				//Sets the max channel outputs
#define MAX_DMX 64

//Define global variables
unsigned int DACvalue = 0;		//10bit value to be sent to the DAC
unsigned int stateDAC = 0;		//state of DAC to keep track of I2C sending
unsigned int stateMPX = 0;		//state of MPX transmission
unsigned int channelOutCount = 1;//keeps track of output channel
unsigned int DMXchannelCount = 0;//keeps track of DMX in channel
unsigned char DMXdata[MAX_DMX+1];
unsigned int breakZeros = 0, zeroState = 0;
unsigned int dmxnew = 0, dmxcurrent = 0, dmxaddress = 1, dmxreceived = 0;
unsigned int idleHistory = 0;
unsigned char menuState = 0, upState = 0, downState = 0;
unsigned char menuTemp = 0, upTemp = 0, downTemp = 0;

//display variables
unsigned int digitCount = 1;
char digit1 = ' ', digit2 = ' ', digit3 = ' ', digit4 = ' ';
unsigned int mode = 0;

//T1 interrupt for status LED
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
	LED_Status = ~LED_Status;
	if(U1STAbits.RIDLE && (idleHistory > 2))
	{
		APOS = 0;
		idleHistory = 0;
	}
	else
	{
		idleHistory++;
	}
	_T1IF = 0;
}

//T2 interrupt for sending MPX data
//contains state machine to send data via MPX standard
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt (void)
{
	_T2IF = 0;
	if(stateMPX == 0)		//Going to 0.25ms pulses, sends first channel, and increases Ch Count
	{
		PR2 = 1000;
		sendDAC(DMXdata[channelOutCount]*2+512);
		channelOutCount++;
		stateMPX = 1;
	}
	else if (stateMPX == 1)	//0.25ms sync
	{
		sendDAC(SYNC);
		stateMPX = 2;
	}
	else if (stateMPX == 2)	//Sends next channel, then decides which sync pulse to go to depending on current Ch count
	{
		
		if(channelOutCount < (MAX_CHANNEL))
		{
			sendDAC(DMXdata[channelOutCount]*2+512);
			channelOutCount++;
			stateMPX = 1;
		}
		else	
		{
			sendDAC(DMXdata[channelOutCount]*2+512);
			stateMPX = 3;
			channelOutCount = 1;
		}	
	}
	else if (stateMPX == 3)	//After last channel is sent go to 5.5ms sync pulse
	{
		PR2 = 22000;
		sendDAC(SYNC);
		stateMPX = 0;
	}			
}
	
//I2C interrupt
//Contains state machine to go through the process of sending data for the DAC
void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt (void)
{
	_MI2C1IF = 0;
	if(stateDAC == 0)
	{
		stateDAC = 1;
		I2C1TRN = DAC_W_ADDRESS;
	}
	else if(stateDAC == 1)
	{
		stateDAC = 2;
		I2C1TRN = 0b00001111&(DACvalue>>6);
	}
	else if(stateDAC == 2)
	{
		stateDAC = 3;
		I2C1TRN = (DACvalue << 2);
	}
	else if (stateDAC == 3)
	{
		stateDAC = 4;
		I2C1CONbits.PEN = 1;
	}
	else if (stateDAC == 4)
	{
		stateDAC = 0;
	}		
}

//Timer3 interrupt for break detection
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt (void)
{
	_T3IF = 0;
	if(_RC6 && zeroState == 0)
	{
		breakZeros = 0;
	}
	else if(_RC6 && zeroState == 1)
	{
		_T3IE=0;
		U1STAbits.OERR=0;
		_U1RXIE = 1;
		_U1ERIE = 1;
	}
	else	
	{
		breakZeros++;	
		if(breakZeros == 20)
		{
			zeroState = 1;
//		_T3IE=0;
//		U1STAbits.OERR=0;
//		_U1RXIE = 1;
//		_U1ERIE = 1;
		}
	}		
}

//U1RX interrupt
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt (void)
{
	_U1RXIF = 0;
	dmxreceived = U1RXREG;
	dmxcurrent++;
	if(dmxcurrent > dmxaddress)
	{
		DMXdata[DMXchannelCount] = dmxreceived;
		DMXchannelCount++;
		if(DMXchannelCount == MAX_DMX+1)
		{
			_U1RXIE = 0;
			_U1ERIE = 0;
			
			dmxnew = 1;
		}
		APOS = 1;
	}
}

//UART error interrupt
//NEED figure out which error occured. 
void __attribute__((interrupt, no_auto_psv)) _U1ErrInterrupt (void)
{
	_U1ERIF = 0;
	if(U1STAbits.OERR)
	{
		_U1RXIE = 0;
		_U1ERIE = 0;
		U1STAbits.OERR=0;
		dmxnew = 1;
	}
	if(U1STAbits.FERR)
	{
		APOS = 0;
	}
	if(U1STAbits.PERR)
	{
		APOS = 0;
	}
}

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt (void)
{
	_T4IF = 0;
	switch(digitCount)
	{
		case 1:
			PORTB |= (0x10FF);
			digitCount++;
			PORTB &= (0x1FFF);
			display(digit1);
			break;
		case 2:
			PORTB |= (0x20FF);
			digitCount++;
			PORTB &= (0x2FFF);
			display(digit2);
			break;
		case 3:
			PORTB |= (0x40FF);
			digitCount++;
			PORTB &= (0x4FFF);
			display(digit3);
			break;
		case 4:
			PORTB |= (0x80FF);
			digitCount = 1;
			PORTB &= (0x8FFF);
			display(digit4); 
			break;
		default:
			digitCount = 1;
			break;
	}	
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt (void)
{
	_CNIF = 0;
	if(!menu && !menuState && (menuTemp == 0))
	{
		menuTemp = 1;
	}
	else if(menu && !menuState && (menuTemp == 1))
	{
		menuTemp = 2;
	}
	if(!up && !upState && (upTemp == 0))
	{
		upTemp = 1;
	}
	else if(up && !upState && (upTemp == 1))
	{
		upTemp = 2;
	}
	if(!down && !menuState && (downTemp == 0))
	{
		downTemp = 1;
	}
	else if(down && !downState && (downTemp == 1))
	{
		downTemp = 2;
	}			
}

void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void)
{
	if((menuTemp == 2) && menu)
	{
		menuTemp++;
	}
	else if((menuTemp == 3) && menu)
	{
		menuTemp = 0;
		menuState = 1;
	}
	if((upTemp == 2) && up)
	{
		upTemp++;
	}
	else if((upTemp == 3) && up)
	{
		upTemp = 0;
		upState = 1;
	}
	if((downTemp == 2) && down)
	{
		downTemp++;
	}
	else if((downTemp == 3) && down)
	{
		downTemp = 0;
		downState = 1;
	}
	_T5IF = 0;
}
	
int main()
{
	setup();
	while(1)
	{	
		if(dmxnew)
		{
			dmxnew=0;
			DMXchannelCount = 0;
			breakZeros = 0;
			dmxcurrent = 0;
			zeroState = 0;
			_T3IE = 1;
		}
		switch(mode)
		{
			case 0: //presently Address
				if(menuState)
				{
					mode = 1;
					menuState = 0;
				}
				else if(upState)
				{
					if(dmxaddress < 512)
					{
						dmxaddress++;
					}
					else
					{
						dmxaddress = 0;
					}		
					upState = 0;
					threeDigits('A', dmxaddress);
				}
				else if(downState)
				{
					if(dmxaddress > 0)
					{
						dmxaddress--;
					}
					else
					{
						dmxaddress = 512;
					}		
					downState = 0;
					threeDigits('A', dmxaddress);
				}
				break;
			case 1: //presently # of channel set
				if(menuState)
				{
					mode = 0;
					menuState = 0;
				}
				else if(upState)
				{
					if(dmxaddress < 64)
					{
						dmxaddress++;
					}
					else
					{
						dmxaddress = 0;
					}		
					upState = 0;
					twoDigits('C', 'H', dmxaddress);
				}
				else if(downState)
				{
					if(dmxaddress > 0)
					{
						dmxaddress--;
					}
					else
					{
						dmxaddress = 64;
					}		
					downState = 0;
					twoDigits('C', 'H', dmxaddress);
				}
				break;
		}		
	}	
}

//Starts the I2C state machine and loads the desired DAC value to be sent
void sendDAC(int value)
{
	DACvalue = value;
	if(stateDAC == 0)
	{
		I2C1CONbits.SEN = 1;
	}	
}	

void setup()
{
	unsigned int i;
	CLKDIVbits.RCDIV = 0b000;
	ANSA = 0;	//Set all analog ports to digital
	ANSB = 0;
	//TRIS bit settings
	TRISCbits.TRISC4 = 0;	//Sets LED_Status as output
	
	TRISBbits.TRISB12 = 0;	//DIG1
	TRISBbits.TRISB13 = 0;	//DIG2
	TRISBbits.TRISB14 = 0;	//DIG3
	TRISBbits.TRISB15 = 0;	//DIG4
	
	TRISAbits.TRISA8 = 0;	//COL
	TRISAbits.TRISA9 = 0;	//APOS
	
	TRISBbits.TRISB5 = 0;	//A
	TRISBbits.TRISB3 = 0;	//B
	TRISBbits.TRISB7 = 0;	//C
	TRISBbits.TRISB2 = 0;	//D
	TRISBbits.TRISB1 = 0;	//E
	TRISBbits.TRISB6 = 0;	//F
	TRISBbits.TRISB4 = 0;	//G
	TRISBbits.TRISB0 = 0;	//DP
	
	TRISCbits.TRISC0 = 1;	//menu SW
	TRISCbits.TRISC1 = 1;	//up SW
	TRISCbits.TRISC2 = 1;	//down SW
	
	INTCON1bits.NSTDIS = 1; //Disables interrupt nesting
	
	//change notification bits
	CNEN3bits.CN32IE = 1;
	CNEN2bits.CN31IE = 1;
	CNEN1bits.CN10IE = 1;
	CNPU3bits.CN32PUE = 1;
	CNPU2bits.CN31PUE = 1;
	CNPU1bits.CN10PUE = 1;
	_CNIE = 1;
	_CNIP = 0b010;
	
	for(i=0; i<=(MAX_DMX); i++)
	{
		DMXdata[i] = 0;
	}
	
	T1CONbits.TON = 1;		//Turns timer1 on
	T1CONbits.TCKPS = 0b11;	//Sets 1:256 prescalar
	T1CONbits.TCS = 0;		//Clock source from internal
	T1CONbits.TGATE = 0;	//Gate acculmalation off
	PR1 = 31250;			//Sets timer1 for .5second
	_T1IE = 1;				//enables timer1 interrupt
	_T1IP = 0b001;			//sets low priority for timer1 interrupt
	
	I2C1BRG=37;
	I2C1CONbits.I2CEN = 1;	//enables I2C
	I2C1CONbits.A10M = 0;	//7-bit address
	_MI2C1IE = 1;			//I2C interrupt enable
//	_MI2C1IP = 0b100;		//priority 4
	sendDAC(SYNC);			//Set DAC output to -10V
	
	T2CONbits.T32 = 0;		//16 bit mode
	T2CONbits.TCKPS = 0b01;	//1:8 prescalar
	T2CONbits.TCS = 0;		//internal clock source
	T2CONbits.TGATE = 0;	//Gated time accumulation is disabled
	PR2 = 11000;			//5.5ms; 500-->0.25ms
	_T2IE = 1;				//enable timer2 interrupt
	T2CONbits.TON = 1;		//Turns timer2 on
//	_T2IP = 0b011;			//priority 3
	
	U1MODEbits.BRGH = 1;
	U1BRG = 15;
	U1MODEbits.PDSEL = 0b00;	//8-bit data, no parity
	U1MODEbits.STSEL = 1;		//2 stop bits
	U1MODEbits.RXINV = 0;
	_U1RXIE = 0;				//enable rx interrupt
	U1STAbits.URXISEL = 0b00;	//interrupt when any char is received
	U1MODEbits.UARTEN = 1;		//enable uart
	_U1ERIE = 0;
	_U1RXIF = 0;				// reset RX flag
//	_U1RXIP = 0b101;			//priority 5
	
	T3CONbits.TCKPS = 0b00;
	T3CONbits.TCS = 0;
	T3CONbits.TGATE = 0;
	PR3 = 64;					//0.004ms
	_T3IE = 1;
	T3CONbits.TON = 1;
//	_T3IP = 0b010;				//priority 2

	//Display
	T4CONbits.T32 = 0;		//16 bit mode
	T4CONbits.TCKPS = 0b00;	//1:1 prescalar
	T4CONbits.TCS = 0;		//internal clock source
	T4CONbits.TGATE = 0;	//Gated time accumulation is disabled
	PR4 = 200;				//6us
	_T4IE = 1;				//enable timer4 interrupt
	T4CONbits.TON = 1;		//Turns timer4 on
//	_T4IP = 2;			//priority 3
	
	//Button debounce
	T5CONbits.TCKPS = 0b10;		//1:64
	T5CONbits.TCS = 0;
	T5CONbits.TGATE = 0;
	PR5 = 2500;					//10ms
	_T5IE = 1;
	T5CONbits.TON = 1;
//	_T5IP = 0b010;				//priority 2
}

// Output number to digit 0,1,2, or 3
void display(char character)
{
	if ((character & 0x80)==0x80)
	{
		PORTB &= 0xFFFF;
	}
	else
	{
		//  Turn on necessary segments
		switch (character)
		{
			case 0:
			case '0':
			case 'O':
				PORTB &= 0xFF24; //G=1
				break;
			case 1:
			case '1':
			case 'l':
				PORTB &= 0xFF77; //A, D, E, F, G=1
				break;
			case 2:
			case '2':
				PORTB &= 0xFF94; //C, F=1
				break;
			case 3:
			case '3':
				PORTB &= 0xFF16; //E, F = 1
				break;
			case 4:
			case '4':
				PORTB &= 0xFF47; //A, D, E = 1;
				break;
			case 5:
			case '5':
			case 'S':
			case 's':
				PORTB &= 0xFF0E; //B, E = 1
				break;
			case 6:
			case '6':
				PORTB &= 0xFF0C; //B = 1
				break;
			case 7:
			case '7':
				PORTB &= 0xFF37; //D, E, F, G = 1
				break;
			case 8:
			case '8':
				PORTB &= 0xFF04;
				break;
			case 9:
			case '9':
			case 'g':
				PORTB &= 0xFF06; //E = 1
				break;
			case 10:
			case 'A':
			case 'a':
				PORTB &= 0xFF05; //D = 1
				break;
			case 12:
			case 'C':
				PORTB &= 0xFFAC; //B, C, G = 1
				break;
			case 'H':
				PORTB&= 0xFF45; //A, D = 1
				break;
		}
	}
}

void threeDigits(char first, unsigned int number)
{
	unsigned int temp;
	digit1 = first;
	if(number < 10)
	{
		digit2 = ' ';
		digit3 = ' ';
		digit4 = (char)number;
	}	
	else if(number < 100)
	{
		digit2 = ' ';
		digit3 = (char)(number/10);
		temp = number - (digit3*10);
		digit4 = (char)(temp);
	}
	else
	{	
		digit2 = (char)(number/100);
		temp = number - (digit2*100);
		digit3 = (char)(temp/10);
		temp = temp - (digit3*10);
		digit4 = (char)(temp);
	}
}

void twoDigits(char first, char second, unsigned int number)
{
	unsigned int temp;
	digit1 = first;
	digit2 = second;
	if(number < 10)
	{
		digit3 = ' ';
		digit4 = (char)number;
	}	
	else
	{
		digit3 = (char)(number/10);
		temp = number - (digit3*10);
		digit4 = (char)(temp);
	}
}
