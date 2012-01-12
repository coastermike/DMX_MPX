#include <p24FV32KA301.h>

void setup(void);
void sendDAC(int value);

_FDS(DSWDTEN_OFF & DSBOREN_OFF)	//Deep Sleep Watchdog disabled, Deep sleep Zero-Power BOR Disabled
_FICD(ICS_PGx1)					//ICD Pin on PGC1/PGD1
_FPOR(BOREN_BOR3 & I2C1SEL_PRI & BORV_V30 & MCLRE_ON) //Brown-out reset enabled in hardware, Default I2C1 pins, Brownout set to 3V, MCLR pin enabled
_FWDT(FWDTEN_OFF & WINDIS_OFF)	//Watchdog disabled
_FOSC(OSCIOFNC_OFF & POSCMOD_NONE) 	//CLKO disable, primary osc diabled
_FOSCSEL(FNOSC_FRCPLL)					//Fast RC oscillator
_FGS(GWRP_OFF & GSS0_OFF)			//Flash write protection disabled, no protection
_FBS(BWRP_OFF & BSS_OFF)			//boot protects disabled

//Define pins
//RB14 DMXdata
#define LED_DMX PORTBbits.RB14

//RB15 Status
#define LED_Status PORTBbits.RB15


//Define constant values
#define DAC_W_ADDRESS 0b11000000//Address + write bit for I2C DAC
#define SYNC 0b0000000000		//Results in -10V sync pulse
#define ZERO 0b0111111111		//DAC value for 0V output
#define FULL 0b1111111111		//DAC value for 10V output
#define MAX_CHANNEL 16				//Sets the max channel outputs
#define MAX_DMX 16

//Define global variables
unsigned int DACvalue = 0;		//10bit value to be sent to the DAC
unsigned int stateDAC = 0;		//state of DAC to keep track of I2C sending
unsigned int stateMPX = 0;		//state of MPX transmission
unsigned int channelOutCount = 1;//keeps track of output channel
unsigned int DMXchannelCount = 0;//keeps track of DMX in channel
unsigned char DMXdata[MAX_DMX+1];
unsigned int breakZeros = 0, zeroState = 0;
unsigned int dmxnew = 0, dmxcurrent = 0, dmxaddress = 8, dmxreceived = 0;
unsigned int idleHistory = 0;

//T1 interrupt for status LED
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
	LED_Status = ~LED_Status;
	if(U1STAbits.RIDLE && (idleHistory > 2))
	{
		LED_DMX = 0;
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
	if(_RB2 && zeroState == 0)
	{
		breakZeros = 0;
	}
	else if(_RB2 && zeroState == 1)
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
		LED_DMX = 1;
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
		LED_DMX = 0;
	}
	if(U1STAbits.PERR)
	{
		LED_DMX = 0;
	}
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
	int i;
	CLKDIVbits.RCDIV = 0b000;
	ANSA = 0;	//Set all analog ports to digital
	ANSB = 0;
	TRISBbits.TRISB14 = 0;	//Sets LED_DMX as output
	TRISBbits.TRISB15 = 0;	//Sets LED_Status as output
	
	INTCON1bits.NSTDIS = 1; //Disables interrupt nesting

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
	T2CONbits.TCKPS = 0b01;	//1:1 prescalar
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
}
