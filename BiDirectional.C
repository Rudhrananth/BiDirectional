//RUDHRANANTH BALADHANDAPANI     1001232756
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"

#define PUSH_BUTTON (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) //PF4
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED           (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define RS485_DE      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))   //PC6
const uint8_t max=80;
char str[max+1]={NULL};
uint8_t position[10]={NULL};
uint8_t parsecount;
uint8_t ascstr[max+1];
uint8_t newAddress;
uint8_t FL_SEQ_ID=0;
uint8_t rxPhase=0;
uint8_t CHECKSUM;
char ackStatus[] = "OFF";
uint8_t red_timeout = 0;
uint8_t green_timeout = 0;
uint8_t ai;
uint8_t Mode;
uint8_t MYADDRESS= 93;
uint8_t decidecount=0;
uint8_t sizeFlag = 0;


uint8_t rxADDRESS;
uint8_t rxCHANNEL;
uint8_t rxDATA;
uint8_t rxSIZE;
uint8_t rxCOMMAND;
uint8_t rxFL_SEQ_ID;
uint8_t rxCHECKSUM;
uint8_t getVal;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//void send485(uint8_t add, uint8_t cmd, uint8_t ch, uint8_t s, uint8_t DATA[]);

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, C and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOB;

    // Configure LED and pushbutton pins
       GPIO_PORTF_DIR_R |= 0x0E;  // make bit 1,3 an outputs
       GPIO_PORTF_DR2R_R |= 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
       GPIO_PORTF_DEN_R |= 0x1E;  // enable LED, PUSH BUTTON
       GPIO_PORTF_PUR_R = 0x10;

      // Configure UART0 pins
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;



   	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


    // Configure UART1 pins
     SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
     GPIO_PORTC_DEN_R |= 0x70;                           // default, added for clarity
     GPIO_PORTC_AFSEL_R |= 0x30;                         // default, added for clarity
     GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_U1RX | GPIO_PCTL_PC5_U1TX;

     GPIO_PORTC_DIR_R |= 0x40;								//DE as output  '64'  0x60

    // Configure UART1 to 38400 baud
    UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 65;
    UART1_FBRD_R = 7;
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; //| UART_CTL_LBE ; // enable TX, RX, and module

    UART1_IM_R |= UART_IM_RXIM;                       // turn-on RX interrupt
    NVIC_EN0_R |= 1 << (INT_UART1 - 16);               // turn-on interrupt 22 (UART1)


    // Configure Timer 1 as the time base
       SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
       TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
       TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
       TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
       TIMER1_TAILR_R = 0x61A80;                      // set load value for 100 Hz interrupt rate
       TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
       NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
       TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

/////////////////////PWM////////////////////////////////
       GPIO_PORTB_DIR_R |= 0x20;   // make bit5 an output
      	    GPIO_PORTB_DR2R_R |= 0x20;  // set drive strength to 2mA
      	    GPIO_PORTB_DEN_R |= 0x20;   // enable bit5 for digital
      	    GPIO_PORTB_AFSEL_R |= 0x20; // select auxilary function for bit 5
      	    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB5_M0PWM3; // enable PWM on bit 5

      	  // Configure PWM module0 to drive RGB backlight
      	  	    // RED   on M0PWM3 (PB5), M0PWM1b
      	  	    // BLUE  on M0PWM4 (PE4), M0PWM2a
      	  	    // GREEN on M0PWM5 (PE5), M0PWM2b
      	  	    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
      	  	    __asm(" NOP");                                   // wait 3 clocks
      	  	    __asm(" NOP");
      	  	    __asm(" NOP");
      	  	    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
      	  	    SYSCTL_SRPWM_R = 0;                              // leave reset state
      	  	    PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1
      	  	    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
      	  	                                                     // output 3 on PWM0, gen 1b, cmpb

      	  	    PWM0_1_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

      	  	    PWM0_INVERT_R = PWM_INVERT_PWM3INV;
      	  	                                                     // invert outputs for duty cycle increases with increasing compare values
      	  	    PWM0_1_CMPB_R = 0;                               // red off (0=always low, 1023=always high)

      	  	    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1

      	  	    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN;
      	  	                                                     // enable outputs


}

setRgbColor(uint16_t red)
{
	PWM0_1_CMPB_R = red;

}
void runPWM()
{
	int16_t i = 0;
	uint8_t a;
		for(a=0; a<3; a++)
		{
			// Backlight off
			setRgbColor(0);
		    waitMicrosecond(1000000);
			// Ramp from off to bright
		    for (i = 0; i < 1024; i++)
			{
				setRgbColor(i);
			    waitMicrosecond(10000);
			}

		}

}
uart1Parity(uint8_t a)
{
//	    UART1_CTL_R = 0;                                 // turn-off UART1 to allow safe programming
	    if(a==0)
	    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS ; // configure for 8N1, FIFOdisabled, Parity0  0xE2
	    else if (a==1)
	    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_SPS | UART_LCRH_PEN ; // configure for 8N1, FIFOdisabled, Parity1  0xE6
//	    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
	__asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char tempc)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = tempc;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* tempstr)
{
	uint8_t l;
    for (l = 0; l < strlen(tempstr); l++)
	  putcUart0(tempstr[l]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}

void decideMode()
{
char a;
putsUart0(" \n \r Enable Device Mode:");
a = getcUart0();
putcUart0(a);
if(a== '1')
{
	Mode=1;
}
if(a== '0')
	{
	Mode=0;
	}

if(Mode== 1)
{
	RS485_DE = 1;
		 __asm(" NOP");                   // wait 3 clocks
		 __asm(" NOP");
		 __asm(" NOP");

		 while (UART1_FR_R & UART_FR_TXFF);
		 uart1Parity(0);
		 while (UART1_FR_R & UART_FR_TXFF);
		 UART1_DR_R = 43;    //DummyOut
		 while (UART1_FR_R & UART_FR_TXFF);
		 RS485_DE = 0;
		 		 __asm(" NOP");                   // wait 3 clocks
		 		 __asm(" NOP");
		 		 __asm(" NOP");
}
}

void getString()
{
	memset(&str[0], 0, sizeof(str));
	uint8_t count=0;
	putsUart0(" \n \r Enter your Command:");
		//Receive:
		while(count < max)
		{
			uint8_t c = getcUart0();
			putcUart0(c);

		if (c == 8)
	    {
	    	if (count>0)
	    	{
	    		count--;
	    		//goto Receive;
	    	}
	    	//else
	    		//goto Receive;
	    }

		else if (c == 13)
		{
			str[count]= NULL;//0
//			putcUart0("\n");
			break;
	    }

		else if (c >= 32)
		{
			str[count++]=c;
			if (count == max)
			{
				str[count]=NULL;
				break;
//				putcUart0("\n");
			}
			//else
				//goto Receive;
		}

//		else
//			goto Receive; //ignore other character means return to receiving next?

		}
}

void processString()
{
	uint8_t i;
	uint8_t j;
	uint8_t k;
	parsecount=0;
	uint8_t posDelim[80];
	uint8_t delimCount=0;
	char type[10];
//	memset(&posDelim[0], 0, sizeof(posDelim));//Why local variable throws an error?
//	memset(&type[0], 0, sizeof(type));
	memset(&position[0], 0, sizeof(position));

	for(j=0;j<=strlen(str); j++)
	{
		str[j]=toupper(str[j]);
		ascstr[j]=str[j];
	}

	if (ascstr[0] >= 65 && ascstr[0]<= 90)
	{
	        type[parsecount] = 'a';
	        position[parsecount] = 0;
	        parsecount++;

	}
		else if (ascstr[0] >= 48 && ascstr[0]<= 57 )
		{
			type[parsecount] = 'n';
			position[parsecount] = 0;
			parsecount++;
		}



	for (i=0; i< strlen(str); i++)
		{
			if(( ascstr[i] >= 32 && ascstr[i]<= 47 )|| ( ascstr[i] >= 58 && ascstr[i]<= 64 )|| ( ascstr[i] >= 91 && ascstr[i]<= 96) || ( ascstr[i] >= 123 && ascstr[i]<= 255 ) )
//		if( ascstr[i] == 32 || ascstr[i] == 44 )
		{
//				str[posDelim[k]]=NULL;
				posDelim[delimCount]=i;
				delimCount++;
				if((ascstr[i+1] >= 65 && ascstr[i+1]<= 90 )|| (ascstr[i+1] >=97 && ascstr[i+1] <= 122))
				{
					type[parsecount] = 'a';
				    position[parsecount] = i+1;
				    parsecount++;
				}

				else if( ascstr[i+1] >= 48 && ascstr[i+1]<= 57)
				{
					type[parsecount] = 'n';
					position[parsecount] = i+1;
					parsecount++;
				}

		}
		}

	for(k=0; k<delimCount; k++)
	{
		str[posDelim[k]]=NULL;
	}

}


 isCommand(char fieldName[10],int minArgs)
{
	char *str1 = &str[position[0]];
	char *str2=fieldName; //WHY NOT *str2=&fieldName
	if (!strcmp(str1,str2))
	{
		if((parsecount-1)== minArgs)
			return true;
		else
			return false;
	}
	else
		return false;
}

 isAlpha(int fieldNo)
 {
	 uint8_t i;
	 uint8_t j=0;
	 char *argType = &str[position[fieldNo]];
	 for(i=0; i< strlen(argType); i++)
	 	{
	 		if ((ascstr[position[fieldNo]+i])  >= 65 && (ascstr[position[fieldNo]+ i]) <= 90)
	 		j++;
	 	}

	 	if (j == strlen(argType))
	 		 return true;
	  	 else
	  		return false;
	   }


 isNumber(int fieldNo)
  {
	 uint8_t i;
	 uint8_t j=0;
	 char *argType;
	 argType=&str[position[fieldNo]];
	for(i=0; i< strlen(argType); i++)
	{
		if ((ascstr[position[fieldNo]+i])  >= 48 && (ascstr[position[fieldNo]+ i]) <= 57)
		j++;
	}

	if (j == strlen(argType))
		 return true;
 	 else
 		return false;
  }

 uint8_t getNumber(int fieldNo)
 {
	char *arg = &str[position[fieldNo]];
	uint8_t i;
	i=atoi(arg);
	return i;
 }

 char* getAlpha(int fieldNo)
 {
	 char* arg;
	 arg= &str[position[fieldNo]];
	 return arg;
 }

 void analyseString()
 {
//	 uint8_t fl_dat[10]={'\0'};
	 uint8_t fl_dat;
	 char displayRxd[80]={NULL};
	 uint8_t validCommand= false; //bool
	 uint8_t fl_add;
	 uint8_t fl_newAddress;
	 uint8_t fl_ch;
	 uint8_t fl_val;
	 uint8_t fl_sz;
	 uint8_t fl_cmd;

	 if(isCommand("SET",3))
	 {
		 if(isNumber(1))
		 {
			 if(isNumber(2))
				 {
				 if(isNumber(3))
				 	 {
			 fl_add=getNumber(1);
			 fl_ch=getNumber(2);
			 fl_sz= 1;
			 fl_val=getNumber(3);
//			 fl_dat[0]= fl_val;
			 fl_dat= fl_val;

			 validCommand= true;

			 if(!strcmp(ackStatus,"ON"))
			 {
				 fl_cmd= (0x00) + 128;
			 }
			 else if(!strcmp(ackStatus,"OFF"))
			 {
				 fl_cmd= 0x00;
			 }
			 send485(fl_add,fl_cmd,fl_ch,fl_sz,fl_dat);

//			 if cmd needs ack, wait for ack, else retx here with fl_SEQ-- for certain max retries

//			 sprintf(displayRxd,"\n \r SET command with address %d channel %d value %d received \n",add,ch,val);
//			 putsUart0(displayRxd);
				 	 }
				 }
		 }
	 }
	 else if(isCommand("GET",2))
	 	 {
	 		 if(isNumber(1))
	 		 {

	 			 if(isNumber(2))
	 			 {
	 			 fl_add=getNumber(1);
	 			 fl_ch=getNumber(2);
	 			 validCommand= true;
	 			 if(!strcmp(ackStatus,"ON"))
	 			 {
	 				 fl_cmd= (0x30) + 128;
	 			 }
	 			 else if(!strcmp(ackStatus,"OFF"))
	 			 {
	 				 fl_cmd= 0x30;
	 			 }
	 			 send485(fl_add,fl_cmd,fl_ch,fl_sz,fl_dat);
//	 			 sprintf(displayRxd,"\n \r GET command with address %d channel %d received \n",fl_add,fl_ch,fl_val);
//	 			 putsUart0(displayRxd);
	 				 }
	 				 }
	 		 }


	 else if(isCommand("RESET",1))
	 {
		 if(isNumber(1))
		 {
			 fl_add=getNumber(1);
			 validCommand= true;
			 send485(fl_add,127,0,0,0);
		 }
	 }

	 else if(isCommand("SA",1))
	 	 {
	 		 if(isNumber(1))
	 			 {
	 			 if(isNumber(2))
	 		 {
	 			 fl_add=getNumber(1);
	 			 fl_newAddress=getNumber(2);
	 			 validCommand= true;
	 			send485(fl_add,0x79,0,1,fl_newAddress);
	 		 	 }
	 		 }
	 	 }

	 else if(isCommand("ACK",1))
	 {
		 char *temp;
		 if(isAlpha(1))
		 {
			temp =getAlpha(1);
			if(!strcmp(temp,"ON"))
			{
			strcpy(ackStatus,"ON");
			validCommand= true;
//			sprintf(displayRxd,"\n \r ACK command with %s received \n",ackStatus);
//			putsUart0(displayRxd);
			}
			if(!strcmp(temp,"OFF"))
			{
			strcpy(ackStatus,"OFF");
			validCommand= true;
//			sprintf(displayRxd,"\n \r ACK command with %s received \n",ackStatus);
//			putsUart0(displayRxd);
			}

		 }
	 }

	else if(isCommand("POLL",0))
		{
		 	validCommand= true;
		 	fl_add=255;
		 	fl_sz= 0;
			if(!strcmp(ackStatus,"ON"))
			{
				fl_cmd= (0x78) + 128;
 			}
			else if(!strcmp(ackStatus,"OFF"))
			{
	 			 fl_cmd= 0x78;
			}
		 		send485(fl_add,fl_cmd,0,fl_sz,0);
//		 			sprintf(displayRxd,"\n \r POLL command received \n");
//		 			putsUart0(displayRxd);
		 }

	 if(!validCommand)
		 putsUart0("\n \r Syntax Error \n");
 }



void send485(uint8_t ADDRESS, uint8_t COMMAND, uint8_t CHANNEL, uint8_t SIZE, uint8_t DATA)
 {
//	uint8_t temp1,temp2;
	 uint8_t temp=0;
	 uint8_t i;
	 char displayTxd[50];
	 //	 RED_LED=1;
//	 red_timeout = 25;
	 RS485_DE = 1;
	 __asm(" NOP");                   // wait 3 clocks
	 __asm(" NOP");
	 __asm(" NOP");

	 while (UART1_FR_R & UART_FR_TXFF);
	 uart1Parity(1);

	 while (UART1_FR_R & UART_FR_TXFF);
	 UART1_DR_R = ADDRESS;
	 sprintf(displayTxd,"\n \r Transmitted ADDRESS is %u  \n",ADDRESS);
	 waitMicrosecond(1000);
	 putsUart0(displayTxd);

//	 while(((UART1_FR_R & 8) >> 3));
	 while (UART1_FR_R & UART_FR_TXFF);
	 uart1Parity(0);

//	 while(((UART1_FR_R & 8) >> 3));
	 while (UART1_FR_R & UART_FR_TXFF);
	 UART1_DR_R =FL_SEQ_ID;
	 sprintf(displayTxd,"\n \r Transmitted FL_SEQ_ID is %u  \n",FL_SEQ_ID);
	 putsUart0(displayTxd);
	 waitMicrosecond(1000);
	 FL_SEQ_ID++;

//	 while(((UART1_FR_R & 8) >> 3));
	 while (UART1_FR_R & UART_FR_TXFF);
	 UART1_DR_R = COMMAND;
	 sprintf(displayTxd,"\n \r Transmitted COMMAND is %u  \n",COMMAND);
	 putsUart0(displayTxd);
//	 while(((UART1_FR_R & 8) >> 3));
	 waitMicrosecond(1000);

	 while (UART1_FR_R & UART_FR_TXFF);
	 UART1_DR_R = CHANNEL;
	 sprintf(displayTxd,"\n \r Transmitted CHANNEL is %u  \n",CHANNEL);
	 putsUart0(displayTxd);
	 waitMicrosecond(1000);
//	 while(((UART1_FR_R & 8) >> 3));

	 while (UART1_FR_R & UART_FR_TXFF);
	 UART1_DR_R = SIZE;
	 sprintf(displayTxd,"\n \r Transmitted SIZE is %u  \n",SIZE);
	 putsUart0(displayTxd);
	 waitMicrosecond(1000);

//	 while(((UART1_FR_R & 8) >> 3));
	 while (UART1_FR_R & UART_FR_TXFF);
	 UART1_DR_R = DATA;
	 sprintf(displayTxd,"\n \r Transmitted DATA is %u  \n",DATA);
	 putsUart0(displayTxd);
	 waitMicrosecond(1000);
//	 while(((UART1_FR_R & 8) >> 3));

//	 for(i=0; i< SIZE ; i++)
// 	{
//		 UART1_DR_R = *DATA;
//		 while(((UART1_FR_R & 8) >> 3));
//		 temp=temp+(*DATA);
//		 DATA++;
// 	}

//	 while(*DATA != '\0')
//	 {
//		 UART1_DR_R = DATA;
//		 temp=temp+(*DATA);
//		 DATA++;
//	 }

	 CHECKSUM= ~(ADDRESS + (FL_SEQ_ID-1) + COMMAND + CHANNEL + SIZE + DATA);
	 while (UART1_FR_R & UART_FR_TXFF);
	 UART1_DR_R = CHECKSUM;
	 sprintf(displayTxd,"\n \r Transmitted CHECKSUM is %u  \n",CHECKSUM);
	 putsUart0(displayTxd);
	  waitMicrosecond(10000);

//	 while(((UART1_FR_R & 8) >> 3));
	 RS485_DE = 0;
	 __asm(" NOP");                                   // wait 3 clocks
	 __asm(" NOP");
	 __asm(" NOP");
		RED_LED=1;
	    red_timeout = 25;
 }

 void U1RXIsr()
 {
//	 putsUart0("\n \r INSIDE RxISR");

	 char displayRxd[50];
	 int uartdata= UART1_DR_R;
	 uint8_t paritycheck = UART1_LCRH_R;
//	 putsUart0("\n Inside the interrupt");

	 switch(rxPhase)
	 {
	 case 0 :
		 	 	if(((!((paritycheck & 0x04) >> 2)) & (!((uartdata & 0x200) >> 9))) ||  ((paritycheck & 0x04) >> 2) & ((uartdata & 0x200) >> 9))
		 	 	{
		 	 	rxADDRESS = (uartdata & 0xFF);

		 	 	if(Mode==0) //Controller Mode
		 	 {
		 	 		rxPhase=1;
		 	 		GREEN_LED=1;
		 	 		green_timeout = 25;
//		 	 		UART1_ICR_R|= 0x10;
		 	 		break;		//Controller Mode
		 	 }

		 	 	if(Mode==1) //Device Mode
		 	 {
		 	 	 if((MYADDRESS == rxADDRESS) |(255 == rxADDRESS))
		 	 	 {
		 	 		GREEN_LED=1;
		 	 		green_timeout = 25;
		 	 		sprintf(displayRxd,"\n \r Received ADDRESS is %u  \n",rxADDRESS);
		 	 		putsUart0(displayRxd);
		 	 		rxPhase=1;
//		 	 		UART1_ICR_R|= 0X10;
		 	 		break;
		 	 	 }
		 	 	 else
		 	 		{
		 	 		 putsUart0("\n \r Wrong Adress");
//	 	 		UART1_ICR_R|= 0X10;
		 	 		 break;
		 	 		}
		 	 }
		 	 	 }
//		 	 	 else
//		 	 		 {putsUart0("\n \r Parity problem");
//		 	 		UART1_ICR_R|= 0X10;
//			 	 	putsUart0("\n \r Phase 1");
		 	 	 break;
	 case 1 :
		 	 	 rxFL_SEQ_ID = (uartdata & 0xFF);
		 	 	 rxPhase=2;
		 	 	sprintf(displayRxd,"\n \r Received FL_SEQ_ID is %u  \n",rxFL_SEQ_ID);
		 	 	putsUart0(displayRxd);
//		 	 	UART1_ICR_R|= 0X10;
//		 	 	putsUart0("\n \r Phase 2");
		 	 	 break;

	 case 2 :
	 		 	 rxCOMMAND = (uartdata & 0xFF);
	 		 	 rxPhase=3;
	 		 	sprintf(displayRxd,"\n \r Received COMMAND is %u  \n",rxCOMMAND);
	 		 	putsUart0(displayRxd);
//	 		 	UART1_ICR_R|= 0X10;
//	 		 	putsUart0("\n \r Phase 3");
	 		 	 break;

	 case 3 :
	 	 		 rxCHANNEL = (uartdata & 0xFF);
	 	 		 rxPhase=4;
	 	 		sprintf(displayRxd,"\n \r Received CHANNEL is %u  \n",rxCHANNEL);
	 	 		putsUart0(displayRxd);
//	 	 		UART1_ICR_R|= 0X10;
//	 	 		putsUart0("\n \r Phase 4");
	 	 		 break;

	 case 4 :
	 	 		 rxSIZE = (uartdata & 0xFF);
	 	 		 rxPhase=5;
	 	 		sprintf(displayRxd,"\n \r Received SIZE is %u  \n",rxSIZE);
	 	 		putsUart0(displayRxd);
//	 	 		UART1_ICR_R|= 0X10;
//	 	 		putsUart0("\n \r Phase 5");
	 	 		 break;

	 case 5 :
		 	 rxDATA = (uartdata & 0xFF);
		 	 sprintf(displayRxd,"\n \r Received data is %u  \n",rxDATA);
		 	 putsUart0(displayRxd);
//		 	UART1_ICR_R|= 0X10;
		 	rxPhase=6;
		 	break;
	 case 6:
		 	 		rxCHECKSUM = (uartdata & 0xFF);
		 	 		sprintf(displayRxd,"\n \r Received CHECKSUM is %u  \n",rxCHECKSUM);
		 	 		putsUart0(displayRxd);
		 	 		rxPhase=0;
//		 	 		UART1_ICR_R|= 0X10;

		 	 		//PROCESS COMMAND

		 	 		if(Mode==1) // DEVICE MODE
		 	 	{
		 	 		if(((rxCOMMAND & 0x80) >> 7)) //STEP8 Command that requires handshake received so send Acknowlege
		 				{
		 				 send485(MYADDRESS,0x70,0,1,rxFL_SEQ_ID);
		 				putsUart0("\n \r Ack sent \n");
		 				}



		 	 		if(rxCOMMAND == 0x78) //STEP8 POLL requested so send poll response F0
		 	 			{
		 	 			send485(MYADDRESS,0x78,0,1,MYADDRESS);
		 	 			putsUart0("\n \r Poll response sent \n");
		 	 			}

		 	 		if(rxCOMMAND == 0x30) //STEP8 GET VALUE A8
		 	 			{
		 	 			if(rxCHANNEL == 30)
		 	 			{
		 	 			send485(MYADDRESS,0x20,0,1,BLUE_LED);
		 	 			sprintf(displayRxd,"\n \r Value %u sent \n",BLUE_LED);
		 	 			putsUart0(displayRxd);
		 	 			}
		 	 			if(rxCHANNEL == 25)
		 	 				{
		 	 				send485(MYADDRESS,0x20,0,1,BLUE_LED);
		 	 				sprintf(displayRxd,"\n \r Value %u sent \n",PUSH_BUTTON);
		 	 				putsUart0(displayRxd);
		 	 				}

		 	 			}
		 	 		if(rxCOMMAND == 0x70)   //STEP9 Acknowledgement received So letting user know
		 				 {
		 				 	sprintf(displayRxd,"\n \r Controller Acknowledged SEQID %u \n",rxDATA);
		 				 	putsUart0(displayRxd);
		 				 }

		 	 		if(rxCOMMAND == 0x79)   //Set Address
		 	 			{
		 	 				 sprintf(displayRxd,"\n \r New Address set to %u \n",rxDATA);
		 	 				 putsUart0(displayRxd);
		 	 			}

		 	 		if(rxCOMMAND == 0x7F)   //STEP9 RESET
		 	 				 {
		 	 				NVIC_APINT_R = NVIC_APINT_VECTKEY | 0x4;
		 	 				 putsUart0("\n \r Reset Command Received");
		 	 				  }
		 	 		if(rxCOMMAND == 0x0 || rxCOMMAND == 0x80)   //STEP9 SET command received
		 	 			{
		 	 			 if(rxCHANNEL == 20 )
		 	 				BLUE_LED = rxDATA;
		 	 			}
		 	 		}



		 	 		if(Mode==0) //CONTROLLER MODE
		 	 	{
		 	 		if(((rxCOMMAND & 0x80) >> 7)) //STEP8 Command that requires handshake received so send Acknowlege
		 	 		{
		 	 		 send485(rxADDRESS,0x70,rxCHANNEL,rxSIZE,rxFL_SEQ_ID);
		 	 		}

		 	 		if(rxCOMMAND == 0x70)   //STEP9 Acknowledgement received So letting user know
		 	 		 {
		 	 		sprintf(displayRxd,"\n \r Node %u Acknowledged SEQID %u \n",rxADDRESS,rxDATA);
		 	 		putsUart0(displayRxd);
		 	 		 }

		 	 		if(rxCOMMAND == 0x20)   //STEP9 Receive Value in response to GET 98
		 	 		{
		 	 		sprintf(displayRxd,"\n \r Node %u sent Value %u \n",rxADDRESS,rxDATA);
		 	 		putsUart0(displayRxd);
		 	 		}


		 	 		if(rxCOMMAND == 0x78)   //STEP9 Letting User know the Poll response F0
		 	 		{
		 	 		sprintf(displayRxd,"\n \r Node %u responding \n",rxADDRESS);
		 	 		putsUart0(displayRxd);
		 	 		}
				}

		 	 		break;

	 default:
		 break;
	 }

	 }



 void timer1Isr()
 {
if(red_timeout != 0)
	red_timeout--;
if(red_timeout == 0)
	RED_LED = 0;
if(green_timeout != 0)
	green_timeout--;
if(green_timeout == 0)
	GREEN_LED = 0;
TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
 }
//-----------------------------------------------------------------------------
// Main

//-----------------------------------------------------------------------------

int main(void)

{
	initHw();
	{
		decideMode();

	while(1)
		{


		  GREEN_LED = 1;
	      waitMicrosecond(500000);
	      GREEN_LED = 0;

	      waitMicrosecond(500000);
		  getString();
		  processString();
		  analyseString();
//	      runPWM();
		}
	}
}




///////////////////STEP10,11,12code//////////////////////////////////////////////////////////////////////

//void Tx485(uint8_t add, uint8_t cmd, uint8_t ch, uint8_t sz, uint8_t *dt)
//{
//	uint8_t ind=0;
//	uint8_t i;
//
//	while((ind<256) && (tx_valid[w485_index]!=0))
//	{
//		w485_index= (w485_index+1)%256;
//		ind= (ind+1)%256;
//	}
//
////	for(ind=w485_index; valid[ind]==0 ; (ind+1)%256 )
////	{
////		w485_index = ind;
////	}
//	if(tx_valid[w485_index] == 0)
//	{
//	tx_ADDRESS[w485_index]=add;
//	if(((cmd & 0x80) >> 7) == 1)
//	{
//		tx_ack[w485_index]= 1;
//		tx_retx_count[w485_index]=0;
//		tx_time_retx[w485_index]=500;
//	}
//	FL_SEQ_ID[w485_index]= flseqid;
//	flseqid++;
//	tx_COMMAND[w485_index]=cmd;
//	tx_CHANNEL[w485_index]=ch;
//	tx_SIZE[w485_index]= sz;
//	for(i=0; i< sz ; i++)
//	 	{
//		tx_DATA[w485_index][i] = *dt;
//		tx_datasum[w485_index]=tx_datasum[w485_index]+(*dt);
//		dt++;
//	 	}
//	tx_CHECKSUM[w485_index]= ~(tx_ADDRESS[w485_index] + (FL_SEQ_ID[w485_index]-1) + tx_COMMAND[w485_index] + tx_CHANNEL[w485_index] + tx_SIZE[w485_index] + tx_datasum[w485_index]);
//	tx_valid[w485_index]=1;
//	w485_index= (w485_index+1)%256;
////	while(((UART1_FR_R & 8) >> 3));
////	UART1_ICR_R |= 0X20; //clearing TXRIS in UARTRIS manually
//
//	uart1Parity(0);
//	UART1_DR_R = 80;
//	while(((UART1_FR_R & 8) >> 3));
//	}
//}
//
// void U1RXIsr()
// {
//	 uint8_t rxADDRESS;
//	 uint8_t rxCHANNEL;
//	 uint8_t rxDATA[]={NULL};
//	 int rxdatasum=0;
//	 uint8_t rxSIZE;
//	 uint8_t rxCOMMAND;
//	 uint8_t rxFL_SEQ_ID;
//	 uint8_t rxCHECKSUM;
//	 uint8_t getVal;
//	 uint8_t MYADDRESS= 10;
//	 uint8_t localchecksum;
//	 char displayRxd[50];
//	 int uartdata= UART1_DR_R;
//	 uint8_t paritycheck = UART1_LCRH_R;
//	 uint8_t ind1 = 0;
//	 uint8_t ind2=0;
////	 putsUart0("\n Inside the interrupt");
//
//	 if(UART1_RIS_R & 0X20) //TxEmpty Interrupt
//{
//	if((r485_phase == 0 && !tx_ack[r485_index]) || (((tx_ack[r485_index] == 1) && tx_time_retx[r485_index]==0)))
////		r485_index=(r485_index+1)%256;
////
////		if(r485_phase == 0 && ((UART1_FR_R & 8) >> 3)!=1);
//		{
//		while((ind1<256) && (tx_valid[r485_index]!=1))
//		{
//				r485_index= (r485_index+1)%256;
//				ind1 = (ind1+1)%256;
//		}
//		}
//
//	if(tx_valid[r485_index]==1)
//{
//		 while(((UART1_FR_R & 8) >> 3));
//		 RS485_DE = 1;
//		__asm(" NOP");                   // wait 3 clocks
//		__asm(" NOP");
//		__asm(" NOP");
//
//		switch(r485_phase)
//		{
//		case 0 :
//			RED_LED=1;
//			red_timeout = 25;
//			while (UART1_FR_R & UART_FR_TXFF);
//			uart1Parity(1);
//			while (UART1_FR_R & UART_FR_TXFF);
//			UART1_DR_R = tx_ADDRESS[r485_index];
//			r485_phase=1;
//			break;
//
//		case 1:
//			while(((UART1_FR_R & 8) >> 3));
//			uart1Parity(0);
//			while(((UART1_FR_R & 8) >> 3));
//			UART1_DR_R =FL_SEQ_ID[r485_index];
//			r485_phase=2;
//			break;
//
//		case 2:
//			while(((UART1_FR_R & 8) >> 3));
//			UART1_DR_R = tx_COMMAND[r485_index];
//			r485_phase=3;
//			break;
//
//		case 3:
//			while(((UART1_FR_R & 8) >> 3));
//			UART1_DR_R = tx_CHANNEL[r485_index];
//			r485_phase=4;
//			break;
//
//		case 4:
//			 while(((UART1_FR_R & 8) >> 3));
//			 UART1_DR_R = tx_SIZE[r485_index];
//			 r485_phase=5;
//			 ti=0;
//			 break;
//
//		 case 5 :  //MULTIPLE SIZE DATA CASE
//
//			 if(tx_SIZE[r485_index] == 0||ti==tx_SIZE[r485_index])
//			 {
//			    UART1_DR_R = tx_CHECKSUM[r485_index];
//
//			    if(tx_ack[r485_index] != 1)
//			    {
//			    tx_valid[r485_index]=0;  //aCK NOT REQUIRED MAKE INVALID
//			    }
//			    else
//			    {
//			    tx_retx_count[r485_index]++; //ACK REQUIRED INCREASE TRANSMISSION COUNT
//			    if(tx_retx_count[r485_index] > 3)
//			    {
//			    sprintf(displayRxd,"\n \r  Transmission timed out %u  \n",rxFL_SEQ_ID);
//		 	 	putsUart0(displayRxd);
//		 	 	tx_valid[r485_index]=0;
//			    }
//			    else tx_time_retx[r485_index] = tx_retx_count[r485_index] + (2^(tx_retx_count[r485_index])*500);
//			    }
//			    r485_index=((r485_index+1)%256);
//			    r485_phase=0;
//			 	while(((UART1_FR_R & 8) >> 3));
//			 	RS485_DE = 0;
//			  __asm(" NOP");                                   // wait 3 clocks
//			  __asm(" NOP");
//			  __asm(" NOP");
//
//			 }
//			 else if(ti < tx_SIZE[r485_index])
//			 {
//				UART1_DR_R=tx_DATA[r485_index][ti];
////			 	txdatasum[r485_index]= txdatasum[r485_index] + tx_DATA[r485_index][ti];
//			 	ti++;
//			 	break;
//			 }
//
//		}
//}
//}
//
//
//	 if(UART1_RIS_R & 0X10)
//{
//	 switch(rxPhase)
//	 {
//	 case 0 :
//		 	 	 if((!((paritycheck & 0x04) >> 2)) & (!((uartdata & 0x200) >> 9)))
//		 	 	 {
//		 	 	GREEN_LED=1;
//		 	 	green_timeout = 25;
//		 	 	rxADDRESS = (uartdata & 0xFF);
//		 	 	if(Mode==0) rxPhase=1; //Controller Mode
//		 	 	if(Mode==1) //Device Mode
//		 	 	{
//		 	 	 if((MYADDRESS == rxADDRESS) |(255 == rxADDRESS))
//		 	 	 {
//		 	 		sprintf(displayRxd,"\n \r Received ADDRESS is %u  \n",rxADDRESS);
//		 	 		putsUart0(displayRxd);
//		 	 		rxdatasum = 0;
//		 	 		rxPhase=1;
//		 	 	 }
//		 	 	 else
//		 	 	 {
//		 	 		putsUart0("\n \r Wrong Adress");
//		 	 	 }
//		 	 	}
//		 	 	 }
////		 	 	putsUart0("\n \r Phase 1");
//		 	 	 break;
//	 case 1 :
//		 	 	 rxFL_SEQ_ID = (uartdata & 0xFF);
//		 	 	 rxPhase=2;
//		 	 	sprintf(displayRxd,"\n \r Received FL_SEQ_ID is %u  \n",rxFL_SEQ_ID);
//		 	 	putsUart0(displayRxd);
////		 	 	putsUart0("\n \r Phase 2");
//		 	 	 break;
//
//	 case 2 :
//	 		 	 rxCOMMAND = (uartdata & 0xFF);
//	 		 	 rxPhase=3;
//	 		 	sprintf(displayRxd,"\n \r Received COMMAND is %u  \n",rxCOMMAND);
//	 		 	putsUart0(displayRxd);
////	 		 	putsUart0("\n \r Phase 3");
//	 		 	 break;
//
//	 case 3 :
//	 	 		 rxCHANNEL = (uartdata & 0xFF);
//	 	 		 rxPhase=4;
//	 	 		sprintf(displayRxd,"\n \r Received CHANNEL is %u  \n",rxCHANNEL);
//	 	 		putsUart0(displayRxd);
////	 	 		putsUart0("\n \r Phase 4");
//	 	 		 break;
//
//	 case 4 :
//	 	 		 rxSIZE = (uartdata & 0xFF);
//	 	 		 rxPhase=5;
//	 	 		sprintf(displayRxd,"\n \r Received SIZE is %u  \n",rxSIZE);
//	 	 		putsUart0(displayRxd);
////	 	 		putsUart0("\n \r Phase 5");
//	 	 		 ri=0;
//	 	 		 break;
//
//	 case 5 :
//		 	 	 if(rxSIZE == 0||ri==rxSIZE)
//				 {
//		 	 		rxCHECKSUM = (uartdata & 0xFF);
//		 	 		sprintf(displayRxd,"\n \r Received CHECKSUM is %u  \n",rxCHECKSUM);
//		 	 		putsUart0(displayRxd);
//		 	 		rxPhase=0;
//
//		 	 		//PROCESS COMMAND
//
//	 	 		if(Mode=1) // DEVICE MODE
//		 	 		{
//		 	 		if(((rxCOMMAND & 0x80) >> 7) == 1) //STEP8 Command that requires handshake received so send Acknowlege
//		 				{
//		 				 Tx485(MYADDRESS,0x70,0,1,rxFL_SEQ_ID);
//		 				}
//
//		 	 		if(rxCOMMAND == 0X78 | 0XF0) //STEP8 POLL requested so send poll response
//		 	 			{
//		 	 			Tx485(MYADDRESS,0x78,0,1,MYADDRESS);
//		 	 			}
//
//		 	 		if(rxCOMMAND == 0X30 | 0XA8) //STEP8 GET VALUE
//		 	 			{
//		 	 			Tx485(MYADDRESS,0x20,0,1,getVal);
//		 	 			}
//		 	 		if(rxCOMMAND == 0x70)   //STEP9 Acknowledgement received So letting user know
//		 				 {
//		 				 	sprintf(displayRxd,"\n \r Controller Acknowledged SEQID %u \n",rxDATA);
//		 				 	putsUart0(displayRxd);
//		 				 }
//		 	 		if(rxCOMMAND == 0x70)   //STEP9 Acknowledgement received So letting user know
//		 	 		 	{
//		 	 		 		sprintf(displayRxd,"\n \r Controller Acknowledged SEQID %u \n",rxDATA);
//		 	 				putsUart0(displayRxd);
//		 	 			}
//
//		 			}
//
//
//
//		 	 		if(Mode=0) //CONTROLLER MODE
//		 	 		{
//		 	 		if(((rxCOMMAND & 0x80) >> 7) == 1) //STEP8 Command that requires handshake received so send Acknowlege
//		 	 		{
//		 	 		 Tx485(rxADDRESS,0x70,rxCHANNEL,rxSIZE,rxFL_SEQ_ID);
//		 	 		}
//
//		 	 		if(rxCOMMAND == 0x70)   //STEP9 Acknowledgement received So letting user know
//		 	 		 {
//		 	 		sprintf(displayRxd,"\n \r Node %u Acknowledged SEQID %u \n",rxADDRESS,rxDATA);
//
//		 	 		while((ind2<256) && FL_SEQ_ID[r485_index]== rxDATA)
//		 	 				{
//		 	 				tx_valid[r485_index]=0;
//		 	 				ind2 = (ind2+1)%256;
//		 	 				}
//		 	 		 }
//
//		 	 		if(rxCOMMAND == 0x20 | 0X98)   //STEP9 Receive Value in response to GET
//		 	 		{
//		 	 		sprintf(displayRxd,"\n \r Node %u sent Value %u \n",rxADDRESS,rxDATA);
//		 	 		putsUart0(displayRxd);
//		 	 		}
//
//		 	 		if(rxCOMMAND == 0x78 | 0XF0)   //STEP9 Letting User know the Poll response
//		 	 		{
//		 	 		sprintf(displayRxd,"\n \r Node %u responding %u \n",rxADDRESS);
//		 	 		putsUart0(displayRxd);
//		 	 		}
//		 	 		}
//
//					break;
//				 }
//
//				 else if(ri < rxSIZE)
//				 {
//					 rxDATA[ri] = (uartdata & 0xFF);
//					 rxdatasum= rxdatasum + rxDATA[ri];
//					 sprintf(displayRxd,"\n \r Received data is %u  \n",rxDATA[ri]);
//					 ri++;
//					 putsUart0(displayRxd);
//					 break;
//				 }
//
//	 default:
//	 }
//}
//
// }
