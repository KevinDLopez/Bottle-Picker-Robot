
#include "stdint.h"
#include <stdlib.h>
#include "inc/tm4c123gh6pm.h"
#include <math.h>


// From ARM2.1_INIT
#define PWM_0_GENA_ACTCMPAD_ONE 0x000000C0  // Set the output signal to 1
#define PWM_0_GENA_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0
#define PWM_0_GENB_ACTCMPBD_ONE 0x00000C00  // Set the output signal to 1
#define PWM_0_GENB_ACTLOAD_ZERO 0x00000008  // Set the output signal to 0

#define SYSCTL_RCC_USEPWMDIV    0x00100000  // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M     0x000E0000  // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2     0x00000000  // /2



//#define INITIAL_PWM_LOAD  	65000 // 1.25Mhz / 65000 = 19hz  or 50 MiliSeconds  // note high current 
//#define INITIAL_PWM_LOAD  	25000 // 1.25Mhz / 25k = 50hz  or 20 MiliSeconds  // note med current 
#define INITIAL_PWM_LOAD  	1250 // 1.25Mhz / 1250 = 1khz  or 1 MiliSeconds  // note low current 
#define INITIAL_PWM_COMP		1000
#define TURNING_PWM_COM 		800

	#define aSize 2
	#define rSize 2
	#define lSize 2

#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
//#define SW1       0x10                      // on the left side of the Launchpad board
//#define SW2       0x01                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define SWITCHES  (*((volatile uint32_t *)0x40025044))
#define SW2      0x10                      // on the left side of the Launchpad board
#define SW1      0x01                      // on the right side of the Launchpad board
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define YELLOW    0x0A
#define SBLUE     0x0C
#define PINK      0x06
#define WHITE 		0X0E

#define MC_LEN 0.0625      // length of one machine cyce in microsecond for 16MHz clock
#define SOUND_SPEED 0.0343 // centimeter per micro-second


#define SYSCTL_RIS_PLLLRIS      0x00000040  // PLL Lock Raw Interrupt Status
#define SYSCTL_RCC_XTAL_M       0x000007C0  // Crystal Value
#define SYSCTL_RCC_XTAL_6MHZ    0x000002C0  // 6 MHz Crystal
#define SYSCTL_RCC_XTAL_8MHZ    0x00000380  // 8 MHz Crystal
#define SYSCTL_RCC_XTAL_16MHZ   0x00000540  // 16 MHz Crystal
#define SYSCTL_RCC2_USERCC2     0x80000000  // Use RCC2
#define SYSCTL_RCC2_DIV400      0x40000000  // Divide PLL as 400 MHz vs. 200
                                            // MHz
#define SYSCTL_RCC2_SYSDIV2_M   0x1F800000  // System Clock Divisor 2
#define SYSCTL_RCC2_SYSDIV2LSB  0x00400000  // Additional LSB for SYSDIV2
#define SYSCTL_RCC2_PWRDN2      0x00002000  // Power-Down PLL 2
#define SYSCTL_RCC2_BYPASS2     0x00000800  // PLL Bypass 2
#define SYSCTL_RCC2_OSCSRC2_M   0x00000070  // Oscillator Source 2
#define SYSCTL_RCC2_OSCSRC2_MO  0x00000000  // MOSC
#define SYSDIV2 4


//void putOnBox(void); 

// bus frequency is 400MHz/(SYSDIV2+1) = 400MHz/(4+1) = 80 MHz
// see the table at the end of this file

// configure the system to get its clock from the PLL
void PLL_Init(void){
  // 0) configure the system to use RCC2 for advanced features
  //    such as 400 MHz PLL and non-integer System Clock Divisor
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   // clear XTAL field
  SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;// configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;// clear oscillator source field
  SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO;// configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
  // 4) set the desired system divider and the system divider least significant bit
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;  // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000) // clear system clock divider field
                  + (SYSDIV2<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){};
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
}


////////////////// ENCODER DEFINITION
	int32_t leftEncoder; //pf0
	int32_t rightEncoder; //pc6
	int32_t RightEncoderRatio, LeftEncoderRatio;
	uint8_t CAR_ACTION;
	uint8_t CAR_RELATED2PLANE;   	// CAR2PLANE{UP,LEFT, RIGHT, DOWN};  ON WHAT DIRRECTION IS THE CAR TRAVELING RELATED TO THE PLANE
	uint8_t PAST_LOCATIONS[100][100]; // Y, X    
	uint8_t xCarPosition = 50, yCarPosition = 50; //XPLANE, YPLANE IS CHANGED BY SENSORS AND CAMARA
	uint8_t RightEncoderState, LeftEncoderState; 
	int32_t leftEncoder2, rightEncoder2, newQuadR, newQuadL;
	int32_t differenceOfEncoders;

////////////////////	ULTRASONIC DEFINITION
	uint32_t iUltraSonic; 		// To keep count of MicroSeconds
	uint32_t RightSensorTime, LeftSensorTime; // object of farthest distance 
	uint16_t  RightSensorDistance, LeftSensorDistance;
	uint32_t FrontSensorTime, FrontSensoDistance;
	uint16_t FrontSensorDistances[aSize];
	uint16_t RightSensorDistances[rSize];
	uint16_t LeftSensorDistances[lSize];
	uint8_t nF, nR, nL;
	uint16_t averageFrontS = 999, averageRightS = 999, averageLeftS = 999, totalRightS, totalFrontS, RSD, LSD, FSD;


//////////////////// UART DEFINITIONS 
	uint16_t UART0_DATA, UART0_STATE, X_POSITION, Y_POSITION, UART0D_TEMP1, UART0D_TEMP2, UART0_TIMEOUT;
	uint16_t UART0_CHECK_SUM;
	uint16_t UART0D_TEMP3, UART0D_TEMP4, UART0D_TEMP5, UART0D_TEMP6, UART0D_TEMP7, UART0D_TEMP8, UART0D_TEMP9, TEMPX, TEMPY, UART0D_TEMP10;


///////////////////// PID FOR TRACKING CAN
	int PWM_RightWheel_READ ;
	int PWM_leftWheel_READ ; //READ 

//ARM CONTROLLER
void harm_graveX(uint16_t x_pos, uint16_t base);

void IPWM0AnB_Init(uint16_t duty){ //module 1     pwm1    
	//              gen 3b 
  volatile unsigned long delay; 
  // enable Peripheral Clocks
  SYSCTL_RCGCPWM_R |= 2;        // enable clock to PWM1
  SYSCTL_RCGCGPIO_R |= 0x18;    // enable clock to GPIOD, E -----                    FE-DCBA
	SYSCTL_RCC_R |= 0x001E0000;	// pre-divide CLOCK FOR PWM, 80MHZ/64   = 1.25Mhz   PG.254
  
  // Enable port PD0,1 FOR PWM
  GPIO_PORTD_AFSEL_R |= 0x3;           
  // make PD0,1  PWM output pin
  GPIO_PORTD_PCTL_R &= ~0x000000FF; 
  GPIO_PORTD_PCTL_R |=  0x00000055; 
  GPIO_PORTD_DEN_R |= 0x3;   
	// Enable port E for H-Bridge
	GPIO_PORTE_DEN_R |= 0x3C;  // Pins 2,3,4,5
  GPIO_PORTE_DIR_R |= 0x3C;  // Outputs 
  
  PWM1_0_CTL_R = 0;           // PWM stop counter
	PWM1_0_GENA_R = 0xC8;      // M1PWM0 reload down count  set when load is smaller tha cmpa----- for PD0
	PWM1_0_GENB_R = 0xC08;      // M1PWM1 reload down count  set when load is smaller tha cmpa  -- FOR PD1
  PWM1_0_LOAD_R = INITIAL_PWM_LOAD -1; ;      // reload value for 1kHz
  PWM1_0_CMPA_R = duty*5;      // match comparator value to make signal low
	PWM1_0_CMPB_R = duty*5;       // match comparator value to make signal low
  PWM1_0_CTL_R = 1;           // start timer
  PWM1_ENABLE_R |= 0x3;       // start PWM1 CHANNEL 0, 1
}

// 57600, 8 Data bits, 1 stop, odd pari    , interupt
void UART7_INIT(void){ //PE0,1  rx,tx
  SYSCTL_RCGCUART_R |=  SYSCTL_RCGCUART_R7;
  SYSCTL_RCGCGPIO_R |=  0x10; // CLOCK FOR PORT E

  GPIO_PORTE_DEN_R |= 3;  
  GPIO_PORTE_AFSEL_R |= 3; // ALTERNATE FUNTION 
  GPIO_PORTE_PCTL_R |= 0x00000011;

  UART7_CTL_R = 0;
  UART7_IBRD_R = 86;   //17;  104;
  UART7_FBRD_R = 51; //731;		11
  UART7_CC_R = 0;
  UART7_LCRH_R |= 0x62;  // 8 bits,  Odd Paraty enable 

  UART7_IM_R |= 0x0010;  // ENABLE RX interrupt   pg. 924 UARTIM      INTERRUPT


  UART7_CTL_R |= 0x300;  // ENABLE RX TX 
  UART7_CTL_R |= 1;      // ENABLE UART

  NVIC_EN1_R |= 0x80000000;     // enable IRQ63 for UART7   pg.104     INTERRUPT   
 // NVIC_PRI15_R = 3 << 29;       // GIVE PRIOTY 3 ON INTERRUPT          INTERRUPT

}


void UART7_Tx(uint8_t ByteToSend){
  while((UART7_FR_R & UART_FR_TXFF) != 0);
    UART7_DR_R = ByteToSend ;
}

uint8_t UART7_Rx(void){
  while((UART7_FR_R & UART_FR_RXFE) != 0);
  return UART7_DR_R;
}

void UART7_Transmit_String(const uint8_t *MessageString){
  while(*MessageString){ // while there is a character
    UART7_Tx(*MessageString); //send character
    MessageString++;
  }
}


 
void UART0_INIT(){ // INTERRUPT  8-bit,  ODD parity, 1-stop
	volatile int32_t herf; 
	// initialize UART0
	SYSCTL_RCGCGPIO_R |= 0x01;		// enable clock to PORTA
	herf = SYSCTL_RCGCGPIO_R;
	GPIO_PORTA_DEN_R 		|= 0x03;			// PA1,0 are digital
	GPIO_PORTA_AFSEL_R 	|= 0x03;		// PA1,0 alternate function enable
	GPIO_PORTA_PCTL_R 	|= 0x00000011;			// PA1,0 configured for UART
	
	SYSCTL_RCGCUART_R |= 0x01;		// enable clock to UART0
	herf = SYSCTL_RCGCGPIO_R;
	UART0_CTL_R = 0;							// disable UART0
	UART0_IBRD_R = 43;						// 9600 baud integer portion
	UART0_FBRD_R = 23;						// 9600 baud fraction portion
	UART0_CC_R = 0;								// UART0 timed using System clock
	UART0_LCRH_R = 0x62;					// 8-bit, ODD parity, 1-stop,  no FIFO   Line Control PG.917
	UART0_IM_R |= 0x0010;					// enable RX interrupt :	ds 	.p924 UARTIM
	UART0_CTL_R |= 0x301;					// enable UART0, TX, RX
	
	NVIC_PRI1_R |= 3 << 13;				// set interrupt priority to 3
	NVIC_EN0_R |= 0x00000020;			// enable IRQ5 for UART0
	
}
void UART0_TX(uint8_t ByteToSend){
  while((UART0_FR_R & UART_FR_TXFF) != 0);
    UART0_DR_R = ByteToSend;
}







void EcodersInit(void){ // enalbe PortF1 and port C7
		volatile uint32_t delay;
	SYSCTL_RCGCGPIO_R |= 0x00000024;  // 1) activate clock for Port F and C
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
	GPIO_PORTC_DEN_R |= 0x80;          //Enable PC.6,7 input
	GPIO_PORTC_DIR_R &= ~0x80;          //	PC.6,7  input
	GPIO_PORTF_DEN_R |= 0x02;          //Enable PF. 0,1 input
	//GPIO_PORTF_AFSEL_R &= ~0x03;          //  input
	
	
		// configure PORTC6-7 
	GPIO_PORTC_IS_R &= ~0x80;				// make PC.6-7 edge sensitive
	GPIO_PORTC_IBE_R &= ~0x80;		// trigger on both edges
	GPIO_PORTC_ICR_R |= 0x80;				// clear any prior interrupt
	GPIO_PORTC_IM_R |= 0x80;				// unmask interrupt
	//NVIC_PRI2_R = 3 << 5;					// set interrupt priority to 3
	NVIC_EN0_R |= 0x00000004;					// enable IRQ2 bit 2 of EN0
	
	// configure PF1 
	GPIO_PORTF_IS_R &= ~0x02;				// make PF0,1 edge sensitive
	GPIO_PORTF_IBE_R  &= ~0x02;			// Not trigger on both edges
	GPIO_PORTF_IEV_R &= ~0x02;      /* falling edge trigger */
	GPIO_PORTF_ICR_R |= 0x02;				// clear any prior interrupt
	GPIO_PORTF_IM_R |= 0x02;				// unmask interrupt
	//NVIC_PRI30_R = 3 << 5;					// set interrupt priority to 3
	NVIC_EN0_R |= 0x40000000;				// enable IRQ30 bit 30 of EN0	----- page 105

}
												




/////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// 		ULTRASONIC SENSORS  INIT 	//////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

							//	OUT, IN
void PortD_Init_WInterrupt(){ // PD2, 3    (TRIG, ECHO)  // INTERRUPT FOR PD3
volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= 0x00000008;  // 1) activate clock for Port D
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTD_AMSEL_R |= 0x0C;        // 3) disable analog on PF
  GPIO_PORTD_PCTL_R &= ~0x0000FF00;   // 4) PCTL GPIO on PD2,3
  GPIO_PORTD_DIR_R &= ~0x08;          // 5) PD3 in ( 0 input )
  GPIO_PORTD_DIR_R |= 0x04;          // 5) PD2 in ( 1 output )
  GPIO_PORTD_AFSEL_R &= ~0x0C;        // 6) disable alt funct on PD2,3
  GPIO_PORTD_DEN_R |= 0x0C;          // 7) enable digital

  /////PORT D INTERRUPT ENABLE 
	GPIO_PORTD_IS_R &= ~0x08;			// make PD3 edge sensitive
	GPIO_PORTD_IBE_R |= 0x08;			// trigger is controlled by IEV
	GPIO_PORTD_IEV_R &= ~0x08;			// falling edge trigger
	GPIO_PORTD_ICR_R |= 0x08;				// clear any prior interrupt
	GPIO_PORTD_IM_R |= 0x08;				// unmask interrupt
	NVIC_EN0_R |= 0x00000008;					// enable Port D interrupt 
}




/* 
		FOR DEBUGGIN PORPUSES, USE A OSILOSCOPE 
		TO MEASURE WHEN PINS GO HIGH OR LOW 
*/
void PortA_Init(){ // PA6,7 
volatile uint32_t delay;
	SYSCTL_RCGCGPIO_R |= 0x00000001;  // 1) activate clock for Port D
	delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
	GPIO_PORTA_AMSEL_R |= 0xC0;        // 3) disable analog on PA
	GPIO_PORTA_PCTL_R &= ~0xFF000000;   // 4) PCTL GPIO on PD2,3
	GPIO_PORTA_DIR_R |= 0xC0;          // 5) PA6,7 in ( 1 output )
	GPIO_PORTA_AFSEL_R &= ~0xC0;        // 6) disable alt funct on PD2,3
	GPIO_PORTA_DEN_R |= 0xC0;          // 7) enable digital

	/*
  /////PORT D INTERRUPT ENABLE 
	GPIO_PORTA_IS_R &= ~0x80;			// make PD3 edge sensitive
	GPIO_PORTA_IBE_R |= 0x80;			// trigger is controlled by IEV
	GPIO_PORTA_IEV_R &= ~0x80;			// falling edge trigger
	GPIO_PORTA_ICR_R |= 0x80;				// clear any prior interrupt
	GPIO_PORTA_IM_R |= 0x80;				// unmask interrupt
	NVIC_EN0_R |= 0x00000001;					// enable Port A interrupt 
	*/

}



//trig   output     	PB2  RIGHT SENSOR
// echo   interupt 		PB3  
void PortnB_Init(void){ // PORT B INTI
	volatile uint32_t delay;
  	SYSCTL_RCGCGPIO_R |= 0x00000002;  // 1) activate clock for  B
 	delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
	
	GPIO_PORTB_DEN_R |= 0x0C;          //ENBLE PB2, 3
	GPIO_PORTB_DIR_R |= 0x04;          	//PB2 OUTPUT (1 = OUTPUT)
	GPIO_PORTB_DIR_R &= ~0x08;			//PB3 INPUT ( 0 = INPUT)


}

void PortF_Init_WInterrupt(){ // PF 3, 4   (TRIG_OUT 3, ECHO_IN 4)  LEFT SENSOR
volatile uint32_t delay;
  SYSCTL_RCGCGPIO_R |= 0x00000020;  // 1) activate clock for Port F
  delay = SYSCTL_RCGCGPIO_R;        // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R |= 0x18;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R &= 0x00FFF;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R &= ~0x10;          // 5) PF4 in ( 0 input )
  GPIO_PORTF_DIR_R |= 0x08;          // 5) PF3 in ( 1 output )
  GPIO_PORTF_AFSEL_R &= ~0x18;        // 6) disable alt funct on PF7-0
 // GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R |= 0x18;          // 7) enable digital

  /////PORT F INTERRUPT ENABLE 
	GPIO_PORTF_IS_R &= ~0x10;				// make PF4 edge sensitive
	GPIO_PORTF_IBE_R |= 0x10;			// trigger is controlled by IEV
	GPIO_PORTF_IEV_R &= ~0x10;			// falling edge trigger
		
	GPIO_PORTF_ICR_R |= 0x10;				// clear any prior interrupt
	GPIO_PORTF_IM_R |= 0x10;				// unmask interrupt
	

	// enable interrupt in NVIC and set priority to 3
	NVIC_PRI30_R = 3 << 5;					// set interrupt priority to 3
	NVIC_EN0_R |= 0x40000000;				// enable IRQ30 (bit 30 of EN0)
	//NVIC_EN0_R |= 0x00000020;					// enable Port F interrupt 

}

void PortB_interrupt(){//making nedge PB3
	// configure PORTC7-0 for falling edge trigger interrupt
	GPIO_PORTB_IS_R &= ~0x08;				// make PB3 edge sensitive
	GPIO_PORTB_IBE_R |= 0x08;			// trigger is controlled by IEV
	GPIO_PORTB_IEV_R &= ~0x08;			// falling edge trigger
		
	GPIO_PORTB_ICR_R |= 0x08;				// clear any prior interrupt
	GPIO_PORTB_IM_R |= 0x08;				// unmask interrupt
	

	NVIC_PRI1_R = 5 << 5;				// set interrupt priority to 5
	NVIC_EN0_R |= 0x00000002;					// enable Port B interrupt 
	
}


void timer5b(){ // 
	SYSCTL_RCGCTIMER_R |= 0x20;		// enable Timer Block 5
	
	TIMER5_CTL_R = 0;				// disable Timer before init
	TIMER5_CFG_R = 0x04;			// 16-bit mode
	TIMER5_TBMR_R = 0x02;			// periodic countdown mode

	TIMER5_TBPR_R = 640;				// 80MHz/1K = 80K
	TIMER5_TBILR_R = 6400;				// 64kHz/6400 = 5H    			The period now is 200 ms
	
	TIMER5_ICR_R = 0x100;				// clear Timerb timeout flag
	TIMER5_IMR_R |= 0x100;				// enable Timer5b timeout interrupt ds.p745
	TIMER5_CTL_R = 0x100;			// enable timer 5b
	
	NVIC_EN2_R |= 0x20000000;		// enable IRQ93 pg.104    93 BIT
}

	
void timer1A (){  
	
	// Timer1A configuration
	SYSCTL_RCGCTIMER_R |= 0x02;	// enable clock to timer Block 1
	TIMER1_CTL_R = 0;						// disable Timer1 during configuration
	TIMER1_CFG_R = 0x04;				// 16-bit timer
	TIMER1_TAMR_R = 0x01;				// one-shot mode
	TIMER1_TAPR_R = 2*5;				// 16MHz/2 = 8MHz
	TIMER1_TAILR_R = 80;			// 8MHz/80 = 100kHz 				The period now is 10 MicroSecons
	
	TIMER1_ICR_R = 0x1;					// clear Timer1A timeout flag
	TIMER1_IMR_R |= 0x01;				// enable Timer1A timeout interrupt 
	TIMER1_CTL_R |= 0x01;				// enable Timer1A
	NVIC_EN0_R |= 0x00200000;		// enable IRQ21
}


/*
 TIMER TO CHECK HOW LONG THE SIGNAL TOOK TO TRAVEL 
*/
void timer2A(){ //count Ms that has passed
								// Timer2A configuration
	SYSCTL_RCGCTIMER_R |= 0x04;			// enable clock to Timer Block 2
	TIMER2_CTL_R = 0;					// disable Tier2 during configuration
	TIMER2_CFG_R = 0x04;				// 16-bit timer
	TIMER2_TAMR_R = 0x02;				// periodic countdown mode
	TIMER2_TAPR_R = 10;					// 80MHz/10 = 8MHz
	TIMER2_TAILR_R = 80;				// 8MHz/80 = 100kHz    			The period now is 10 MicroSecons
	
	TIMER2_ICR_R = 0x01;				// clear Timer2A timeout flag
	TIMER2_IMR_R |= 0x01;				// enable Timer2A timeout interrupt ds.p745
//	TIMER2_CTL_R |= 0x01;				// enable Timer2A
	NVIC_EN0_R |= 0x00800000;			// enable IRQ23
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// UART ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void timer0A_delayMs(uint16_t delay){ //one-shot timer // NOTE  
	SYSCTL_RCGCTIMER_R |= 1;		// enable Timer Block 0
	TIMER0_CTL_R = 0;				// disable Timer before init
	TIMER0_CFG_R = 0x04;			// 16-bit mode
	TIMER0_TAMR_R = 0x01;			// one-shot mode, down-counter
	
	TIMER0_TAPR_R = 20000;			// predivide
	TIMER0_TAILR_R = delay * 8000 - 1;	// interval load value register
	
	TIMER0_ICR_R = 0x1;				// clear TimerA timeout flag
	TIMER0_CTL_R = 0x01;			// enable timer A
	while((TIMER0_RIS_R & 0x01) == 0) ;	// wait for timeout
}


	// Configure SysTick  SysTick interrupt @ 1Hz
void SysTick_Init_Interrupt(){
		NVIC_ST_CTRL_R = 0;					// disable SysTick during setup
		NVIC_ST_RELOAD_R = 16000000-1;	// # of clocks per second
		NVIC_ST_CTRL_R  = 0x6;				//  CLK_SRC, INTEN, ENABLE    110 = 6

}
	
	void SysTick_EnDis(uint8_t bit1or0){
		
		switch( bit1or0){
			case 1: 	NVIC_ST_RELOAD_R = 16000000-1; NVIC_ST_CTRL_R |= 0x01;  break; // ENABLE TIMER
			case 0: 	NVIC_ST_CTRL_R &= ~0x01; break; // DISABLE TIMER
		}
		
	}
	
	
	
	
void timer0b(){ //works
		
	SYSCTL_RCGCTIMER_R |= 0x1;		// enable Timer Block 0
	TIMER0_CTL_R = 0;				// disable Timer B before init
	TIMER0_CFG_R = 0x04;			// 16-bit mode
	TIMER0_TBMR_R = 0x01;			// one-shot mode, down-counter
	TIMER0_TBPR_R = 124;				// 80MHz/10K = 8k
	TIMER0_TBILR_R = 8000;				// 8kHz/8k = 1z    			The period now is 200 ms

	TIMER0_ICR_R |= 0x100;				// clear Timerb timeout flag
	TIMER0_IMR_R |= 0x100;				// enable Timer0b timeout interrupt ds.p745
	TIMER0_CTL_R |= 0x100;			// enable timer b
	NVIC_EN0_R =  1 << 20; 			// ENABLE TIMER0B INTERRUPT FLAG
}


void timer3a_64bit(){ // DOESNT WORK
		
	SYSCTL_RCGCWTIMER_R |= 0x8;		// enable Timer Block 3  
	TIMER3_CTL_R = 0;				// Register is cleared before any modifications
	TIMER3_CFG_R = 0x04;			// 32-bit mode
	TIMER3_TAMR_R = 0x02;			// one-shot mode, down-counter
	
	TIMER3_TAPR_R = 64000 - 1;				// 80MHz/10K = 8k
	TIMER3_TAILR_R = 65400 - 1;	// interval load value register

	TIMER3_ICR_R = 0x1;				// clear TimerA timeout flag

	TIMER3_ICR_R = 0x01;				// clear Timer3A timeout flag
	TIMER3_IMR_R |= 0x01;				// enable Timer3A timeout interrupt ds.p745
	TIMER3_CTL_R |= 0x01;				// enable Timer3A
	NVIC_EN3_R |= 0x00000010;			// enable IRQ100
}




/* 
16 BIT TIMER TO CHECK WHERE THE CAR IS AND TO KEEP TRACK OF WHAT IS DOING
*/
void timer3a_16bit(){  //PERIOD OF 209 
		
	SYSCTL_RCGCTIMER_R |= 0x8;		// enable Timer Block 3  
	TIMER3_CTL_R = 0;				// Register is cleared before any modifications
	TIMER3_CFG_R = 0x04;			// 32-bit mode
	TIMER3_TAMR_R = 0x02;			// one-shot mode, down-counter
	
	TIMER3_TAILR_R = 65400 - 1;	// 400Mhz/65k = 6153		interval load value register
	TIMER3_TAPR_R = 32000 - 1;	// 61k/32k = 2hz  		but for some reason the period is 209(we can work with that)

	TIMER3_ICR_R = 0x1;				// clear TimerA timeout flag

	TIMER3_ICR_R = 0x01;				// clear Timer3A timeout flag
	TIMER3_IMR_R |= 0x01;				// enable Timer3A timeout interrupt ds.p745
	TIMER3_CTL_R |= 0x01;				// enable Timer3A
	NVIC_EN1_R |= 0x00000008;			// enable IRQ35
}




/*
THIS FUNTINO WILL SET ALL VARIABLES FOR UART FOR 
COMPUTER COMMUNICATOIN TO ZERO
*/
void setAll2Zero(){
	
	UART0D_TEMP1=0; UART0D_TEMP2 = 0;
	UART0D_TEMP3= 0; UART0D_TEMP4 = 0; UART0D_TEMP5 = 0; 
	UART0D_TEMP6=0; UART0D_TEMP7=0; UART0D_TEMP8=0; 
	UART0D_TEMP9=0; UART0D_TEMP10=0;

}

// Cris


uint32_t k;

void PWM0A_Init( uint32_t duty){   //////////// PB6
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
	SYSCTL_RCC_R |= 0x001E0000;	// pre-divide for PWM clock, 250KHZ - 4uSec T

  while((SYSCTL_PRGPIO_R&0x02) == 0){}; //delay

  /////////////////////////port B INIT
  GPIO_PORTB_AFSEL_R |= 0xF0;           // enable alt funct on PB6
  GPIO_PORTB_PCTL_R &= ~0xFFFF0000;     // configure PB4-7 as PWM0
  GPIO_PORTB_PCTL_R |= 0x44440000;
  GPIO_PORTB_AMSEL_R &= ~0xF0;          // disable analog functionality on PB6
  GPIO_PORTB_DEN_R |= 0xF0;             // enable digital I/O on PB6
  //////////////////////// PWM CONFIG

		
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_0_LOAD_R = 25000 - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPA_R = duty - 1;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000001;          // enable PB6/M0PWM0
}

void Lower_1st_PB6(uint16_t degree){ /// compare for PB6
	uint16_t temp; 

	temp = degree ;//(1.1167*degree + 145) -1;

  PWM0_0_CMPA_R = temp*5-1;

}




void PWM0B_Init( uint16_t duty){ ///////////////// PB7
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
	SYSCTL_RCC_R |= 0x001E0000;	// pre-divide for PWM clock, 250KHZ - 4uSec T
  delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating


  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENB_R = 0xC08;
  // PB7 goes low on LOAD
  // PB7 goes high on CMPB down
  PWM0_0_LOAD_R = 25000 - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPB_R = duty - 1;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000002;          // enable PB7/M0PWM1
}
void SecLower_2nd_PB7(uint16_t degree){ /// compare for PB7
	uint16_t temp; 

	temp = degree		;				//(1.5278*degree + 145) -1;

  PWM0_0_CMPB_R = (temp*5) -1;             

}





void PWM1A_Init( uint16_t duty){ ////////// PB4
//                                                  pmw2  Gen1A
  ////port B and PWM is initalized 

	SYSCTL_RCC_R |= 0x001E0000;	// pre-divide for PWM clock, 250KHZ - 4uSec T

  PWM0_1_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_1_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_1_LOAD_R = 25000 - 1;           // 5) cycles needed to count down to 0
  PWM0_1_CMPA_R = duty - 1;             // 6) count value when output rises
  PWM0_1_CTL_R |= 0x00000001;           // 7) start PWM0
	PWM0_ENABLE_R |= 0x00000004;          // enable PB6/M0PWM0

}
void ThirdLower_3rd_PB4(uint8_t degree){ /// compare for PB4 
  uint16_t temp; 
	
  temp = (degree*2.5778 + 145);

		PWM0_1_CMPA_R = temp*5 -1;
}






void PWM1B_Init( uint16_t duty){// PB5
                                                // pwm3;  gen1B
  ////port B and PWM is initalized 

	SYSCTL_RCC_R |= 0x001E0000;	// pre-divide for PWM clock, 250KHZ - 4uSec T
  PWM0_1_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_1_GENB_R = 0xC08;
  // PB7 goes low on LOAD
  // PB7 goes high on CMPB down
  PWM0_1_LOAD_R = 25000 - 1;           // 5) cycles needed to count down to 0
  PWM0_1_CMPB_R = duty - 1;             // 6) count value when output rises
  PWM0_1_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000008;          // enable PB7/M0PWM3
}
void FourthLower_4th_PB5(uint8_t degree){ /// compare for PB5 
  uint16_t temp; 
	
  temp = (degree*2.5778 + 150);

		PWM0_1_CMPB_R = temp*5 - 1;
}








void PWM3A_Init( uint16_t duty){ ////////// PC4           ROTATE_PINCHER
//                                                  M0PWM6  Gen3A
  ////port c and PWM is initalized 
	SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
	SYSCTL_RCGCGPIO_R |= 0x04;            // 2) activate port C

	volatile unsigned long delay;
  ////////////// enalbing pc4,5
  delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating
	
	////////////////PORT C INIT
  GPIO_PORTC_AFSEL_R |= 0x30;           // enable alt funct on Pc4,5
  GPIO_PORTC_PCTL_R &= ~0x00FF0000;     // configure Pc4,5 as M0PWM6,7
  GPIO_PORTC_PCTL_R |= 0x00440000;
  GPIO_PORTC_AMSEL_R &= ~0x30;          // disable analog functionality on Pc4,5
  GPIO_PORTC_DEN_R |= 0x30;             // enable digital I/O on Pc4,5
	//////////// PORT C INIT



	SYSCTL_RCC_R |= 0x001E0000;	// pre-divide for PWM clock, 250KHZ - 4uSec T

  PWM0_3_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_3_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_3_LOAD_R = 25000-1;           // 5) cycles needed to count down to 0
  PWM0_3_CMPA_R = duty - 1;             // 6) count value when output rises
  PWM0_3_CTL_R |= 0x00000001;           // 7) start PWM0
	PWM0_ENABLE_R |= 0x0000040;          // M0PWM6

}
void ROTATE_PINCHER_PC4(uint8_t degree){ /// compare for PC4

    if (degree >= 90 && degree <= 180){
        uint8_t temp; 
        temp = (degree*3.3333333333 -1) ; //Degress into pulses 
        PWM0_3_CMPA_R = temp*5 - 1;
    }

}








void PWM3B_Init( uint16_t duty){ ////////// PC5
//                                                  pmw7 Gen3B
  ////port B and PWM is initalized 

	SYSCTL_RCC_R |= 0x001E0000;	// pre-divide for PWM clock, 250KHZ - 4uSec T
	SYSCTL_RCC_R |= 0x001E0000;	// pre-divide for PWM clock, 250KHZ - 4uSec T
  PWM0_3_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_3_GENB_R = 0xC08;
  // PB7 goes low on LOAD
  // PB7 goes high on CMPB down
  PWM0_3_LOAD_R = 25000 - 1;            // 5) cycles needed to count down to 0
  PWM0_3_CMPB_R = duty - 1;             // 6) count value when output rises
  PWM0_3_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R |= 0x00000080;          // enable PB7/M0PWM3

}
void CLOSE_OPEN_PINCHER_PC5(uint8_t OPENnCLOSE_NUMBER){ /// compare for PC5
  // OPEN = 10
  // CLOSE = 0
	uint16_t temp; 

	temp = (OPENnCLOSE_NUMBER*30 + 400 - 1) ; //Degrees into pulses 

  PWM0_3_CMPB_R = (temp*5) - 1;             

}




/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// 	BLCKING DELAY  ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
void delay (uint32_t delayMs)
{
	uint32_t j;
	for( k = 0; k < delayMs; k++) 
		{
		for(j=0; j < 318; j++);
		}
}
















































