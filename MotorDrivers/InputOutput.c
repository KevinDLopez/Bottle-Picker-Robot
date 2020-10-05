
/*///////////////////////////////////////////////////////////////////////////////////////////
 _______  _______  _        _       _________ _        _______      _______  _______  _______ 
(  ____ )(  ___  )( \      ( \      \__   __/( (    /|(  ____ \    (  ___  )(  ____ )(       )
| (    )|| (   ) || (      | (         ) (   |  \  ( || (    \/    | (   ) || (    )|| () () |
| (____)|| |   | || |      | |         | |   |   \ | || |          | (___) || (____)|| || || |
|     __)| |   | || |      | |         | |   | (\ \) || | ____     |  ___  ||     __)| |(_)| |
| (\ (   | |   | || |      | |         | |   | | \   || | \_  )    | (   ) || (\ (   | |   | |
| ) \ \__| (___) || (____/\| (____/\___) (___| )  \  || (___) |    | )   ( || ) \ \__| )   ( |
|/   \__/(_______)(_______/(_______/\_______/|/    )_)(_______)    |/     \||/   \__/|/     \|
KEVIN LOPEZ 
CRISTIAN LOPEZ 
CECS 490B 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
THIS IS THE FIRMWARE FOR  A ROBOT THAT WILL PICK UP CANS AUTOMOUSLY AND WILL DO SO
USING A TM4C MICROCONTROLLER AND A SINNGLE BOARD COMPUTER. THE SINGLLE BOARD COMPUTER OF 
CHOICE IS INTEL UPBOARD BECAUSE OF THE ABILITY OF USING INTELREALSENSE CAMERAS AND USE DEPTH AND 
IR PICUTRES TO MEASURE DISTANCE AND SHAPE OF OBJECTS. 
THIS PROJECT IS MAINLY COMPOSED OF A ROBOTIC ARM, A INTELREALSENSE CAMERA, 3 IR MODULES AND 2 DC ENCODERS MODLUES. 
THIS PROJECT OS COMPOSED OF 4 MODLUES. 
INIT FILE = INITIALIZATION FOR ALL REGISTERS AND ABILITIES 
ENCODER FILE = TRACKS THE STEPS OF THE ENCODERS AND ADDS FUNTIONALITY TO CONTROL THE CAR
ULTRASONIC FILE = TRACS THE DISTNACE BETWEEN LEFT, RIGHT AND FRONT( CONTAINS INTERRUPT FOR PORTS AND TIMERS)
InputOutput FILE = HAS THE CORE OF THE PROJECT- EVERY SINGLE MAIN FUNCTION HAPPENS HERE AND EVERYTHIGN IS INITIALIZED HERE
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
																																															
*/


#include "ArmControl.c"


#define SetPoint4Encoders 800
//#define uMperStep					890   //micro Meters
#define STEPS4METER 			4300		// STEPS TO TRAVEL 1 METTER
#define STEPS4HALF_METER  2150
#define ASCII_COMMA				44
#define ASCII_CR					13 

	
	void PID4WHEELS_N_TURNS(void);
	void newQuadrant(void);
	void PID2CAN(void);

int32_t main(void){
		 PLL_Init();	 	// 80Mhz

		///////// 	Encoders 	//////////
		IPWM0AnB_Init(1);
		EcodersInit(); 	// 
		UART0_INIT(); 	//INTERRUPT  8-bit,  ODD parity, 1-stop ( DATA COMMING IN FROM CAMERA).
		UART7_INIT();		// 
		SysTick_Init_Interrupt();
		
		timer3a_16bit(); // 
		PortA_Init(); // USED FOR DEBUGGIN PORPUSES  

		///////	 	UltraSonicSensor ////////
		PortnB_Init();
		PortB_interrupt(); 			 // RIGHT SENSOR
		PortF_Init_WInterrupt(); // FRONT SENSOR
		PortD_Init_WInterrupt(); // FRONT SENSOR
		timer1A (); 						 //	10Ms 
		timer2A();							 //	10Ms  TO COUNT FOR TIME OF TRAVEL AND RESET 
		///////   ARM SERVOS///////////////
	PWM0A_Init    (1925); // 50hz 5% duty cycle 20ms, 1ms pulse      		PB6 	=  BOTTOM SERVO  //original = 385 
  PWM0B_Init 		(1250); // 50hz 5% duty cycle													PB7 	= 2ND LOWEST     //original = 250
  PWM1A_Init  	(500); // 50hz 5% duty cycle   									 		 	PB4  	= 3RD LOWEST     //original = 100
  PWM1B_Init 		(3000); // 50hz 5% duty cycle													PB5 	= 4TH LOWERST 	 //original = 600
	
	///////////////////////////HS-311 ///////////////////////////////////////////////////////////
  PWM3A_Init 		(450); // 50hz 5% duty cycle													PC4		= PICHER ROTATE  //original = 90
  PWM3B_Init 		(2500); // 50hz 5% duty cycle													PC5  	= CLOSE PINCEHR  //original = 500
	//delay(100000);
		
		PAST_LOCATIONS[xCarPosition][yCarPosition] = TRAVELED;   // TRAVEL_LOCATIONS{TRAVELED = 1, OBJECT = 5};
		
		GPIO_PORTB_DATA_R |= 0x04; // TURN ON TRIG PB2
		GPIO_PORTF_DATA_R |= 0x08; // TURN ON TRIG PF3
		GPIO_PORTD_DATA_R |= 0x04; // TURN ON TRIG PD2

		
		// USE UART 1 TO CHECK FOR DATA COMING IN FORM CAMARA 
		UART7_Transmit_String( FIRST_STATE);
		setState(INITIAL);
			//putOnBox();
		while(1){

			PID4WHEELS_N_TURNS();


		} // while
		
		
}



/*
		TO CHECK WHERE THE CAR IS THIS IS DONE EVERY 200 milliseconds
*/
void Timer3A_Handler(){  
	//	uint32_t readback;
		//GPIO_PORTA_DATA_R ^= 0xC0;  // TO check that pin is actual .1 second
		//PID4WHEELS_N_TURNS();
		newQuadrant();

	
		TIMER3_ICR_R = 0x01;				// clear Timer1A timeout flag
		//readback = TIMER3_ICR_R;		// force interrupt flag clear
		
}


/**
COMMUNICATION BETWEEN SBC AND TM4C. 
*/
void UART0_Handler(void){ // USB UART/ PA0,1
	//	volatile int32_t readback;
		//if(UART0_MIS_R & 0x0010){	// UARTMIS ds.p930
			UART0_DATA = UART0_DR_R;
			
			//PROTOCAL TO CHECK THAT DATA IS CORRECT
			//Also timer is running so we could reset data if data is not sent continuously 
			
			switch(UART0_STATE){
				case 0: 	if(UART0_DATA == 'A'){ UART0_STATE++; SysTick_EnDis(1); } break; 															//RECEIVE START BIT 
				case 1:  	SysTick_EnDis(0); UART0D_TEMP1 = UART0_DATA - '0'; UART0_STATE++; SysTick_EnDis(1); break;		//FIRST  BIT, MSB
				case 2:	 	SysTick_EnDis(0); UART0D_TEMP2 = UART0_DATA - '0'; UART0_STATE++; SysTick_EnDis(1); break; 		//FIRST  BIT
				case 3:	 	SysTick_EnDis(0); UART0D_TEMP3 = UART0_DATA - '0'; UART0_STATE++; SysTick_EnDis(1); 					//FIRST  BIT, LSB
									TEMPX = 100*UART0D_TEMP1 + 10*UART0D_TEMP2 + UART0D_TEMP3;											break; 				//CONVERRT BITS TO DECIMAL
				
				case 4:	 	SysTick_EnDis(0); UART0D_TEMP4 = UART0_DATA - '0'; UART0_STATE++; SysTick_EnDis(1); break; 		// SECOND BIT, MSB
				case 5:	 	SysTick_EnDis(0); UART0D_TEMP5 = UART0_DATA - '0'; UART0_STATE++; SysTick_EnDis(1); break; 		// SECOND BIT, 
				case 6:	 	SysTick_EnDis(0); UART0D_TEMP6 = UART0_DATA - '0'; UART0_STATE++; SysTick_EnDis(1); 				 	// SECOND BIT, LSB
									TEMPY = 100*UART0D_TEMP4 + 10*UART0D_TEMP5 + UART0D_TEMP6;											break; 				//CONVERRT BITS TO DECIMAL 

				case 7:	 	SysTick_EnDis(0); UART0D_TEMP7 = UART0_DATA - '0'; UART0_STATE++; SysTick_EnDis(1); break; 		// CHECK SUM, MSB
				case 8:	 	SysTick_EnDis(0); UART0D_TEMP8 = UART0_DATA - '0'; UART0_STATE++; SysTick_EnDis(1); break; 		// CHECK SUM
				case 9: 	SysTick_EnDis(0); UART0D_TEMP9 = UART0_DATA - '0';	UART0_STATE++; SysTick_EnDis(1); break;	  // CHECK SUM
				case 10:  SysTick_EnDis(0); UART0D_TEMP10 = UART0_DATA - '0';																						// CHECK SUM, LSB
									UART0_CHECK_SUM = 1000*UART0D_TEMP7 + 100*UART0D_TEMP8 + 10*UART0D_TEMP9 +  UART0D_TEMP10;					 								//CONVERRT BITS TO DECIMAL
				
									if( TEMPX + TEMPY == UART0_CHECK_SUM ){	//DATA IS CORRECT   // PERFORM CHECK SUM, IF CORRECT PASS DATA TO VARIBLES/FUNCT 
										X_POSITION = TEMPX; 
										Y_POSITION = TEMPY;
										UART0_STATE = 0;		// RESET UART TO RECEIVE ANOTHER SET OF CORDINATES
										UART0_TX('5'); /// HANDSHAKE (SENDING DATA TO CONFIRM THAT DATA HAS RECEIVED CORRECTLY)
										
										PID2CAN();
										//setState(GRAVING_CAN); 

									}else{// DATA IS INCORRECT 
										UART0_STATE = 0;				
								}
								setAll2Zero(); // SETS ALL THE UART0D TO ZERO TO NOT GET ERRORS AGAIN
									break;	
			}//switch
			
			UART0_ICR_R = 0x0010;
		//	readback = UART0_ICR_R;
	//	}
	//	else{
			UART0_ICR_R = UART0_MIS_R;	// clear all interrutp flags
		//	readback = UART0_ICR_R;			// force flags clear
	//	}	
}


/**
	TO RESET VALUES FORM THE UART PROTOCOL 
*/
void SysTick_Handler(void){
	 UART0_STATE = 0; // DATA HAS NOT BEEN SENT CONTINOUSLY
}




/*//////////////////////////////////////////////////////////
  _____  _____  _____      _____       _____            _   _ 
 |  __ \|_   _||  __ \    |  __ \     / ____|    /\    | \ | |
 | |__) | | |  | |  | |   | |  | |   | |        /  \   |  \| |
 |  ___/  | |  | |  | |   | |  | |   | |       / /\ \  | . ` |
 | |     _| |_ | |__| |   | |__| |   | |____  / ____ \ | |\  |
 |_|    |_____||_____/    |_____/     \_____|/_/    \_\|_| \_|                                                                                                                          
*////////////////////////////////////////////////////////////

/** We will get the pixels from the UART0 and we will try to get the robot to drive there 
We will use the camera to check the direction of the car and the 
UltraSonic to check how far the can is, since the camera it self hasn't been reliable 
*/
#define  	SetPoint4CanX   		320   // full resolution /2 
#define 	Pi  								0.7         //K constant for P
#define 	DESIRED_DISTANCE2CAN	22
int16_t ErrCan; // Distance from the setPoint and Can
int16_t LastErr;
int16_t RWtempPV, LWtempPV, PV;
int16_t PORTION;
int32_t INTEGRAL, DERIVATIVE;
float   LEFT_PORTION, RIGHT_PORTION;
uint8_t MOVING_LEFT, MOVING_RIGHT, CAN_CENTERED;
	int PWM_RIGHT_WHEEL , PWM_LEFT_WHEEL ;

//float 	PORTION_DISTANCE;
//PWM1_0_CMPA_R // RIGHT wheel 
//PWM1_0_CMPB_R // LEFT wheel 

/// WE ARE GOING TO CHANGE PWM DEPENDING ON CAMERA X POSITION 
void PID2CAN(){
	setState( TRACKING_CAN);
	PWM_RightWheel_READ = PWM1_0_CMPA_R;
	PWM_leftWheel_READ = PWM1_0_CMPB_R; //READ 
	
	PWM_LEFT_WHEEL = 650;
	PWM_RIGHT_WHEEL = 660; 
	
	PWM_RIGHT_WHEEL = PWM1_0_CMPA_R; PWM_LEFT_WHEEL = PWM1_0_CMPB_R;
	MOVING_LEFT = 0;
	MOVING_RIGHT = 0; 
  if( FrontSensoDistance > 8 ) { // this will only run when distance is > 8
	//do{
		ErrCan =  X_POSITION - SetPoint4CanX ;  // IF ITS 	NEGATIVE IS TO THE LEFT    	Decrease LEFT wheel 		increase RIGHT wheel 
												// 			POSITIVE IS TO THE RIGHT 	Decrease RIGHT wheel 		increase LEFT wheel
		PORTION = (short)((abs(ErrCan)*100*Pi)/ SetPoint4CanX); // Percentage of Error
		INTEGRAL += ErrCan;
	//	if( INTEGRAL >= 0xFFFFFFFE) { INTEGRAL = 0; } 
		//								(condition)?  ifTrue: ifFalse;
		//PORTION_DISTANCE = (FrontSensoDistance  > 100)?  30  :  (-0.5185*FrontSensoDistance + 107.8)  ;   /// it will move a lot more if can is next to it 
		//PORTION_DISTANCE = 	(PORTION_DISTANCE/100); //MAKING IT BETWEEN A NUMBER OF 0-1
		//PORTION_DISTANCE = ( PORTION_DISTANCE < 0) ? 0: PORTION_DISTANCE; //getting ride of negatives 
		LEFT_PORTION =  ( (PORTION) * (0.005) );
		RIGHT_PORTION = ( (PORTION) * (0.005) );
    
		//if (X_POSITION > 315  && X_POSITION < 325 ){
		//			PWM_LEFT_WHEEL = 800;
		//			PWM_RIGHT_WHEEL = 800;

		//}else{
			if( ErrCan < 0) { // if negative it means is to the left 
					MOVING_LEFT = 1; CAN_CENTERED = 0;

		 			PWM_LEFT_WHEEL -= (int)(PWM_LEFT_WHEEL*LEFT_PORTION*0.8); // take away this percentage 	LEFT
					PWM_RIGHT_WHEEL += (int)(PWM_RIGHT_WHEEL*RIGHT_PORTION); // add this percentage 		RIGHT

			} else if (ErrCan > 0) { // if positive it means is to the right 
					MOVING_RIGHT = 1; CAN_CENTERED = 0;

					PWM_LEFT_WHEEL += (int)(PWM_LEFT_WHEEL*LEFT_PORTION*1.4); // take away this percentage 
					PWM_RIGHT_WHEEL -= (int)(PWM_RIGHT_WHEEL*RIGHT_PORTION); // add this percentage 

			} //End of If statements
		//}
		if( PWM_LEFT_WHEEL < 300 ){ PWM_LEFT_WHEEL = 300;}
		if ( PWM_RIGHT_WHEEL < 300){PWM_RIGHT_WHEEL = 300;}

		if( PWM_LEFT_WHEEL >=1248 ){ PWM_LEFT_WHEEL = 1240;}
		if ( PWM_RIGHT_WHEEL >=1248){PWM_RIGHT_WHEEL = 1240;}

		PWM1_0_CMPA_R = PWM_RIGHT_WHEEL;
		PWM1_0_CMPB_R = PWM_LEFT_WHEEL;
		
		PWM_RightWheel_READ = PWM1_0_CMPA_R;
	  PWM_leftWheel_READ = PWM1_0_CMPB_R;
		LastErr = ErrCan;
		//also check what the front sensor read if its ever <20 stop 
		if( FrontSensoDistance < DESIRED_DISTANCE2CAN && abs(PORTION) < 20) {
			setState(GRAVING_CAN); //STOPS AND GRAVES CAN
		} else{
			// the car is not between 15 percent  of portion  // maybe move back and do it again
			
		}
	} // IF FORNTSENSOR DISTNACE > 8
		
	//}while( FrontSensoDistance > DESIRED_DISTANCE2CAN );

}// void PID















/**
	TO KEEP TRACK OF THE ENCODERS - LEFT ENCODER
*/
void GPIOPortC_Handler(void){ //Works
		volatile int readback;
		
	
			if( LeftEncoderState == FOWARD){
				leftEncoder++;
				leftEncoder2++;  // NOTE EVERY STEP = 890um 
				newQuadL++;
			}else{
				leftEncoder--;
				leftEncoder2--;  // NOTE EVERY STEP = 890um 
				newQuadL--;

			}// DIFFERENCE OF 600 ON ENCODERS MEANS THAT THE WHEELS TURNED
			
		GPIO_PORTC_ICR_R |= 0xC0;				// clear the interrupt flag
		readback = GPIO_PORTC_ICR_R;		// read to force interrupt flag clear
}




/**
	TO KEEEP TRACK FO THE ENCODERS  - RIGHT ENCODER
	AND CHECK THE LEFT SENSOR
*/
void GPIOPortF_Handler(void){
		volatile int readback;
		
		
	if (GPIO_PORTF_MIS_R & 0x02 ){ // Encoder interrupt
			
			if ( RightEncoderState == FOWARD){
				rightEncoder++;
				rightEncoder2++;  // NOTE EVERY STEP = 890um
				newQuadR++;

			} else { // ASSUMED IS BACKWARDS
				rightEncoder--;
				rightEncoder2--;  // NOTE EVERY STEP = 890um
				newQuadR--;

			}
	

	}else if ( GPIO_PORTF_MIS_R & 0x10){ // Ultrasonic Interrupt
				LeftSensorTime = iUltraSonic;
				if ( LeftSensorTime >= ULTRASONIC_MIN_WORKING_RANGE){  LeftSensorDistance = Distance(LeftSensorTime); }

				if ( (LeftSensorDistance <= 13) && (CAR_ACTION == STRAIGHT) && (LeftSensorDistance  > 2) ){// CAR GOT CLOSE TO WALL 
					setState(RIGHT45);
				}
				
				
	}
		
		GPIO_PORTF_ICR_R |= 0x12;				// clear the interrupt flag
		readback = GPIO_PORTF_ICR_R;		// read to force interrupt flag clear
}


	





uint8_t UART7_DATA;

/*
	UART TO RECEIVE DATA FORM SMARTPHONE
*/
void UART7_Handler(void){ // THIS UART IS USED TO RECEIVE DATA FORM HM-10  PE0, PE1
		volatile int32_t readback;
		
		//if(UART7_MIS_R & 0x0010){	// UARTMIS ds.p930
			UART7_DATA = UART7_DR_R;
			
switch( UART7_DATA){
                case 'L' : setState( LEFT);                 UART7_Tx( 'g' ); break;
                case 'R' : setState( RIGHT);                 UART7_Tx( 'g' ); break;
                case 'N' : setState( NO_MOVING);             UART7_Tx( 'g' ); break;
                case 'S' : setState( STRAIGHT );             UART7_Tx( 'g' ); break;
                case 'B' : setState( BACKWARDS);             UART7_Tx( 'g' ); break;
                case 'G' : setState( GRAVING_CAN);          UART7_Tx( 'g'); break;
                case 'r' : setState( RIGHT45);                UART7_Tx( 'g'); break;
                case 'l' : setState( LEFT45);                 UART7_Tx( 'g'); break;
                default: break;
           }

						UART7_ICR_R = 0x0010;
			readback = UART7_ICR_R;
		//}else{
			UART7_ICR_R = UART7_MIS_R;	// clear all interrupt flags
			readback = UART7_ICR_R;			// force flags clear
		//}	
}



































void newQuadrant(){ // Asuume it only travels foward 
		
		
				if ( newQuadR >= STEPS4HALF_METER && newQuadL >= STEPS4HALF_METER){
			
				switch( CAR_RELATED2PLANE ) {  //GOING TO BE SET BY THE DIFFERENCE OF STEPS 
					case UP: 		yCarPosition++; PAST_LOCATIONS[xCarPosition][yCarPosition] = TRAVELED;	break; // initial state    
					case DOWN: 	yCarPosition--; PAST_LOCATIONS[xCarPosition][yCarPosition] = TRAVELED;	break;//  HOW WE ARE GOING TO INCREASE OR DECREASE THE VARIBLES OF THE MAP
					case LEFT: 	xCarPosition--; PAST_LOCATIONS[xCarPosition][yCarPosition] = TRAVELED;	break;
					case RIGHT: xCarPosition++; PAST_LOCATIONS[xCarPosition][yCarPosition] = TRAVELED; 	break; 

					case RIGHT45:	


					break; // IF CAR TURNS 45 DEGREES TWICE = 90 DEGREE TURN

					case LEFT45: 	


					break;
				}// switch
			
				if ( newQuadR >= STEPS4HALF_METER ){ 	newQuadR = 0; }
				else if( newQuadL >= STEPS4HALF_METER) { newQuadL = 0; }

				
				
				// send data to uart everyNewPostiion
					///	void UART7_Tx(uint8_t ByteToSend){
		
			uint8_t MSBbyteToSend = xCarPosition/10; //msb
			uint8_t LSBbyteToSend = xCarPosition%10; //lsb
			UART7_Tx(MSBbyteToSend + 0x30);
			UART7_Tx( LSBbyteToSend + 0x30);
			UART7_Tx( ASCII_COMMA); 
			MSBbyteToSend = yCarPosition/10; // msb
			LSBbyteToSend = yCarPosition%10; //lsb
			UART7_Tx( MSBbyteToSend   + 0x30);
			UART7_Tx( LSBbyteToSend 	+ 0x30);
			UART7_Tx( ASCII_CR);
				
		}// IF NEW STEP

}


/*
  _____ _____ _____        _____ _______ _____           _____ _____ _    _ _______ 
 |  __ |_   _|  __ \      / ____|__   __|  __ \    /\   |_   _/ ____| |  | |__   __|
 | |__) || | | |  | |    | (___    | |  | |__) |  /  \    | || |  __| |__| |  | |   
 |  ___/ | | | |  | |     \___ \   | |  |  _  /  / /\ \   | || | |_ |  __  |  | |   
 | |    _| |_| |__| |     ____) |  | |  | | \ \ / ____ \ _| || |__| | |  | |  | |   
 |_|   |_____|_____/     |_____/   |_|  |_|  \_/_/    \_|_____\_____|_|  |_|  |_|                                                                                                                                                        
*/

int PWM_RIGHT_WRITE, PWM_LEFT_WRITE;

void PID4WHEELS_N_TURNS(){
	PWM_RIGHT_WRITE = PWM1_0_CMPA_R;
	PWM_LEFT_WRITE = PWM1_0_CMPB_R;
	
	// NEED TO SET CAR FOWARD AFTER WE GO BACK  
		switch(CAR_ACTION){
			case STRAIGHT:
					if (rightEncoder >= SetPoint4Encoders || leftEncoder >= SetPoint4Encoders){ // Check encoders every 300 steps 
							RightEncoderRatio = SetPoint4Encoders/rightEncoder;
							if (RightEncoderRatio != 0) { PWM_LEFT_WRITE *= RightEncoderRatio; }
							LeftEncoderRatio = SetPoint4Encoders/leftEncoder;	
							if (LeftEncoderRatio != 0) { PWM_RIGHT_WRITE *= LeftEncoderRatio; }
							rightEncoder = 0;	leftEncoder = 0;
					}   
					if( PWM_RIGHT_WRITE >= INITIAL_PWM_LOAD){ 	PWM_RIGHT_WRITE = INITIAL_PWM_LOAD -10; }
					if (PWM_LEFT_WRITE 	>= INITIAL_PWM_LOAD){	 	PWM_LEFT_WRITE = INITIAL_PWM_LOAD -10; 	}
					PWM1_0_CMPA_R = PWM_RIGHT_WRITE;
					PWM1_0_CMPB_R = PWM_LEFT_WRITE;
			break;
					

			case LEFT: 
					if( (rightEncoder - leftEncoder) >= 2130){ // 600 = NUMBER OF TURNS TO COMPLETE A TURN
						rightEncoder = 0;	leftEncoder = 0;
						setState(STRAIGHT);
					}
			
					break;


			case RIGHT:
					if ( (leftEncoder - rightEncoder >= 2130)){ // 600 = NUMBER OF TURNS TO COMPLETE A TURN
						rightEncoder = 0;	leftEncoder = 0;
						setState(STRAIGHT);
						}				
			
					break;


			case BACKWARDS: 
					if (abs(rightEncoder) >= SetPoint4Encoders && abs(leftEncoder) >= SetPoint4Encoders){ // Check encoders every 300 steps 
						RightEncoderRatio = SetPoint4Encoders/rightEncoder;
						if (RightEncoderRatio != 0) { PWM_LEFT_WRITE *= RightEncoderRatio; }
						LeftEncoderRatio = SetPoint4Encoders/leftEncoder;	
						if (LeftEncoderRatio != 0) { PWM_RIGHT_WRITE *= LeftEncoderRatio; }
						rightEncoder = 0;	leftEncoder = 0;
					}   
					if( PWM_RIGHT_WRITE >= INITIAL_PWM_LOAD){ 	PWM_RIGHT_WRITE = INITIAL_PWM_LOAD -10; }
					if (PWM_LEFT_WRITE 	>= INITIAL_PWM_LOAD){	 PWM_LEFT_WRITE = INITIAL_PWM_LOAD -10; 	}

					PWM1_0_CMPA_R = PWM_RIGHT_WRITE;
					PWM1_0_CMPB_R = PWM_LEFT_WRITE;

						if ( abs(rightEncoder) >= 600 ||  abs(leftEncoder) >= 600){  setState(STRAIGHT); }
			break;


			case NO_MOVING:    

			break;
			
			
			case TRACKING_CAN: 
					if ( CAN_CENTERED ==1) { // IT WILL NEED TO TRAVEL STRAIGHT 
						
						
						if (rightEncoder >= SetPoint4Encoders || leftEncoder >= SetPoint4Encoders){ // Check encoders every 300 steps 
								RightEncoderRatio = SetPoint4Encoders/rightEncoder;
								if (RightEncoderRatio != 0) { PWM_LEFT_WRITE *= RightEncoderRatio; }
								LeftEncoderRatio = SetPoint4Encoders/leftEncoder;	
								if (LeftEncoderRatio != 0) { PWM_RIGHT_WRITE *= LeftEncoderRatio; }
								rightEncoder = 0;	leftEncoder = 0;
						}   
						if( PWM_RIGHT_WRITE >= INITIAL_PWM_LOAD){ 	PWM_RIGHT_WRITE = INITIAL_PWM_LOAD -10; }
						if (PWM_LEFT_WRITE 	>= INITIAL_PWM_LOAD){	 	PWM_LEFT_WRITE = INITIAL_PWM_LOAD -10; 	}
						PWM1_0_CMPA_R = PWM_RIGHT_WRITE;
						PWM1_0_CMPB_R = PWM_LEFT_WRITE;
						}
			break;
			

			case RIGHT45: 
					if ( (leftEncoder - rightEncoder >= 1050)){ // 300 = NUMBER OF TURNS TO COMPLETE 45 DEGREE TURN
						rightEncoder = 0;	leftEncoder = 0;
						setState(STRAIGHT);
					}				
			break;


			case LEFT45:
					if( (rightEncoder - leftEncoder) >= 1050){ // 300 = NUMBER OF TURNS TO COMPLETE 45 DEGREE TURN
						rightEncoder = 0;	leftEncoder = 0;
						setState(STRAIGHT);
					}
			break; 


			default: break;
		}
			//READ
	//PWM_RightWheel_READ = PWM1_0_CMPA_R;
	//PWM_leftWheel_READ = PWM1_0_CMPB_R;  

		
	
	
}









































