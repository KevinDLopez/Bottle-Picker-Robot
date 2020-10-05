
#include "init.c"

/*

16CPR(motor shaft)/700CPR(gearbox shaft)

Wheel 1 
	PWM -			PD0
	DIRECTION-		PE2,3
    encoder a - 	PF0
    encoder b  - 	PF1




Wheel 2
	PWM -			PD1
	DIRECTION-		PE4,5
    encoder a  -	PC6
    encoder b - 	PC7 
*/

	uint8_t FIRST_STATE[] = "INITIAL STATE GOING Y++\n\r";
	uint8_t UP_STRING[] = "TRAVELING UP NOW \n\r";
	uint8_t DOWN_STRING[] = "TRAVELING DOWN \n\r";
	uint8_t LEFT_STRING[] = "TRAVELING LEFT \n\r";
	uint8_t RIGHT_STRING[] = "TRAVELING RIGHT \n\r";
	uint8_t LEFT_TURN_STRING[] = "TURNING LEFT \n\r";
	uint8_t RIGHT_TURN_STRING[] = "TURNING RIGHT \n\r";
	uint8_t STRAIGHT_ACTION[] = "CAR IS GOING STRAIGHT FOR NOW \n\r";
	uint8_t NOT_MOVING_STRING[] = "CAR IS NOT MOVING FOR NOW \n\r";
	uint8_t GRAVING_CAN_STRING[] = "CAR IS GRAVING CAN \n\r";
	uint8_t OBJECT_FOUND_STRING[] = "CAR HAS FOUNT A CAN \n\r";
	//uint8_t GRAVING_CAN_STRING[] = "CAR IS GRAVING CAN \n\r";
	
	int8_t rightPos_leftNeg = 0; // if = 2    then 90 degree turn to right 
							//  if = -2 	then 90 degree turn to left 

	enum TRAVEL_LOCATIONS{TRAVELED = 1, OBJECT = 5};
	enum CAR_DIRECTIONS{INITIAL =50, STRAIGHT, LEFT, RIGHT, NO_MOVING, TRACKING_CAN, LEFT45, RIGHT45}; // SET BY CAMARA AND SENSORS
	enum ENUMTRAVELPLANE{XPLANE = 10, YPLANE};
	enum CAR_RELATED2PLANE{UP = 20, DOWN};
	enum ENCODER_STATE{ FOWARD =100, BACKWARDS, STOPPED};
	enum CAR_ACTIONS{ GRAVING_CAN = 150};



/**
	Sets the State that the car is on. This is Set by Ultrasonic sensor and its used to keep a 
	constant movement and easier tracking
**/ 
void setState(uint8_t state){ // TURNS ARE ONLY TEMPORARY

	switch (state) {// SETS THE PWM FOR THE WHEELS AND DIRECTION   // INITIAL STATE 
		case INITIAL: 
				GPIO_PORTE_DATA_R |= 0x28; GPIO_PORTE_DATA_R &= ~0x14;  // SETS BOTH WHEELS TO GO FOWARD
				GPIO_PORTE_DATA_R |= 0x10; GPIO_PORTE_DATA_R &= ~0x20;   //SETS BOTH WHEELS TO GO FOWARD
				PWM1_0_CMPB_R = INITIAL_PWM_COMP; //PD1-Right motor
				PWM1_0_CMPA_R = INITIAL_PWM_COMP; //PD0-Left Motor
				CAR_ACTION = STRAIGHT;
				CAR_RELATED2PLANE = UP;
				LeftEncoderState = FOWARD;
				RightEncoderState = FOWARD;
		break;
		
		case STRAIGHT: // straight
				GPIO_PORTE_DATA_R |= 0x28; GPIO_PORTE_DATA_R &= ~0x14;  // SETS BOTH WHEELS TO GO FOWARD
				GPIO_PORTE_DATA_R |= 0x10; GPIO_PORTE_DATA_R &= ~0x20;   //SETS BOTH WHEELS TO GO FOWARD

			
				PWM1_0_CMPB_R = INITIAL_PWM_COMP; //PD1-Right motor
				PWM1_0_CMPA_R = INITIAL_PWM_COMP; //PD0-Left Motor
				CAR_ACTION = STRAIGHT;
				LeftEncoderState = FOWARD;
				LeftEncoderState = FOWARD;
				RightEncoderState = FOWARD;

				UART7_Transmit_String(STRAIGHT_ACTION); 
		break;
		
		case LEFT: // Turn 90 degrees left	
					GPIO_PORTE_DATA_R |= 0x28; GPIO_PORTE_DATA_R &= ~0x14;  // LEFT
					PWM1_0_CMPB_R = TURNING_PWM_COM - 150;
					PWM1_0_CMPA_R = TURNING_PWM_COM;  // LEFT WHEEL 
					CAR_ACTION = LEFT;
					rightEncoder = 0;	leftEncoder = 0;
					LeftEncoderState = BACKWARDS;
					RightEncoderState = FOWARD;
					switch( CAR_RELATED2PLANE  ){ // WHAT WHAT DIRECTION WAS THE CAR TRAVELING RELATED TO THE PLANE AND WHAT IS GOING TO DIRECT TO
						case UP: 		CAR_RELATED2PLANE = LEFT;		UART7_Transmit_String(LEFT_STRING); break;
						case DOWN:	CAR_RELATED2PLANE	= RIGHT;	UART7_Transmit_String(RIGHT_STRING); break;
						case LEFT: 	CAR_RELATED2PLANE	= DOWN;		UART7_Transmit_String(DOWN_STRING); break;
						case RIGHT: CAR_RELATED2PLANE	= UP;			UART7_Transmit_String(UP_STRING); break;
					}
					newQuadR = 0;
					newQuadL = 0;
					UART7_Transmit_String(LEFT_TURN_STRING); 
		break;
				
		case RIGHT: // Turn 90 degrees right
					GPIO_PORTE_DATA_R &= ~0x28; GPIO_PORTE_DATA_R |= 0x14;  // SETS BOTH WHEELS TO GO BACKWARDS
					PWM1_0_CMPB_R = TURNING_PWM_COM; //RIGHT WHEEL
					PWM1_0_CMPA_R = TURNING_PWM_COM - 150;
					CAR_ACTION = RIGHT;
					rightEncoder = 0;	leftEncoder = 0;
				

					LeftEncoderState = FOWARD;
					RightEncoderState = BACKWARDS;

					switch( CAR_RELATED2PLANE  ){ // WHAT WHAT DIRECTION WAS THE CAR TRAVELING RELATED TO THE PLANE and what it will be after right turn 
						case UP: 	CAR_RELATED2PLANE = RIGHT;	UART7_Transmit_String(RIGHT_STRING);		break;
						case DOWN:	CAR_RELATED2PLANE = LEFT;		UART7_Transmit_String(LEFT_STRING);	break;
						case LEFT: 	CAR_RELATED2PLANE = UP;			UART7_Transmit_String(UP_STRING); break;
						case RIGHT: CAR_RELATED2PLANE = DOWN;		UART7_Transmit_String(DOWN_STRING);	break;
					}
					newQuadR = 0;
					newQuadL = 0;
					UART7_Transmit_String(RIGHT_TURN_STRING); 
		break;
			
		case BACKWARDS: // BACKWARDS
				GPIO_PORTE_DATA_R |= 0x28; GPIO_PORTE_DATA_R &= ~0x14;   // SETS BOTH WHEELS TO GO FOWARD
				GPIO_PORTE_DATA_R |= 0x04; GPIO_PORTE_DATA_R &= ~0x08;   // SETS THE LEFT WHEEL TO GO  BAKWARDS

				PWM1_0_CMPB_R = INITIAL_PWM_COMP;
				PWM1_0_CMPA_R = INITIAL_PWM_COMP;
				CAR_ACTION = BACKWARDS;
				rightEncoder = 0;	leftEncoder = 0;
				LeftEncoderState = BACKWARDS;
				RightEncoderState = BACKWARDS;
		break;
		
		case NO_MOVING: // CAR IS STOPPED
				GPIO_PORTE_DATA_R &= ~0x3C; 
				PWM1_0_CMPB_R = 1;
				PWM1_0_CMPA_R = 1;
				CAR_ACTION = NO_MOVING;
				UART7_Transmit_String(NOT_MOVING_STRING);
				//LeftEncoderState = STOPPED;
				//RightEncoderState = STOPPED;
				LeftEncoderState = FOWARD;  // for debuggin porspuses 
				RightEncoderState = FOWARD; // for debuggin porspuses 

		break;
		
		case TRACKING_CAN:  // PWM WILL BE CONTROLLED BY THE CAMERA
				CAR_ACTION = TRACKING_CAN; 
				GPIO_PORTE_DATA_R |= 0x28; GPIO_PORTE_DATA_R &= ~0x14;  // SETS BOTH WHEELS TO GO FOWARD
				GPIO_PORTE_DATA_R |= 0x10; GPIO_PORTE_DATA_R &= ~0x20;   //SETS BOTH WHEELS TO GO FOWARD
				PWM1_0_CMPB_R = 650; //SLOW INITIAL PWM SO WE DONT RUN OVER CAN
				PWM1_0_CMPA_R = 650;	//SLOW INITIAL PWM SO WE DONT RUN OVER CAN
				
				UART7_Transmit_String(OBJECT_FOUND_STRING);
				LeftEncoderState = FOWARD;
				RightEncoderState = FOWARD;
				///PWM AND WHEEL CONTROL WILL BE SET BY PID2CAN
		break;
		
		case GRAVING_CAN: 
				GPIO_PORTE_DATA_R &= ~0x3C; 
				PWM1_0_CMPB_R = 1;            //Stop the motors
				PWM1_0_CMPA_R = 1;
				CAR_ACTION = GRAVING_CAN;
				UART7_Transmit_String(GRAVING_CAN_STRING);
				
			
				//
				// servos do the work now
				// put funtions here to grave can 
				harm_graveX(X_POSITION, 385);
				//harm_graveX(320, 385);

		
				break;



		case RIGHT45: // 45 DEGREE TURN WHEN RIGHT SENSOR HITS THE WALL 
					GPIO_PORTE_DATA_R &= ~0x28; GPIO_PORTE_DATA_R |= 0x14;  // SETS BOTH WHEELS TO GO BACKWARDS
					PWM1_0_CMPB_R = TURNING_PWM_COM; //RIGHT WHEEL
					PWM1_0_CMPA_R = TURNING_PWM_COM - 150;
					CAR_ACTION = RIGHT45;
					rightEncoder = 0;	leftEncoder = 0;
					LeftEncoderState = FOWARD;
					RightEncoderState = BACKWARDS;

					rightPos_leftNeg++;	 // AFTER TWO 45 DEGREE TURNS IT TURNS RIGHT 
						if( rightPos_leftNeg >= 2){ 
							rightPos_leftNeg = 0;
							switch( CAR_RELATED2PLANE  ){ // WHAT WHAT DIRECTION WAS THE CAR TRAVELING RELATED TO THE PLANE and what it will be after right turn 
								case UP: 	CAR_RELATED2PLANE = RIGHT;		UART7_Transmit_String(RIGHT_STRING);		break;
								case DOWN:	CAR_RELATED2PLANE = LEFT;		UART7_Transmit_String(LEFT_STRING);	break;
								case LEFT: 	CAR_RELATED2PLANE = UP;			UART7_Transmit_String(UP_STRING); break;
								case RIGHT: CAR_RELATED2PLANE = DOWN;		UART7_Transmit_String(DOWN_STRING);	break;
							}//switch
						}// if 	

				break;

		case LEFT45: // 45 DEGREE TURN TO THE LEFT WHEN SENSOR HITS THE WALL 
					GPIO_PORTE_DATA_R |= 0x28; GPIO_PORTE_DATA_R &= ~0x14;  // LEFT
					PWM1_0_CMPB_R = TURNING_PWM_COM - 150; // LEFT SLOWER SO RIGHT TAKES MORE POWER
					PWM1_0_CMPA_R = TURNING_PWM_COM;  // RIGHT WHEEL 
					CAR_ACTION = LEFT45;
					rightEncoder = 0;	leftEncoder = 0;
					LeftEncoderState = BACKWARDS;
					RightEncoderState = FOWARD;

					rightPos_leftNeg--; //AFTER TWO 45 DEGREE TURNS IT TURNS LEFT
						if( rightPos_leftNeg <= -2 ) {
							rightPos_leftNeg = 0;
							switch( CAR_RELATED2PLANE  ){ // WHAT WHAT DIRECTION WAS THE CAR TRAVELING RELATED TO THE PLANE AND WHAT IS GOING TO DIRECT TO
								case UP: 	CAR_RELATED2PLANE = LEFT;		UART7_Transmit_String(LEFT_STRING); break;
								case DOWN:	CAR_RELATED2PLANE	= RIGHT;	UART7_Transmit_String(RIGHT_STRING); break;
								case LEFT: 	CAR_RELATED2PLANE	= DOWN;		UART7_Transmit_String(DOWN_STRING); break;
								case RIGHT: CAR_RELATED2PLANE	= UP;			UART7_Transmit_String(UP_STRING); break;
							}//SWITCH

						}//IF  				 

		break;

	}
	
	
}





