
#include "Encoders.c"
#define TRUE  1
#define FALSE 0
#define ULTRASONIC_MIN_WORKING_RANGE	70

/**
This method converts the Ultrasonic travel disntace to usable Distance Data. 
Data is represented in CM
*/
int Distance(int countms ) {
	//return (  (countms*SOUND_SPEED)/2  ) ; 
	return ( (countms/2)/2.9  );
}


void turnRequired(){ // this can happen on UART 0 WHEN we get DATA saying we have encountered a wall 

	// if the car has traveled across any of those areas 
	// first check if the car has traveled across the other square so we wont travel across form it again

		uint8_t rightSideOpen = TRUE, leftSideOpen = TRUE;  // this is related to the car
		//uint8_t LOOKING4NEWROUTE  = FALSE;
						rightEncoder = 0;	leftEncoder = 0;
		
			switch(CAR_RELATED2PLANE){ 
				case UP:    if( PAST_LOCATIONS[xCarPosition + 1][yCarPosition] == TRAVELED ){ // IF IT TRAVELED TRHOUGHT THE SIDE 
							rightSideOpen = FALSE;
							} else if( PAST_LOCATIONS[xCarPosition - 1 ][yCarPosition]  == TRAVELED){
							leftSideOpen = FALSE;
							} break;
				case DOWN:	if( PAST_LOCATIONS[xCarPosition  - 1][yCarPosition] == TRAVELED ){ // IF IT TRAVELED TRHOUGHT THE SIDE 
							rightSideOpen = FALSE;
							} else if( PAST_LOCATIONS[xCarPosition + 1 ][yCarPosition]  == TRAVELED){
							leftSideOpen = FALSE;
							} break;

				case LEFT: if( PAST_LOCATIONS[xCarPosition ][yCarPosition + 1]  == TRAVELED ){ // IF IT TRAVELED TRHOUGHT THE SIDE 
							rightSideOpen = FALSE;
							} else if( PAST_LOCATIONS[xCarPosition ][yCarPosition - 1]  == TRAVELED){
							leftSideOpen = FALSE;
							} break;

				case RIGHT: 
							if( PAST_LOCATIONS[xCarPosition ][yCarPosition - 1]  == TRAVELED ){ // IF IT TRAVELED TRHOUGHT THE SIDE 
							rightSideOpen = FALSE;
							} else if( PAST_LOCATIONS[xCarPosition ][yCarPosition + 1]  == TRAVELED){
							leftSideOpen = FALSE;
							} break;
			}       

			
			if( rightSideOpen == TRUE && leftSideOpen == TRUE ){// if both sides are open
				if( (RightSensorDistance > 20) && (LeftSensorDistance <= RightSensorDistance)) {setState(RIGHT);}  
				else if( LeftSensorDistance > 20 && (RightSensorDistance < LeftSensorDistance)) {setState(LEFT);}
				else{
					// we need to mark that the site is closed. 
					setState(BACKWARDS);
				}
			}else if ( rightSideOpen == TRUE ){ // if right side is open
				if ( RightSensorDistance > 20) { setState(RIGHT); } // if there is no object
			}else if ( leftSideOpen == TRUE ) {// if left side is open
				if ( LeftSensorDistance > 20 ) { setState(LEFT);  } // if there is noting next to it
			}else { // no side is open just go to the side that there is no object
				
				if(RightSensorDistance > 20) {setState(RIGHT);}  
				else if(LeftSensorDistance > 20) {setState(LEFT); }
				else{
					// we need to mark that the site is closed. 
					setState(BACKWARDS);
				}
							// NOW IS TRAVELING TRHOUGH A CITE THAT IT HAS TRAVELED BEFORE 
							//LOOKING4NEWROUTE = TRUE;
			}// else
				


}//end of FUNCTION 



/////////////////////////////////////////////////////////////////////////////
//////////////////////////	INTERRUPT HANDELRS FOR ULTRASONIC //////////////
/////////////////////////////////////////////////////////////////////////////
/**
	10MicroSecond ONE SHOT TIMER TIMER TO CREATE THE TRING PULSE FOR THE ULTRASONIC
*/
void Timer1A_Handler(void){ //10MicroS  one-shot   TO TURN OFF TRIG
	volatile uint32_t readback;
		if(TIMER1_MIS_R & 0x01){			// Timer1A timeout flag is set
			
		GPIO_PORTB_DATA_R &= ~0x04; // TURN OFF TRIG  PB2
		GPIO_PORTF_DATA_R &= ~0x08; // TURN OFF TRIG PF3
		GPIO_PORTD_DATA_R &= ~0x04; // TURN OFF TRIG PD2

		iUltraSonic = 1; // reset counter
			
		TIMER2_CTL_R |= 0x01;				// enable Timer2A to count for Ms
		TIMER1_CTL_R = 0;						// disable Timer1 			DISABLE IT SELF
			
		TIMER1_ICR_R = 0x01;				// clear Timer1A timeout flag
		readback = TIMER1_ICR_R;		// force interrupt flag clear
	}
	else{
		TIMER1_ICR_R = TIMER1_MIS_R;	// clear all flags
		readback = TIMER1_ICR_R;			// force interrupt flags clear
	}
}

	



/**
	10MicroSecond INTERRUPT TO COUNT THE TIME THE SIGNAL TAKES TO TRAVEL BACKT TO 
	THE PORT (ECHO).
*/
void Timer2A_Handler(void){ // To count MicroSenconds  
	//volatile uint32_t readback;
		//if(TIMER2_MIS_R & 0x01){					// Timer2A timeout flag is set
			
		iUltraSonic++;   // increase iUltraSonic  to keep count of Mseconds

		TIMER2_ICR_R = 0x01;					// clear Timer2A timeout flag
	
		if(iUltraSonic >= 18000){ //THE MAX VALUE FOR THE ULTRASONIC 
			GPIO_PORTD_DATA_R |= 0x04; // TURN ON TRING PD2						
			GPIO_PORTB_DATA_R |= 0x04; // turn ON Trig  PB2
			GPIO_PORTF_DATA_R |= 0x08; // TURN ON TRIG PF3
				//turn on one-shot 10Ms
			TIMER1_CTL_R |= 0x01;				// enable Timer1A one-shot
		}
		
		//readback = TIMER2_ICR_R;			// force interrupt flag clear
	//}
	//else{
		TIMER2_ICR_R = TIMER2_MIS_R;	// clear all flags
		//readback = TIMER2_ICR_R;			// force interrupt flag clear
	//}
}
		
		




/**
INTERUPT FOR PORTB HAS OCCURED. THIS INTERRUPT MEANST THAT A SIGNAL FORM 
THE ULTRASONIC SENSOR HAS ARRRIVED. WE WILL NEED TO FIND OUT HOW LONG IT TOOK 

*/
void GPIOPortB_Handler(void){ //negedge has been detected form PB3
			volatile int readback;	
	
				RightSensorTime = iUltraSonic; 
	
			if ( RightSensorTime >= ULTRASONIC_MIN_WORKING_RANGE ){ 
					RSD = Distance(RightSensorTime); 
				if(nR < rSize){ RightSensorDistances[nR] = RSD; nR++; } 
				else if( nR >= rSize){ // if full 
					nR = 0; 
					totalRightS = 0;
					
					for(int j = 0; j < rSize; j++){
						totalRightS +=  RightSensorDistances[j]; // add all values onto one variable
					}
					averageRightS = totalRightS/rSize;
				//	RightSensorDistance = averageRightS;
				}
		} 
	
			if ( RightSensorTime >=  ULTRASONIC_MIN_WORKING_RANGE){		
							RightSensorDistance = Distance(RightSensorTime);
			}
			
			// CASE WHERE IT GETS CLOSE TO A WALL 
			if ( RightSensorDistance <= 15 && CAR_ACTION == STRAIGHT && RightSensorDistance > 2 ){
				setState(LEFT45);				
			}
			
			GPIO_PORTB_ICR_R |= 0x08;				// clear the interrupt flag
			readback = GPIO_PORTB_ICR_R;		// read to force interrupt flag clear
}





		

/**
INTERUPT FOR PORTB HAS OCCURED. THIS INTERRUPT MEANST THAT A SIGNAL FORM 
THE ULTRASONIC SENSOR HAS ARRRIVED. WE WILL NEED TO FIND OUT HOW LONG IT TOOK 
PORT D3  ECHO IS RECEIVED AND PD2 TRIG IS SENT 
*/
void GPIOPortD_Handler(void){ //PD3
	volatile int readback;
	if (GPIO_PORTD_MIS_R & 0x08) { 
				FrontSensorTime = iUltraSonic; 
		
				/////////////this if will get the average of 3 measurements///////////
				if ( FrontSensorTime >= ULTRASONIC_MIN_WORKING_RANGE ){ 
					
					FSD = Distance(FrontSensorTime); 
					if(nF < aSize){ FrontSensorDistances[nF] = FSD; nF++; } 
					else if( nF >= aSize){ // if full 
						nF = 0; 
						totalFrontS = 0;						
						for(int j = 0; j < rSize; j++){
							totalFrontS +=  FrontSensorDistances[j]; // add all values onto one variable
						}
						averageFrontS = totalFrontS/aSize;
					}
				}/////////////this if will get the average of 3 measurements/////////

					
				//THIS IF WILL SET A DISTNACE IF OUR ULTRASONIC IS READING A CREDIBLE VALUE
				if ( FrontSensorTime >=  ULTRASONIC_MIN_WORKING_RANGE){
								FrontSensoDistance = Distance(FrontSensorTime);
				}
				
				// WHEN WE ARE TRAKING A CAN AND WE GET CLOSE TO THE CAN WE ARE GOING TO START GRAVING
				if( CAR_ACTION == TRACKING_CAN && FrontSensoDistance < 28){  // 22 = 15cm 
					  
						setState( GRAVING_CAN);

					
				}
				
				
		
		if( FrontSensoDistance <= 25 && FrontSensoDistance >= 9 && (CAR_ACTION == STRAIGHT || CAR_ACTION == INITIAL) ){
			turnRequired();
		}  //THIS IS NOT GOOD WHEN APROACHING A CAN
				
	
		GPIO_PORTD_ICR_R = 0x08;
		readback = GPIO_PORTD_ICR_R;
		
	}// if MIS == 0x08




}



















