
#include "Init.c" // Initializes 6PWM modules  


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  ARM CONTROL FILE  ////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

/*
PB6		BOTTOM SERVO 	( LEFT RIGHT BASE)			need new formula										 90degress = centered 
			140 T0 690								385 = centered 90
							

PB7		2nd LOWEST		(UP DOWN, BOTTOM)				need new formula 									90degress = Straight up 
			140- 600 									370 = centered 				
		
		
PB4		3RD LOWEST   (UP DOWN)									Y = 2.578X + 145						90degreess  = straght up     increasing moves down 
			150(up) - 600(down)   FourthLower_4th_PB5(0-180)
			
			
PB5  4TH 	LOWEST 	(PINCHER UP AND DOWN) 			
	100 -600 

			
PC4		5TH LOWEST  	(ROTATES PINCHER)																					90degress = 
	100 - 700 												330 = PERPENDICULAR											 550 = PARREL TO GROUND 

PC5 	6TH LOWEST		(PICHER CLOSER)
	200 - 700 												300 = COMPLETLY CLOSED 										700 = COMPLETLY OPEN
																	
*/

void arm4(){
	for(int i = 0; i<180; i++){// moves down 
	FourthLower_4th_PB5(i);
		delay(1000);
	}
			delay(10000);

	for(int i = 180; i>0; i--){ // moves backwards
	FourthLower_4th_PB5(i);
		delay(1000);
	}
}

/* this function is going to pick up the can smoothly 
*/ 
void RecursionForSecThird(uint16_t secondVal, uint16_t thirdVal){
	uint16_t tempSec, tempThird;
	tempSec = secondVal;
	tempThird = thirdVal;
	
	SecLower_2nd_PB7(secondVal);
	ThirdLower_3rd_PB4(thirdVal);
	
	delay(1100);
	if ( (secondVal > 250) && (thirdVal < 100) ){		RecursionForSecThird( tempSec-1, tempThird+1);  }
	else if( (secondVal > 250) && (thirdVal >= 100)){ 		RecursionForSecThird( tempSec-1, tempThird); }
	else if ( (secondVal <= 250) && (secondVal < 100)) { RecursionForSecThird( tempSec, tempThird + 1); }


	
}

void putOnBox (){
	int j = 20;
	int k = 180;
	for(int i = 500; i > 300 ; i=i-2){
		SecLower_2nd_PB7(i) ;
		if(j < 120){
			ThirdLower_3rd_PB4(j);
		}
		if(k > 0){
			FourthLower_4th_PB5(k);
		}
    delay(1000);
		j++; 
		k=k-2;
	}//for 
	delay(10000);
	
	
	CLOSE_OPEN_PINCHER_PC5( 10 ) ; // OPEN PINCHER 
}

/////////////////////////////////// ARM CONTROLLER //////////////////////////////
// camera resolution 640x480
// 45 degrees from the center is approximately within the camera's edges
// Base of Arm
// 140 - 630 range.
// 385 center & 90 degree turns
// 263 - 485 for 45 degree turns ?
// Rotate base to x_pos
int16_t current_pos, target_pos;
float calculate;
uint16_t output, base_new;
uint16_t track = 0;

void retDefault(void){
	//Default Position
	for(int i = 500; i > 250 ; i--){
		SecLower_2nd_PB7(i) ;
		delay(150);
		if ( i == 400 ) {
				for (int j = 20; j <100; j++){
						ThirdLower_3rd_PB4(j);
						delay(150);
				}//for 
		}//if 
	}
	
	
	for (int j = 30; j <= 600; j++){
			FourthLower_4th_PB5(j);
			delay(100);
	}
	retDefault();
}

// Rotates base to x position
void base_rot(uint16_t base){
  if(base > target_pos){
    for(int i = base; i > target_pos; i--){
      delay(100);
      Lower_1st_PB6(i);
    }
  }else if(base < target_pos){
    for(int i = base; i < target_pos; i++){
      delay(100);
      Lower_1st_PB6(i);
    }
  }
}
// Returns base to middle position
void base_ret(uint16_t base){
	retDefault();
  if(base > target_pos){
    for(int i = target_pos; i < base; i++){
      delay(100);
      Lower_1st_PB6(i);
    }
  }else if (base < target_pos){
    for(int i = target_pos; i > base; i--){
      delay(100);
      Lower_1st_PB6(i);
    }
  }
}
//
// For 3rd Lowest Motor
// 160 - 600. Center position uses 
// PID controller to reach can when not center 3rd_lowest modifty


void rotpincher(void){
 // for(int i = 90; i<180; i++){
   // ROTATE_PINCHER_PC4( i);
    //delay(2800);
 // }
  ROTATE_PINCHER_PC4( 180);
}


void pincher(void){
  
  for(int i = 10; i>0; i--){ //close
    CLOSE_OPEN_PINCHER_PC5( i ) ;
         delay(2800);
  }
  delay(6000);
  for(int i = 0; i<10; i++){ //opening
    CLOSE_OPEN_PINCHER_PC5( i ) ;
         delay(2800);
  }
}

void harm_graveX(uint16_t x_pos, uint16_t base){
  // convert x_pos same units as arm position
  // 244 is the distance from 263-507. all 45 degree values from center
  calculate = ((x_pos* 244)/640) + 263; //x coordinate converted
  target_pos = (short)calculate;
  //track = abs(base - target_pos)/(1.5); //position moved, used to adjust arm reaching can
  
  Lower_1st_PB6(base);   //Center Arm 385
  base_rot(base);            //rotate base to x_pos
  
  CLOSE_OPEN_PINCHER_PC5( 10 ) ;  // OPEN
  ROTATE_PINCHER_PC4( 180);	      // MAKE PINCHER PARALLEL TO GOUND																HIGHER VALUE = TILDS HAND
  for(int i = 174; i >= 30; i--){ //MOVE FOUR SERVO IN PREPEARATION TO GRAVE
        FourthLower_4th_PB5( i) ; ///MOVE PINCHER UP AND DONW   	NOTE 180 = GROUND 	0 = SKY			HIGHER VALUE = MOVES SERVO DOWN
        delay(130);
  }
  for (int i =250; i < 400; i++ ) {
    delay(100);
    SecLower_2nd_PB7( i ); //180 DEGREES TO GET INCLINED TO GROUND   												HIGHER VALUE = MOVES SERVO DOWN
  }
  ThirdLower_3rd_PB4(80); // 20 = MAKES ARM LOOK DOWN						          								HIGHER VALUE = MOVES SERVO UP 
  delay(1000);
  SecLower_2nd_PB7( 400); //180 DEGREES TO GET INCLINED TO GROUND   												HIGHER VALUE = MOVES SERVO DOWN
  delay(1000);
  
  
  for (int i =80; i > 20; i-- ) {
    delay(100);
    ThirdLower_3rd_PB4( i); // 20 = MAKES ARM LOOK DOWN																			HIGHER VALUE = MOVES SERVO UP 
  }  
  
  
  delay(1000);
  for (int i =400; i < 500; i++ ) {
    delay(100);
    SecLower_2nd_PB7( i ); //180 DEGREES TO GET INCLINED TO GROUND   												HIGHER VALUE = MOVES SERVO DOWN
  }
  delay(11000);
  CLOSE_OPEN_PINCHER_PC5( 5 ) ; // CLOSE 		
  delay(11000);

  base_ret(base); // Goes back to middle
	putOnBox();
}

