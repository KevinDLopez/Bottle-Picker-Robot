
#include "ULTRASONIC.c" // Initializes 6PWM modules  


/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////  ARM CONTROL FILE  ////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

/*
PB6		BOTTOM SERVO 	( LEFT RIGHT BASE)			need new formula										 90degress = centered 
			140 T0 630								385 = centered 90
							

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
/////////////////////////////////// ARM CONTROLLER //////////////////////////////
// These function is going to pick up the can smoothly 
int16_t current_pos, target_pos;
//float calculate;
uint16_t calculate2;
uint16_t output, base_new;
uint16_t track = 0;

//Return ARM to default position
void retDefaultFromGrave(void){
	int j = 75;
	int k = 20;
	//Default Position
	for(int i = 450; i > 400 ; i--){
		SecLower_2nd_PB7(i) ;
		delay(1000);
	}
	for(int i = 400; i > 250 ; i--){
		SecLower_2nd_PB7(i);
		if(j<180){
			FourthLower_4th_PB5(j);
		}
		if(k>0){
			ThirdLower_3rd_PB4(k);
		}
		j++;
		k--;
		delay(1000);
	}//for 
}

void retDefaultFromBox(void){
	int i = 400;
	int k = 130;
	for(int j = 0; j < 180; j++){
		if(i>250){
			SecLower_2nd_PB7(i);
		}
		
		FourthLower_4th_PB5(j);
		
		if(k > 0){
			ThirdLower_3rd_PB4(k);
		}
		i--;
		k--; 		
    delay(1000);
	}
		CLOSE_OPEN_PINCHER_PC5( 4 ) ; // OPEN PINCHER 
}

//2nd = 250, 3rd = 0, 4th = 180
//Put the can into a box
void putOnBox (void){
	int i = 250;
	int k = 0;
	for(int j = 180; j > 0 ; j--){
		if(i<400){
			SecLower_2nd_PB7(i);
		}
		
		FourthLower_4th_PB5(j);
		
		if(k < 130){
			ThirdLower_3rd_PB4(k);
		}
		i++;
		k++; 		
    delay(1000);
	}//for 
	delay(7500);
	
	CLOSE_OPEN_PINCHER_PC5( 10 ) ; // OPEN PINCHER 
	delay(7500);
	retDefaultFromBox();
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
	retDefaultFromGrave();
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
// camera resolution 640x480
// 45 degrees from the center is approximately within the camera's edges
// Base of Arm
// 385 center
// 140 - 630 range for full 90 degree turns
// 263 - 507 for 45 degree turns ?
// For 3rd Lowest Motor
// 160 - 600. Center position uses 
// PID controller to reach can when not center 3rd_lowest modifty
void harm_graveX(uint16_t x_pos, uint16_t base){
	int j = 180;
	int k = 0;
	
  // convert x_pos same units as arm position
  // 244 is the distance from 263-507. all 45 degree values from center
  //calculate = ((x_pos* 244)/640) + 263; //x coordinate converted
	calculate2= ((-0.1724) * x_pos) + 430;
  target_pos = (short)calculate2;
  
	//track = abs(base - target_pos)/(1.5); //position moved, used to adjust arm reaching can
  
  Lower_1st_PB6(base);   //Center Arm 385
  base_rot(base);        //rotate base to x_pos
  
	//Grabbing
  CLOSE_OPEN_PINCHER_PC5( 10 ) ;  // OPEN
  ROTATE_PINCHER_PC4( 180);	      // MAKE PINCHER PARALLEL TO GOUND																HIGHER VALUE = TILDS HAND
  for (int i =250; i < 400; i++ ) {
    SecLower_2nd_PB7( i ); //180 DEGREES TO GET INCLINED TO GROUND   												HIGHER VALUE = MOVES SERVO DOWN
		if(j > 75){    
			FourthLower_4th_PB5( j) ; ///MOVE PINCHER UP AND DONW   	NOTE 180 = GROUND 	0 = SKY			HIGHER VALUE = MOVES SERVO DOWN
		}
		if(k < 20){
			ThirdLower_3rd_PB4( k); // 20 = MAKES ARM LOOK DOWN																			HIGHER VALUE = MOVES SERVO UP 
		}
		j--;
		k++;
		delay(1000);
  }

  for (int i =400; i < 450; i++ ) {
    delay(1000);
    SecLower_2nd_PB7( i ); //180 DEGREES TO GET INCLINED TO GROUND   												HIGHER VALUE = MOVES SERVO DOWN
  }
  delay(11000);
  CLOSE_OPEN_PINCHER_PC5( 0 ) ; // CLOSE 		
  delay(11000);

	
	
  base_ret(base); // Goes back to middle
	delay(25000);
	
	putOnBox();
}

