#include <mPuckLib.h>
#include "Arduino.h"

void readDistanceIR(int* irSensor)
{
	unsigned int irSensorOn[6];
	unsigned int irSensorRaw[6];
	int i;
	
	/* Differential reading of IR distance sensors */
  	irSensorRaw[0] = analogRead(A4)  >> 4;
  	irSensorRaw[1] = analogRead(A5)  >> 4;
  	irSensorRaw[2] = analogRead(A6)  >> 4;
  	irSensorRaw[3] = analogRead(A0)  >> 4;
  	irSensorRaw[4] = analogRead(A1)  >> 4;
  	irSensorRaw[5] = analogRead(A3)  >> 4;
  	digitalWrite(5, HIGH);
  	irSensorOn[0] = analogRead(A4)  >> 4;  // A4 front right
  	irSensorOn[1] = analogRead(A5)  >> 4;  // A5 center right
  	irSensorOn[2] = analogRead(A6)  >> 4;  // A6 center left
  	irSensorOn[3] = analogRead(A0)  >> 4;  // A0 front left
  	irSensorOn[4] = analogRead(A1)  >> 4;  // A1 left
  	irSensorOn[5] = analogRead(A3)  >> 4;  // A3 right
  	digitalWrite(5, LOW);
  	
  	for(i=0;i<6;i++)
  		irSensor[i] = irSensorRaw[i] - irSensorOn[i];
}

int getLeftFloor(void)
{
	return digitalRead(7);
}

int getRightFloor(void)
{
	return digitalRead(8);
}

int getSwitch(void)
{
	return (PINB & 0b00000010) >> 1;
}

int getPotentiometer(void)
{
	return  analogRead(A7);
}

void braitenbergSpeed(int* irSensor, int* speedLeft,int* speedRight)
{
	//static int braitenberg[6] = {-50,-30,-10,-20,-12,-8}; // good values
	static int braitenberg[6] = {-50,-38,-10,-10,-7,-8};    // also ok
	int speedLeftRaw, speedRightRaw, speedLeftRawPlus, speedRightRawPlus, speedLeftRawMinus, speedRightRawMinus;
	
	  // opposite side IR/wheel values
  	speedLeftRawPlus = (braitenberg[0]*irSensor[1]+braitenberg[1]*irSensor[0]+braitenberg[2]*irSensor[5]) >> 5;
  	speedRightRawPlus = (braitenberg[0]*irSensor[2]+braitenberg[1]*irSensor[3]+braitenberg[2]*irSensor[4]) >> 5;
  
 	 // same side IR/wheel values
 	 speedLeftRawMinus = (braitenberg[3]*irSensor[2]+braitenberg[4]*irSensor[3]+braitenberg[5]*irSensor[4]) >> 5;
 	 speedRightRawMinus = (braitenberg[3]*irSensor[1]+braitenberg[4]*irSensor[0]+braitenberg[5]*irSensor[5]) >> 5;
  
 	 //effective speed
 	 speedLeftRaw = speedLeftRawPlus + speedLeftRawMinus;
 	 speedRightRaw = speedRightRawPlus + speedRightRawMinus;
  
 	 speedLeftRaw = ((signed int)speedLeftRaw)+95;
 	 speedRightRaw = ((signed int)speedRightRaw)+95;
  	
  	
  	 /* Avoid saturation and statism */
  	 if(speedLeftRaw > 100)
  	 	*speedLeft = 100;
	 else if (speedLeftRaw < -100)
  	 	*speedLeft = -100;
 	 else
 	 {
 	 	if((speedLeftRaw < MOTOR_LIMIT) && (speedLeftRaw > 0))
 	   		*speedLeft = MOTOR_LIMIT;
  	  	else if((speedLeftRaw > -MOTOR_LIMIT) && (speedLeftRaw < 0))
 	   		*speedLeft = -MOTOR_LIMIT;
 	  	else
   	   		*speedLeft = speedLeftRaw;
 	 }	
     
 
  	if(speedRightRaw > 100)
    	*speedRight = 100;
  	else if (speedRightRaw < -100)
   		*speedRight = -100;
 	else
    {
    	if((speedRightRaw < MOTOR_LIMIT) && (speedRightRaw > 0))
    		*speedRight = MOTOR_LIMIT;
   	 	else if((speedRightRaw > -MOTOR_LIMIT) && (speedRightRaw < 0))
   	   		*speedRight = -MOTOR_LIMIT;
   	 	else
   	   		*speedRight = speedRightRaw;
  	}
  	
  	if((abs(speedRightRaw) < MOTOR_LIMIT) && (abs(speedLeftRaw) < MOTOR_LIMIT))    // evasive maneuver
    {
      *speedRight = 70;
      *speedLeft = -70;
    }
}

void resetEncoders(unsigned int* encLeft, unsigned int* encRight)
{
	*encLeft = 0;
	*encRight = 0;
}

void odometry(unsigned int encLeft, unsigned int encRight, int32_t* posX, int32_t* posY, int* theta)
{
	static unsigned int prevEncLeft, prevEncRight;
	int deltaR, deltaL, angleDone, distanceDone;
	
	/* Robot wheel movement */
	deltaR = encLeft - prevEncLeft;
	deltaL = encRight - prevEncRight;
	
	/* Additional layer protection */
	if(deltaR > 2)
		deltaR = 2;
	if(deltaL > 2)
		deltaL = 2;
	if(deltaR < -2)
		deltaR = -2;
	if(deltaL < -2)
		deltaL = -2;
	
	
	prevEncLeft = encLeft;
	prevEncRight = encRight;

	/* Robot actual movement */
	angleDone = (deltaL - deltaR); // to multiply by 0.2 to have radians value (resolution, 0.2 radians)
    distanceDone = (deltaL + deltaR)*DISTANCE_CONSTANT; // tenth of mm
    
    /*----- Update position -----*/
    // update angle
    *theta += angleDone;
	while(*theta > 32)
		*theta -= 32;
    while(*theta < 0)
		*theta += 32;
		
    // update x,y (now in hundredth of mm
    *posX += (int32_t)distanceDone*(int32_t)cosComp[*theta];
    *posY += (int32_t)distanceDone*(int32_t)sinComp[*theta];
    
      /*  Serial.print("theta ");
      Serial.println(*theta);
      Serial.print("deltaRRRRRRRRR ");
      Serial.println(deltaR);
      Serial.print("deltaLLLLLLL ");
      Serial.println(deltaL);
      
            Serial.print("encoderRRR ");
      Serial.println(encRight);
      Serial.print("encoderLL ");
      Serial.println(encLeft);*/

}

void controller(int32_t posX, int32_t posY, int theta, int32_t destinationX, int32_t destinationY, int destinationTheta, int* speedRight, int* speedLeft)
{
	int deltaX, deltaY;
	int rho, alpha, beta;
	int linearSpeed, rotationalSpeed;
	
	/* ----- Update goal -----  */
    deltaX = destinationX - (posX/1000);		// instead of 100 multiples, 128 multiples? (2^7)			
    deltaY = destinationY - (posY/1000);		// delta X,Y in cm. theta in hundredth of radians


    rho = sqrt((int32_t)deltaX*(int32_t)deltaX+(int32_t)deltaY*(int32_t)deltaY);   // in cm			
    
	alpha = (atan2(deltaY,deltaX))*100; // hundreds of radians (not sure about the cast for atan2)

    if(alpha < 0)
		alpha += 628;
	
	alpha -= theta*20;					// theta should match radians DA VERIFICAREEEEEEE
    
      if(alpha > 314)
	alpha -= 628;
      if(alpha < -314)
	alpha += 628;
    
    beta = -theta - alpha + destinationTheta;                 // hundreds of radians
      if(beta > 314)
	beta -= 628;
      if(beta < -314)
	beta += 628;
	

	  /* ----- Proportional part ----- */
  //linearSpeed = KP_RHO*rho;
  //rotationalSpeed = KP_ALPHA*alpha+KP_BETA*beta;
   linearSpeed = rho/5;
  rotationalSpeed = alpha/3-beta/4;

  
  *speedRight = linearSpeed+rotationalSpeed;
  *speedLeft = linearSpeed-rotationalSpeed;
  

      Serial.print("linear:");
      Serial.println(linearSpeed);
        Serial.print("ritational:");
      Serial.println(rotationalSpeed);
      
      Serial.print("x:");
      Serial.println(posX/1000);
        Serial.print("y:");
      Serial.println(posY/1000);
        Serial.print("theta:");
      Serial.println(theta);

  
	/* Avoid saturation */
  	if(*speedLeft > 60)
  		*speedLeft = 60;
	if (*speedLeft < -60)
  	 	*speedLeft = -60;
  	 	
  	if(*speedRight> 60)
    	*speedRight = 60;
  	if (*speedRight < -60)
   		*speedRight = -60;

}