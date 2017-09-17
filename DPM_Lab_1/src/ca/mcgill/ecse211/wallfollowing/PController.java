package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 10;
  private static final double PROPCONST = 2.5;
  public static final int MAXCORRECTION= 50;
  

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  private boolean didReverse;
  private int reverseCounter;
  
  //private float reverseAttenuation;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;
    this.reverseCounter = 0;
    //this.reverseAttenuation = 0.5f;
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 60 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 60) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      if(didReverse && (reverseCounter < 10)) {
    	  reverseCounter++;
      this.distance = distance/2;
      }else{
      didReverse = false;
    	  this.distance = distance;
      };
    }
    int distError = distance - (bandCenter);  
    int motorCorrection = calcProp(distError);
    
    if(distance < 15) {
    		reverseCounter = 0;
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + motorCorrection);
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + motorCorrection);
		WallFollowingLab.rightMotor.backward();
		WallFollowingLab.leftMotor.forward();
		didReverse = true;
	}
    else if(distance > bandCenter + bandWidth) {
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + motorCorrection);
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - motorCorrection);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}else if(distance < bandCenter - bandWidth) {
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - motorCorrection);
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + motorCorrection);
		WallFollowingLab.leftMotor.forward(); 
		WallFollowingLab.rightMotor.forward();
	}else {
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

    
  }

  public int calcProp(int diff) {
	  int correction;
	  if (diff<0) diff = -diff;
	  correction = (int) (PROPCONST*(double)diff);
	  if (correction >= MOTOR_SPEED) correction = MAXCORRECTION;
	  //if(correction < 0) correction = 0;
	  return correction;
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
