package ca.mcgill.ecse211.wallfollowing;

public class BangBangController implements UltrasonicController {

	/* Constants */
	private static final int FILTER_OUT = 20;

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int filterControl;
	private boolean didReverse;
	private int reverseCounter;
	private int deltaSpeed = 200;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.reverseCounter = 0;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
	}

	@Override
	public void processUSData(int distance) {
		// filter out non-repeating large values (false negatives)
		if (distance >= 255 && filterControl < FILTER_OUT) {
			filterControl++;
		// action repeating large values
		} else if (distance >= 255) {
			this.distance = distance;
		// action other values
		} else {
			filterControl = 0;
			// if a reverse operation occurred in last ten cycles, divide distance
			// by two to compensate for large angle of the sensor in relation to the wall
			if(didReverse && (reverseCounter < 10)) {
				reverseCounter++;
				this.distance = distance/2;
			} else {
				didReverse = false;
				this.distance = distance;
			};
		}
		// machine too close to wall, reverse to avoid collision, proportional  to error
		if(distance < 10) {
			reverseCounter = 0;
			didReverse = true;
			WallFollowingLab.rightMotor.setSpeed(motorHigh + deltaSpeed);
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.rightMotor.backward();
			WallFollowingLab.leftMotor.forward();
		}
		// machine outside of band center, accelerate outside wheel
		else if(distance > bandCenter + bandwidth) {
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		// machine inside of band center, accelerate inside wheel
		} else if(distance < bandCenter - bandwidth) {
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		// machine within band
		} else {
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
