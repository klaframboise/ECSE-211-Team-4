package ca.mcgill.ecse211.localization;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {

	private static final float LINE_RED_INTENSITY = 0.25f;
	//TODO measure this value
	private static final double LS_TO_CENTER = 9;

	private SampleProvider colorSampler;
	private float[] lightData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	int counter;
	double[] angles;

	public LightLocalizer(SampleProvider colorSampler, float[] lightData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.colorSampler = colorSampler;
		this.lightData = lightData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		counter = 0;
		angles = new double[4];
	}

	private void sweep() {
		int rotationAngle = LocalizationLab.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360);
		counter = 0;
		angles = new double[4];

		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);

		// start rotating 360 deg
		leftMotor.rotate(rotationAngle, true);
		rightMotor.rotate(rotationAngle, true);

		// sample light sensor every 25 ms to detect lines
		do {
			if(getRedIntensity() < LINE_RED_INTENSITY) {
				angles[counter++] = LocalizationLab.getOdo().getTheta();
				Sound.beep();
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		} while(leftMotor.isMoving() && rightMotor.isMoving() && counter < angles.length);

	}

	public void localize() {
		double x;
		double y;
		double dTheta;

		while(true) {
			sweep();

			// already in position to localize
			if(counter == 4) {
				x = -LS_TO_CENTER * Math.cos((angles[2] - angles[0])/2);	//theta-y is difference in angle between the first and third line crossed
				y =  -LS_TO_CENTER * Math.cos((angles[3] - angles[1])/2);	//theta-x is difference in angle between the second and fourth line crossed
				dTheta = -Math.PI/2.0 - angles[2] - (angles[2] - angles[0])/2;
				break;
			}
			else if(counter == 2) {
				// if the second line is detected at theta < 180 deg, it was the y-axis
				if(angles[1] < Math.PI) {
					adjust('x');	// need to go in positive y direction in order to come across x-axis
				}
				else {
					adjust('y');	// need to go in positive x direction in order to come across y-axis
				}
			}
			//edge case, implement if necessary
			else if(counter == 3 || counter == 1) {
				Sound.buzz();
			}
			else {
				// no lines were found, go in both positive x and y until axis found
				adjust('x');
				adjust('y');
			}
		}
		
		// adjust odometer
		LocalizationLab.getOdo().setX(x);
		LocalizationLab.getOdo().setY(y);
		LocalizationLab.getOdo().setTheta(LocalizationLab.getOdo().getTheta() + dTheta);
		
		//travel to 0,0
		travelTo(0,0);
		//turn to face heading 0deg
		turnTo(0);
	}

	private void adjust(char dir) {

		switch(dir) {
		// head in positive x direction
		case 'y': turnTo(Math.PI/2.0); break;
		case 'x': turnTo(0); break;
		default: return;
		}

		// go forward until axis is found
		while(getRedIntensity() < LINE_RED_INTENSITY) {
			leftMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
			rightMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
			
			leftMotor.forward();
			rightMotor.forward();
			
			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		// go 2cm past the line
		leftMotor.rotate(LocalizationLab.convertDistance(LocalizationLab.WHEEL_RADIUS, 2));
		rightMotor.rotate(LocalizationLab.convertDistance(LocalizationLab.WHEEL_RADIUS, 2));

	}
	
	void travelTo(double x, double y) {

		double dX = x - LocalizationLab.getOdo().getX();
		double dY = y - LocalizationLab.getOdo().getY();
		double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
		double heading = 0; 

		//compute heading
		if (dY >= 0) { //ok
			heading = Math.atan(dX/dY); //isn't it dX/dY ?
		}
		else if (dY < 0) { //ok
			heading = Math.PI + Math.atan(dX/dY);
		}

		// wrap around 2pi
		if(heading > 2 * Math.PI) heading -= 2 * Math.PI;

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		//turn robot to wanted heading 
		turnTo(heading);

		//travel to x,y
		int rotateAngle = LocalizationLab.convertDistance(LocalizationLab.WHEEL_RADIUS, distance);
		leftMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
		rightMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
		leftMotor.rotate(rotateAngle, true);
		rightMotor.rotate(rotateAngle, false);
	}

	private void turnTo(double theta) {

		double dTheta = theta - LocalizationLab.getOdo().getTheta();
		if(dTheta < 0) dTheta += 2 * Math.PI;
		//System.out.println("\n\n\n\n\n\ndtheta: " + dTheta);

		if (dTheta > Math.PI) {
			dTheta = 2* Math.PI - dTheta;
			turn(Math.toDegrees(dTheta), "left");
		}
		else {
			turn(Math.toDegrees(dTheta), "right");
		}

	}

	/**
	 * Turns in given direction.
	 * @param dTheta change in heading wanted, in degrees
	 * @param direction
	 */
	private void turn(double dTheta, String direction) {

		int distance = LocalizationLab.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, dTheta);

		// set motor speed
		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);

		switch (direction) {
		case "left" :
			leftMotor.rotate(-distance, true);
			rightMotor.rotate(distance, false);
			break;
		case "right" :
			leftMotor.rotate(distance, true);
			rightMotor.rotate(-distance, false);
			break;
		}
	}

	/**
	 * Returns the mean of a group of sample from the color sensor as the detected color.
	 * @return color intensity detected
	 */
	private float getRedIntensity() {

		for(int i = 0; i < lightData.length; i++) {
			colorSampler.fetchSample(lightData, i);
		}

		Arrays.sort(lightData);

		return (lightData[(lightData.length/2) - 1] + lightData[lightData.length/2]) / 2.0f;
	}
}
