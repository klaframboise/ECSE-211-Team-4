package ca.mcgill.ecse211.navigation;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation implements UltrasonicController {
	
	/* Constants */
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	private Odometer odo;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean isNavigating;
	private double waypointX;
	private double waypointY;
	
	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		isNavigating = false;
		waypointX = 0;
		waypointY = 0;
	}
	
	void travelTo(double x, double y) {
		
		isNavigating = true;
		waypointX = x;
		waypointY = y;
		
		double dX = x - odo.getX();
		double dY = y - odo.getY();
		double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
		double heading = 0; 
		
		
		//compute heading
		if (dY <= 0 && dX < 0) { //ok
			heading = -(Math.PI/2.0 + Math.atan(dY/dX));
		}
		else if (dY > 0 && dX <= 0) { //ok
			heading = 3.0*Math.PI/2.0 - Math.atan(dY/dX);
		}
		else if (dY < 0 && dX >= 0) { //TODO figure out trig for this condition
			heading = -(3.0*Math.PI/2.0 - Math.atan(dY/dX));
		}
		else if(dY >= 0 && dX > 0) {		//ok
			heading = Math.PI/2.0 - Math.atan(dY/dX);
		}
		
		//convert negative heading to positive 
		if (heading < 0) {
			heading += 2 * Math.PI;
		}
		
		//System.out.println("\n\n\n\n\n\nheading: " + heading);
		
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(3000);
		}
		
		//turn robot to wanted heading 
		turnTo(heading);
		isNavigating = true;
		
		//travel to x,y
		int rotateAngle = convertDistance(NavigationLab.WHEEL_RADIUS, distance);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(rotateAngle, true);
		rightMotor.rotate(rotateAngle, false);
		
		isNavigating = false;
	}
	
	void turnTo(double theta) {

		double dTheta = theta - odo.getTheta();
		System.out.println("\n\n\n\n\n\ndtheta: " + dTheta);
		
		if (dTheta > Math.PI) {
			dTheta = Math.abs(dTheta - 2 * Math.PI);
			turn(dTheta * 180.0 / Math.PI, "left");
		}
		else {
			turn(dTheta * 180.0 / Math.PI, "right");
		}
		
	}
	
	/**
	 * Turns in given direction.
	 * @param dTheta change in heading wanted, in degrees
	 * @param direction
	 */
	private void turn(double dTheta, String direction) {
		
		int distance = convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, dTheta);
		
		// set motor speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
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

	boolean isNavigating() {
		return isNavigating;
	}
	
	/**
	 * From Lab 2 sample code
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * From Lab 2 sample code
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	@Override
	public void processUSData(int distance) {
		
		if(distance < 20) {
			leftMotor.stop(true);
			rightMotor.stop(true);
			Sound.beep();
			TextLCD t = LocalEV3.get().getTextLCD();
			t.drawString("Obstacle encountered", 0, 0);
			Button.waitForAnyPress();
			System.exit(0);
		}
		
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return 0;
	}

	public double getWaypointX() {
		return waypointX;
	}

	public double getWaypointY() {
		return waypointY;
	}

}
