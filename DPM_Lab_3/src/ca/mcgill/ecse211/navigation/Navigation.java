package ca.mcgill.ecse211.navigation;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation implements UltrasonicController{
	
	private Odometer odo;
	private EV3LargeRegulatedMotor leftMotor, rightMotor, sensorMotor;
	private boolean isNavigating;
	private double waypointX;
	private double waypointY;
	private int distance;
	
	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor sensorMotor){
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;
		isNavigating = false;
		waypointX = 0;
		waypointY = 0;
		sensorMotor.setSpeed(50);
	}
	
	void travelTo(double x, double y) {
		
		// if currently navigating, don't change destination waypoint
		waypointX = (isNavigating())? waypointX : x;
		waypointY = (isNavigating())? waypointY : y;
		isNavigating = true;
		
		double dX = x - odo.getX();
		double dY = y - odo.getY();
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
		System.out.println("\n\n\n\n\n\n\nheading: " + (int)Math.toDegrees(heading));
		
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
		leftMotor.setSpeed(NavigationLab.FORWARD_SPEED);
		rightMotor.setSpeed(NavigationLab.FORWARD_SPEED);
		leftMotor.rotate(rotateAngle, true);
		rightMotor.rotate(rotateAngle, false);
		isNavigating = false;
	}
	
	void turnTo(double theta) {

		double dTheta = theta - odo.getTheta();
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
		
		int distance = convertAngle(NavigationLab.WHEEL_RADIUS, NavigationLab.TRACK, dTheta);
		
		// set motor speed
		leftMotor.setSpeed(NavigationLab.ROTATE_SPEED);
		rightMotor.setSpeed(NavigationLab.ROTATE_SPEED);
		
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
	
	void pause() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	
	void resume() {
		travelTo(waypointX, waypointY);
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

	public double getWaypointX() {
		return waypointX;
	}

	public double getWaypointY() {
		return waypointY;
	}

	@Override
	public void processUSData(int distance) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return 0;
	}

}
