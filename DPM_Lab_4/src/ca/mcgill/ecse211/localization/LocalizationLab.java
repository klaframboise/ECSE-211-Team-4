package ca.mcgill.ecse211.localization;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationLab {

	public static final int FORWARD_SPEED = 100;
	public static final int ROTATE_SPEED = 150;
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 13.95;
	public static final int SAMPLE_SIZE = 10;
	public static final double GRID_SIZE = 30.48;

	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port lightPort = LocalEV3.get().getPort("S4");
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static Odometer odo;

	public static void main(String[] args) {
		int buttonChoice;
		TextLCD t = LocalEV3.get().getTextLCD();
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from this instance
		float[] usData = new float[usDistance.sampleSize() * SAMPLE_SIZE]; // usData is the buffer in which data are
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(lightPort);
		SampleProvider colorSampler = colorSensor.getMode("Red");
		float[] lightData = new float[colorSampler.sampleSize() * SAMPLE_SIZE];
		odo = new Odometer(leftMotor, rightMotor);
		OdometryDisplay display = new OdometryDisplay(odo, t);
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(usDistance, usData, leftMotor, rightMotor, odo);
		LightLocalizer lightLocalizer = new LightLocalizer(colorSampler, lightData, leftMotor, rightMotor);

		do {
			// clear the display
			t.clear();

			// ask the user whether to use the rising edge or falling edge
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString("Falling| Rising ", 0, 2);
			t.drawString(" Edge  |  Edge  ", 0, 3);
			t.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// start odometry
		odo.start();
		display.start();
		
		// localize according to user choice
		/*if(buttonChoice == Button.ID_LEFT) usLocalizer.fallingEdge(); 
		else usLocalizer.risingEdge();*/
		//TODO Tell navigation to orient to 0 degrees using odo
		
		// wait for any press to begin light localization
		Button.waitForAnyPress();
		lightLocalizer.localize();
		
	}

	public static Odometer getOdo() {
		return odo;
	}
	
	static void travelTo(double x, double y) {

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

	static void turnTo(double theta) {

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
	static void turn(double dTheta, String direction) {

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
	 * From Lab 2 sample code
	 */
	static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * From Lab 2 sample code
	 */
	static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
