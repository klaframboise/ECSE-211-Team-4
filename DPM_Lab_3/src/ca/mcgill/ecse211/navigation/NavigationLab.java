package ca.mcgill.ecse211.navigation;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationLab {

	public static final int FORWARD_SPEED = 250;
	public static final int ROTATE_SPEED = 150;
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 13.5;
	public static final int SAMPLE_SIZE = 10;
	public static final double GRID_SIZE = 30.48;
	
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
			
	public static void main(String[] args) {

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		float[] usData = new float[usDistance.sampleSize() * SAMPLE_SIZE]; // usData is the buffer in which data are
		// returned
		UltrasonicPoller usPoller = null;
		TextLCD t = LocalEV3.get().getTextLCD();
		
		// clear the display
		t.clear();
		t.drawString("Press any button", 0, 0);
		t.drawString("    to start    ", 0, 1);
		Button.waitForAnyPress();
		
		
		//System.out.println("\n\nbutton pressed");
		// start odometer and display

		Odometer odo = new Odometer(leftMotor, rightMotor);
		//OdometryCorrection correction = new OdometryCorrection(odo);
		Navigation nav = new Navigation(odo, leftMotor, rightMotor);
		ObstacleAvoidanceController cont = new ObstacleAvoidanceController(sensorMotor, rightMotor, leftMotor, nav, usDistance, usData, odo);
		NavigationDisplay navDisplay = new NavigationDisplay(odo, nav, cont, t);
		
		odo.start();
		// correction not required, may be removed if causing trouble
		//correction.start();
		cont.start();
		nav.start();
		navDisplay.start();

	}

}
