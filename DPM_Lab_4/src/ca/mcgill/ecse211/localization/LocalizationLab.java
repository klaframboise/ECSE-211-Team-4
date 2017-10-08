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
	public static final double TRACK = 13.5;
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
		if(buttonChoice == Button.ID_LEFT) usLocalizer.fallingEdge(); 
		else usLocalizer.risingEdge();
		//TODO Tell navigation to orient to 0 degrees using odo
		
		// wait for any press to begin light localization
		Button.waitForAnyPress();
		lightLocalizer.localize();
		
	}

	public static Odometer getOdo() {
		return odo;
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
