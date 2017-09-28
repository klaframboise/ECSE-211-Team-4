package ca.mcgill.ecse211.navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationLab {

	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 13.5;
	public static final int SAMPLE_SIZE = 10;

	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	public static void main(String[] args) {

		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		float[] usData = new float[usDistance.sampleSize() * SAMPLE_SIZE]; // usData is the buffer in which data are
		// returned

	}

}
