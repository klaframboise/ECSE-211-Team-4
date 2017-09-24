/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometerlab;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {

	/* Constants */
	private static final long CORRECTION_PERIOD = 10;
	private static final int SAMPLE_SIZE = 10;
	// TODO measure reflectivity of line
	private static final float LINE_RED_INTENSITY = 0.25f;
	private static final float GRID_SIZE = 30.48f;

	private Odometer odometer;
	public static SensorModes colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	public static SampleProvider colorSampler = colorSensor.getMode("Red");

	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		double[] lastPos = new double[3];
		double[] pos = new double[3];
		boolean update[] = {true, true, false};
		char lastDir = 'N';
		int counter = 0;

		while (true) {
			correctionStart = System.currentTimeMillis();

			if(getRedIntensity() < LINE_RED_INTENSITY) {

				// increase line count
				counter++;
				//play sound
				Sound.beep();

				if ((lastDir != SquareDriver.getDirection()) || counter == 1) { 
					odometer.getPosition(lastPos, Odometer.UPDATE_ALL);
					lastDir = SquareDriver.getDirection();
				}
				// correct according to the direction
				else {
					switch(SquareDriver.getDirection()) {
					case 'N': 
						pos[1] = lastPos[1] + GRID_SIZE;
						pos[0] = lastPos[0] + GRID_SIZE * Math.tan(odometer.getTheta());
						break;
					case 'E': 
						pos[0] = lastPos[0] + GRID_SIZE; 
						pos[1] = lastPos[1] + GRID_SIZE * Math.tan(90 - odometer.getTheta());
						break;
					case 'S': 
						pos[1] = lastPos[1] - GRID_SIZE; 
						pos[0] = lastPos[0] + GRID_SIZE * Math.tan(odometer.getTheta());
						break;
					case 'W': 
						pos[0] = lastPos[0] - GRID_SIZE;
						pos[1] = lastPos[1] + GRID_SIZE * Math.tan(90 - odometer.getTheta());
						break;
					}

					//update odo
					odometer.setPosition(pos, update);

					//save line pos for next correction
					lastPos = Arrays.copyOf(pos, 3);
				}
			}
			// compute magnitude of displacement since last correction


			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
	
	/**
	 * Returns the mean of a group of sample from the color sensor as the detected color.
	 * @return color detected
	 */
	private float getRedIntensity() {

		float[] samples = new float[SAMPLE_SIZE];

		for(int i = 0; i < SAMPLE_SIZE; i++) {
			colorSampler.fetchSample(samples, i);
		}

		Arrays.sort(samples);

		return (samples[SAMPLE_SIZE/2] + samples[(SAMPLE_SIZE/2) + 1]) / 2.0f;
	}
}
