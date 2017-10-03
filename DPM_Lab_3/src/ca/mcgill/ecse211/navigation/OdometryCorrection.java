/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.navigation;

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
	private static final float LINE_RED_INTENSITY = 0.25f;

	private Odometer odo;
	public static SensorModes colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	public static SampleProvider colorSampler = colorSensor.getMode("Red");

	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odo = odometer;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		double gridX = 0;
		double gridY = 0;

		while (true) {
			correctionStart = System.currentTimeMillis();

			if(getRedIntensity() < LINE_RED_INTENSITY) {

				// get grid coordinate
				gridX = odo.getX() / NavigationLab.GRID_SIZE;
				gridY = odo.getY() / NavigationLab.GRID_SIZE;
				
				// assuming odometer is accurate within 3cm
				if((gridX % 1 < 0.1 || gridX % 1 > 0.9) || (gridX % 1 < gridY % 1)) {
					odo.setX((int)gridX * NavigationLab.GRID_SIZE);
				}
				// assuming odometer is accurate within 3cm
				if((gridY % 1 < 0.1 || gridY % 1 > 0.9) || (gridY % 1 < gridX % 1)) {
					odo.setY((int)gridY * NavigationLab.GRID_SIZE);
				}
			
			}

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
