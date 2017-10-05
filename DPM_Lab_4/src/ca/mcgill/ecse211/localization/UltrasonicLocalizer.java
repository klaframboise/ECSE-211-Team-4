package ca.mcgill.ecse211.localization;

import java.util.Arrays;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {

	private static final int SWEEP_SIZE = 30;
	private SampleProvider us;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private float[] sweepData;

	public UltrasonicLocalizer(SampleProvider us, float[] usData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.us = us;
		this.usData = usData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		sweepData = new float[SWEEP_SIZE];
	}

	private void sweep() {

		// compute wheel rotation amount
		int rotationAngle = LocalizationLab.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360.0/SWEEP_SIZE);

		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);

		for(int i = 0; i < SWEEP_SIZE; i++) {

			// gather us data at current heading
			sweepData[i] = getFilteredDistance();

			// position robot for next measurement
			leftMotor.rotate(rotationAngle, true);
			rightMotor.rotate(-rotationAngle, false);
		}
	}

	public void fallingEdge() {
		sweep();
	}

	public void risingEdge() {
		sweep();
	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * Sensor data is filtered. Will return the median of a set of samples
	 * 
	 * @see java.lang.Thread#run()
	 */
	private int getFilteredDistance() {

		for(int i = 0; i < usData.length; i+= us.sampleSize()) {
			us.fetchSample(usData, i * us.sampleSize()); // acquire data
		}
		Arrays.sort(usData);	// sort array
		return (int) ((usData[(usData.length/2)-1] + usData[usData.length/2]) / 2.0 * 100.0); // return median
	}
}
