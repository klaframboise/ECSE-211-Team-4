package ca.mcgill.ecse211.localization;

import java.util.Arrays;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {

	//private int SWEEP_SIZE = 30;
	private SampleProvider us;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private float[] sweepData;
	private double alphaAngle, betaAngle, deltaTheta, theta;
	private boolean clockwise = true;
	private float currentDistance;
	private Odometer odo;

	public UltrasonicLocalizer(SampleProvider us, float[] usData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, Odometer odo) {
		this.us = us;
		this.usData = usData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		//sweepData = new float[SWEEP_SIZE];
		this.odo = odo;

	}

	private void sweep(boolean clockwise, int sweepSize) {
		// compute wheel rotation amount
		int rotationAngle = LocalizationLab.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360.0/sweepSize);

		//for(int i = 0; i < SWEEP_SIZE; i++) {

		// gather us data at current heading
		//sweepData[i] = getFilteredDistance();

		// position robot for next measurement
		if(clockwise) {
			leftMotor.rotate(rotationAngle, true);
			rightMotor.rotate(-rotationAngle, false);
		}else {
			leftMotor.rotate(-rotationAngle, true);
			rightMotor.rotate(rotationAngle, false);
		}

		//}
	}

	public void fallingEdge() {
		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		currentDistance = getFilteredDistance();
		if(currentDistance > 30) { //in case we start away from the walls
			while(currentDistance > 30  && clockwise) {
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			clockwise = false;
			alphaAngle = odo.getTheta();
			sweep(clockwise, 4);
			try {
				Thread.sleep(3000); //give time for motors to move away from back wall
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance > 30 && !clockwise) { //we're now looking for another falling edge on the other side
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			betaAngle = odo.getTheta();
		} else {
			while(currentDistance < 30) {
				sweep(clockwise, 4);
			}
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance(); //same code as above
			while(currentDistance > 30  && clockwise) {
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			clockwise = false;
			alphaAngle = odo.getTheta();
			sweep(clockwise, 4);
			try {
				Thread.sleep(3000); //give time for motors to move away from back wall
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance > 30 && !clockwise) { //we're now looking for another falling edge on the other side
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			betaAngle = odo.getTheta();
		}
		deltaTheta = Math.PI/4 - (alphaAngle + betaAngle)/2;
		double currentTheta = odo.getTheta();
		odo.setTheta(currentTheta + deltaTheta);
		LocalizationLab.turnTo(0);
	}

	public void risingEdge() {
		currentDistance = getFilteredDistance();
		if(currentDistance < 40) { //in case we start facing walls
			while(currentDistance < 40 && clockwise) {
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			clockwise = false;
			alphaAngle = odo.getTheta();
			sweep(clockwise, 4);
			try {
				Thread.sleep(3000); //wait for robot to stabilize
			}catch(InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance < 40 && !clockwise) {
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			betaAngle = odo.getTheta();
		} else {
			while(currentDistance > 40) {
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance < 40 && clockwise) {
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			clockwise = false;
			alphaAngle = odo.getTheta();
			sweep(clockwise, 4);
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance < 40 && !clockwise) {
				currentDistance = getFilteredDistance();
				sweep(clockwise, 120);
			}
			betaAngle = odo.getTheta();
		}
		deltaTheta = (5*Math.PI)/4 - (alphaAngle + betaAngle)/2;
		double currentTheta = odo.getTheta();
		odo.setTheta(currentTheta + deltaTheta);
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
