package ca.mcgill.ecse211.localization;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {

	//private int SWEEP_SIZE = 30;
	private SampleProvider us;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private float[] sweepData;
	private double alphaAngle, betaAngle, deltaTheta, theta;
	private boolean goingClockwise;
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
		goingClockwise = true;

	}

	//	private void sweep(boolean clockwise, int sweepSize) {
	//		// compute wheel rotation amount
	//		int rotationAngle = LocalizationLab.convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, 360.0/sweepSize);
	//
	//		//for(int i = 0; i < sweepSize; i++) {
	//
	//		// gather us data at current heading
	//		//sweepData[i] = getFilteredDistance();
	//
	//		// position robot for next measurement
	//		if(clockwise) {
	//			leftMotor.rotate(rotationAngle, true);
	//			rightMotor.rotate(-rotationAngle, false);
	//		}else {
	//			leftMotor.rotate(-rotationAngle, true);
	//			rightMotor.rotate(rotationAngle, false);
	//		}
	//
	//		//}
	//	}

	public void fallingEdge() {
		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		currentDistance = getFilteredDistance();
		if(currentDistance < 30) { //in case we start facing the walls
			while(currentDistance < 30) { //then move away from the walls in a clockwise manner
				currentDistance = getFilteredDistance();
				leftMotor.forward();
				rightMotor.backward();
			}
//			try {
//				Thread.sleep(3000); //leave some time for the robot to move away so not to disturb the sensor
//			} catch (InterruptedException e) {
//				e.printStackTrace();
//			}
			currentDistance = getFilteredDistance(); 
			while(currentDistance > 30  && goingClockwise) { //now look for the closest wall clockwise
				currentDistance = getFilteredDistance();
				//sweep(clockwise, 72);
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.setSpeed(0); //stop and record the alpha angle once detected first falling edge
			rightMotor.setSpeed(0);
			goingClockwise = false;
			alphaAngle = odo.getTheta();
			leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED); //go counter counterclockwise
			rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			Sound.beep();
			try {											//keep going counterclockwise for at least 3 sec
				Thread.sleep(3000); //give time for motors to move away from back wall
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance > 30 && !goingClockwise) { //we're now looking for another falling edge on the other side
				currentDistance = getFilteredDistance();
				//sweep(clockwise, 72);
				leftMotor.backward(); //we're going counterclockwise
				rightMotor.forward();
			}
			leftMotor.setSpeed(0); //stop motors and record beta angle once found the second falling edge
			rightMotor.setSpeed(0);
			betaAngle = odo.getTheta();
		} else { //this is in case we start away from the walls
			while(currentDistance > 30  && goingClockwise) { //look for the closest wall clockwise
				currentDistance = getFilteredDistance();
				//sweep(clockwise, 72);
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.setSpeed(0); //stop and record the alpha angle once detected first falling edge
			rightMotor.setSpeed(0);
			goingClockwise = false;
			alphaAngle = odo.getTheta();
			leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED); //go counter counterclockwise
			rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			Sound.beep();
			//sweep(clockwise, 4);
			try {
				Thread.sleep(3000); //give time for motors to move away from back wall
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance > 30 && !goingClockwise) { //we're now looking for another falling edge on the other side
				currentDistance = getFilteredDistance();
				//sweep(clockwise, 72);
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep();
			leftMotor.setSpeed(0); //stop motors and record beta angle
			rightMotor.setSpeed(0);
			leftMotor.forward();
			rightMotor.backward();
			betaAngle = odo.getTheta();
			Sound.beep();
		}
		if (alphaAngle > betaAngle) {
			deltaTheta = Math.PI/4.0 - (alphaAngle + betaAngle)/2.0;
		}
		else {
			deltaTheta = (5*Math.PI)/4.0 - (alphaAngle + betaAngle)/2.0;
		} //0.1 used for offset
		double currentTheta = odo.getTheta();
		odo.setTheta(currentTheta + deltaTheta); //correct the odometer's theta value to the correct one
	}

	public void risingEdge() {
		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		currentDistance = getFilteredDistance();
		if(currentDistance < 40) { //in case we start facing walls
			while(currentDistance < 40 && goingClockwise) { //keep going clockwise until away from the walls
				currentDistance = getFilteredDistance();
				//sweep(goingClockwise, 120);
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.setSpeed(0);
			rightMotor.setSpeed(0);
			goingClockwise = false;
			alphaAngle = odo.getTheta(); //detected first rising edge and record the alpha angle
			//sweep(goingClockwise, 4);
			leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
			rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			try {
				Thread.sleep(2000); //leave time for robot to look at the walls again
			}catch(InterruptedException e) {
				e.printStackTrace();
			}
			Sound.beep();
			currentDistance = getFilteredDistance();
			while(currentDistance < 40 && !goingClockwise) { //keep going counterclockwise to find second rising edge
				currentDistance = getFilteredDistance();
				//sweep(goingClockwise, 120);
				leftMotor.backward();
				rightMotor.forward();
			}
			leftMotor.setSpeed(0); //stop motors
			rightMotor.setSpeed(0);
			betaAngle = odo.getTheta(); //record second angle for second rising edge
		} else {
			while(currentDistance > 40) { //if we start facing away from the walls
				currentDistance = getFilteredDistance();
				//sweep(goingClockwise, 120);
				leftMotor.forward(); //then keep going clockwise
				rightMotor.backward();
			}
			try {
				Thread.sleep(5000); //give 5sec for the robot to start facing the walls
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance < 40 && goingClockwise) { //while still facing the walls, keep going clockwise
				currentDistance = getFilteredDistance();
				//sweep(goingClockwise, 120);
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.setSpeed(0); //stop the motors once we detected first rising edge
			rightMotor.setSpeed(0);
			goingClockwise = false;
			alphaAngle = odo.getTheta(); //record first angle for first rising edge
			//sweep(goingClockwise, 4);
			leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED); //go counter clockwise now 
			rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			try {
				Thread.sleep(2000); //give robot 3sec to start facing walls again
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance < 40 && !goingClockwise) { //keep going counterclockwise until we detect second rising edge
				currentDistance = getFilteredDistance();
				//sweep(goingClockwise, 120);
				leftMotor.backward();
				rightMotor.forward();
			}
			leftMotor.setSpeed(0); //stop motors once detected second rising edge
			rightMotor.setSpeed(0);
			betaAngle = odo.getTheta(); //record the angle for the second rising edge
		}
		if (alphaAngle < betaAngle) {
			deltaTheta = Math.PI/4.0 - (alphaAngle + betaAngle)/2.0;
		}
		else {
			deltaTheta = (5*Math.PI)/4 - (alphaAngle + betaAngle)/2 - 0.2;
		}
		double currentTheta = odo.getTheta();
		odo.setTheta(currentTheta + deltaTheta); //TODO recheck value of this
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
