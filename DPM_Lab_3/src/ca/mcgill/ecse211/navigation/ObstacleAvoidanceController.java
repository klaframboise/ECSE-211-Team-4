package ca.mcgill.ecse211.navigation;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class ObstacleAvoidanceController extends Thread {

	private static final int BAND_CENTER = 10;
	private static final int BAND_WIDTH = 5;
	private static final int CORRECTION = 75;
	private static final int NAV_SENSOR_ANGLE = 0;

	private SampleProvider us;
	private float[] usData;
	private EV3LargeRegulatedMotor sensorMotor, rightMotor, leftMotor;
	private Odometer odo;
	private boolean isAvoiding;
	private Navigation nav;
	private int distance;
	private Object lock;

	public ObstacleAvoidanceController(EV3LargeRegulatedMotor sensorMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor, Navigation nav, SampleProvider us, float[] usData, Odometer odo) {
		this.us = us;
		this.usData = usData;
		this.sensorMotor = sensorMotor;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.nav = nav;
		this.odo = odo;
		isAvoiding = false;
		distance = 0;
		lock = new Object();

		sensorMotor.setSpeed(50);
	}

	public void run() {

		int dirModifier = -1;
		double startAvoidHeading = 0;

		while(true) {
			synchronized(lock) {
				// get distance measured by sensor
				distance = getFilteredDistance();

				if(!isAvoiding && distance < (BAND_CENTER - BAND_WIDTH)) {
					nav.pause();
					sensorMotor.rotateTo(NAV_SENSOR_ANGLE + 90, false);
					try {
						Sound.beep();
						Thread.sleep(200);
						Sound.beep();
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					nav.turn(90, "left");
					try {
						Thread.sleep(200);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					startAvoidHeading = odo.getTheta();
					isAvoiding = true;
					continue;
					//dirModifier = 1;
				}

				//avoid by following obstacle
				if(isAvoiding) {
					while(Math.abs(odo.getTheta() - startAvoidHeading) < Math.PI ) {
						distance = getFilteredDistance();
						// machine outside of band center, accelerate outside wheel
						if(distance > BAND_CENTER + BAND_WIDTH) {
							rightMotor.setSpeed(NavigationLab.FORWARD_SPEED + dirModifier * CORRECTION);
							leftMotor.setSpeed(NavigationLab.FORWARD_SPEED - dirModifier * CORRECTION);
							leftMotor.forward();
							rightMotor.forward();
							// machine inside of band center, accelerate inside wheel
						} else if(distance < BAND_CENTER - BAND_WIDTH) {
							rightMotor.setSpeed(NavigationLab.FORWARD_SPEED - dirModifier * CORRECTION);
							leftMotor.setSpeed(NavigationLab.FORWARD_SPEED + dirModifier * CORRECTION);
							leftMotor.forward(); 
							rightMotor.forward();
							// machine within band
						} else {
							rightMotor.setSpeed(NavigationLab.FORWARD_SPEED);
							rightMotor.setSpeed(NavigationLab.FORWARD_SPEED);
							leftMotor.forward();
							rightMotor.forward();
						}
						try {
							Thread.sleep(25);
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					if(Math.abs(odo.getTheta() - startAvoidHeading) < Math.PI) Sound.buzz();
				}

				// if distance is large enough, could be done avoiding
				// try navigating again
				if(isAvoiding) {
					rightMotor.stop(true);
					leftMotor.stop(false);
					sensorMotor.rotateTo(NAV_SENSOR_ANGLE, false);
					isAvoiding = false;
					nav.resumeNav();
				}

				// sleep for 50ms
				try {
					Thread.sleep(250);
				} catch (Exception e) {
				} 
			}
		}
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

	public int getDistance() {
		return distance;
	}
}
