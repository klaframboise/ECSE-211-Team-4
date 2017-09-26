package ca.mcgill.ecse211.odometerlab;

import lejos.ev3.tools.EV3Console;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x;
	private double y;
	private double theta;
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/
	public static final boolean[] UPDATE_ALL = {true, true, true}; 

	private Object lock; /*lock object for mutual exclusion*/

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();

		// reset motor tachos
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		int leftLastTachoCount, rightLastTachoCount;
		double dLeftWheel, dRightWheel, deltaD, deltaT, dX, dY;


		while (true) {
			updateStart = System.currentTimeMillis();
			// TODO put (some of) your odometer code here

			synchronized (lock) {
				/**
				 * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
				 * and theta in this block. Do not perform complex math
				 * 
				 */
				// keep last iteration's tacho counts
				rightLastTachoCount = rightMotorTachoCount;
				leftLastTachoCount = leftMotorTachoCount;

				// get new tacho counts
				rightMotorTachoCount = rightMotor.getTachoCount();
				leftMotorTachoCount = leftMotor.getTachoCount();

				// find each wheel's displacement
				dLeftWheel = Math.PI * OdometryLab.WHEEL_RADIUS * (leftMotorTachoCount - leftLastTachoCount) / 180;
				dRightWheel = Math.PI * OdometryLab.WHEEL_RADIUS * (rightMotorTachoCount - rightLastTachoCount) / 180;


				deltaD = (dLeftWheel + dRightWheel)/2; // magnitude of displacement
				deltaT = (dLeftWheel - dRightWheel)/OdometryLab.TRACK; // change in heading
				theta = (theta + deltaT) % (2*Math.PI); // update heading
				dX = deltaD * Math.sin(theta); // displacement on x-axis
				dY = deltaD * Math.cos(theta); // displacement on y-axis

				// update coords
				x += dX;
				y += dY;
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;
		}
	}
}
