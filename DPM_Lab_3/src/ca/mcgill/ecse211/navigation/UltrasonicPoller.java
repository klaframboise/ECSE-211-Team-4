package ca.mcgill.ecse211.navigation;

import java.util.Arrays;

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
	private SampleProvider us;
	private UltrasonicController cont;
	private float[] usData;

	public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {
		this.us = us;
		this.cont = cont;
		this.usData = usData;
	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * Sensor data is filtered. Will return the median of a set of samples
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		int distance;
		while (true) {
			
			for(int i = 0; i < usData.length; i+= us.sampleSize()) {
				us.fetchSample(usData, i * us.sampleSize()); // acquire data
			}
			Arrays.sort(usData);	// sort array
			distance = (int) ((usData[(usData.length/2)-1] + usData[usData.length/2]) / 2.0 * 100.0); // return median
			cont.processUSData(distance); // now take action depending on value
			try {
				Thread.sleep(50);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

}
