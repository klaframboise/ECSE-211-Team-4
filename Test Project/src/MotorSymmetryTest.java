import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class MotorSymmetryTest {

	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	public static void main(String[] args) {
		
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		leftMotor.forward();
		rightMotor.forward();

	}

}
