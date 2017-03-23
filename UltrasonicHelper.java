package org.usfirst.frc.team4541.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UltrasonicHelper {
	public static final int MAX_UID = 6;
	MecanumDrive drive;
	AHRS gyro;
	
	// Change based on what sensors are on what side of the robot
	final static int U_FRONT = 3, U_BACK = 2, U_RIGHT = 1, U_LEFT = 4;
	final static int ERROR = 5; // all distances will be within 3 cm
	final static int SLOWDOWN = 7;
	final static int WITHIN_RANGE = 100; // the robot must be within range for
											// 100 loops until it is considered
											// valid

	public UltrasonicHelper(MecanumDrive d, AHRS g) {
		drive = d;
		gyro = g;
	}

	public void goToUltrasonicDistance(double distance, SIDE side) {
		int rangeCounter = 0;
		if (ultrasonicFromSide(side) != -1) {
			int ultrasonicID = ultrasonicFromSide(side);
			while (true) {
				while (!ArduinoDueInterface.isDueOnline()) {
					SmartDashboard.putString("AUTO STATUS: ", "ERROR CONNECTING TO ULTRASONIC NETWORK!! HALTING AUTO!!!");
					SmartDashboard.putBoolean("DUE ONLINE", ArduinoDueInterface.isDueOnline());
					ArduinoDueInterface.resetDue();
				} 
				SmartDashboard.putString("AUTO STATUS: ", "Nominal");
				double current = ArduinoDueInterface.getUltrasonic(ultrasonicID);
				SmartDashboard.putBoolean("DUE ONLINE", ArduinoDueInterface.isDueOnline());
				if (isWithinRange(distance, current)) {
					if (rangeCounter >= WITHIN_RANGE)
						break;
					rangeCounter++;
					continue;
				}
				if (isWithinSlowDown(distance, current)) {
					// Slow the robot down a bit as it gets closer to the target
					if (distance < current)
						drive.straightDrive(getVerticalMovement(ultrasonicID) / 2, 0,
								getHorizontalMovement(ultrasonicID) / 2, gyro);
					else
						drive.straightDrive(-1 * getVerticalMovement(ultrasonicID) / 2, 0,
								-1 * getHorizontalMovement(ultrasonicID) / 2, gyro);
				} else {
					if (distance < current)
						drive.straightDrive(getVerticalMovement(ultrasonicID), 0, getHorizontalMovement(ultrasonicID),
								gyro);
					else
						drive.straightDrive(-1 * getVerticalMovement(ultrasonicID), 0, -1 * getHorizontalMovement(ultrasonicID), gyro);
				}
			}
		}
	}

	private static boolean isValid(int UID) {
		return (UID > 0 && UID <= MAX_UID) ? true : false;
	}

	private static double getHorizontalMovement(int UID) {
		switch (UID) {
		case U_RIGHT: {
			return 0.4;
		}
		case U_LEFT: {
			return -0.4;
		}
		default: {
			return 0;
		}
		}
	}

	private static double getVerticalMovement(int UID) {
		switch (UID) {
		case U_FRONT: {
			return 0.4;
		}
		case U_BACK: {
			return -0.4;
		}
		default: {
			return 0;
		}
		}
	}

	private static boolean isWithinRange(double requested, double current) {
		return (Math.abs(requested - current) <= ERROR);
	}

	private static boolean isWithinSlowDown(double requested, double current) {
		return (Math.abs(requested - current) <= SLOWDOWN);
	}

	private static int ultrasonicFromSide(SIDE side) {
		switch (side) {
		case FRONT: {
			return U_FRONT;
		}
		case BACK: {
			return U_BACK;
		}
		case LEFT: {
			return U_LEFT;
		}
		case RIGHT: {
			return U_RIGHT;
		}
		default: {
			return -1;
		}
		}
	}
}