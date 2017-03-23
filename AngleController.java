package org.usfirst.frc.team4541.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AngleController implements PIDOutput {
	public boolean isSecondLoop = false;
	AHRS ahrs;
	Joystick stick;
	PIDController turnController;
	MecanumDrive myRobot;
	double rotateToAngleRate;
	double P1, I1, D1, P2, I2, D2;
	static final double kToleranceDegrees = 1.0f;
	
	public AngleController(AHRS gyro, Joystick s, MecanumDrive dr, double p1, double i1, double d1, double p2, double i2, double d2) {
		ahrs = gyro;
		stick = s;
		turnController = new PIDController(p1, i1, d1, 0, ahrs, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		myRobot = dr;
		P1 = p1;
		I1 = i1;
		D1 = d1;
		P2 = p2;
		I2 = i2;
		D2 = d2;
		SmartDashboard.putNumber("P1: ", p1);
		SmartDashboard.putNumber("I1: ", i1);
		SmartDashboard.putNumber("D1: ", d1);
		SmartDashboard.putNumber("P2 ", p2);
		SmartDashboard.putNumber("I2 ", i2);
		SmartDashboard.putNumber("D2 ", d2);
	} 
	public void turnToAngle(int angle) {
		int targetAmount = 0;
		int waitLength = 100; //how many times the robot checks for on target until breaking loop
		turnController.setSetpoint(angle);
		while (!stick.getRawButton(7)) {
			SmartDashboard.putNumber("angle", standardize(ahrs.getAngle()));
			SmartDashboard.putNumber("error", turnController.getError());
			SmartDashboard.putNumber("avg err", turnController.getAvgError());
			SmartDashboard.putNumber("target Amount", targetAmount);
			double standa = standardize(ahrs.getAngle());
			
			isSecondLoop = (Math.abs(standa - angle) < 20.0);
			if (isSecondLoop) {
				updatePID();
				turnController.setPID(P2, I2, D2);
			} else {
				updatePID();
				turnController.setPID(P1, I1, D1);
			}
			turnController.enable();
			myRobot.drive(0, rotateToAngleRate, 0);
			
			if (Math.abs(turnController.getError()) < 1.0) { //Math.abs(ahrs.getAngle() - angle)
				targetAmount++;
			} else {
				targetAmount = 0;
			}
			if (targetAmount > waitLength)
			{
				turnController.disable();
				break;
			}
			
//			if (Math.abs(ahrs.getAngle() - angle) < 1.0 && turnController.onTarget() && turnController.getError() < 1.0) {
//				
//			}
		}
	}
	public void updatePID() {
		P1 = SmartDashboard.getNumber("P1: ", turnController.getP());
		I1 = SmartDashboard.getNumber("I1: ", turnController.getI());
		D1 = SmartDashboard.getNumber("D1: ", turnController.getD());
		
		P2 = SmartDashboard.getNumber("P2: ", turnController.getP());
		I2 = SmartDashboard.getNumber("I2: ", turnController.getI());
		D2 = SmartDashboard.getNumber("D2: ", turnController.getD());
	
		double f = ahrs.getAngle();
	}
	public static double standardize(double angle) {
		while (angle <= -180 || angle >= 180) {
			if (angle > 0)
				angle -= 360;
			else
				angle += 360;
		}
		return angle;
	}
	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX-MXP yaw angle input and PID Coefficients. */
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}


