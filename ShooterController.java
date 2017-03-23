package org.usfirst.frc.team4541.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterController {
	public boolean isSecondLoop = false;
	AHRS ahrs;
	PIDController turnController;
	MecanumDrive myRobot;
	double rotateToAngleRate;
	double p1, i1, d1, f1;
	double p2, i2, d2, f2;
	static final double kToleranceDegrees = 1.0f;
	CANTalon T1, T2;
	public ShooterController(CANTalon t1, CANTalon t2) {
		t1.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		t1.reverseSensor(false);
		t1.configNominalOutputVoltage(+0.0f, -0.0f);
		t1.configPeakOutputVoltage(+12.0f, -12.0f);
		t1.setProfile(0);
		t1.setF(0.1097);
		t1.setP(0.22);
		t1.setI(0);
		t1.setD(0);
		
		t2.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		t2.reverseSensor(false);
		t2.configNominalOutputVoltage(+0.0f, -0.0f);
		t2.configPeakOutputVoltage(+12.0f, -12.0f);
		t2.setProfile(0);
		t2.setF(0.1097);
		t2.setP(0.22);
		t2.setI(0);
		t2.setD(0);
		
		SmartDashboard.putNumber("Shooter1 P", 0.22);
		SmartDashboard.putNumber("Shooter1 I", 0);
		SmartDashboard.putNumber("Shooter1 D", 0);
		SmartDashboard.putNumber("Shooter1 f", 0.1097);
		
		SmartDashboard.putNumber("Shooter2 P", 0.22);
		SmartDashboard.putNumber("Shooter2 I", 0);
		SmartDashboard.putNumber("Shooter2 D", 0);
		SmartDashboard.putNumber("Shooter2 f", 0.1097);
		T1 = t1;
		T2 = t2;
	} 
	
	public void setSpeedPID(double speed) {
		T1.changeControlMode(TalonControlMode.Speed);
		T1.set(-speed);
		
		T2.changeControlMode(TalonControlMode.Speed);
		T2.set(speed);
		
		SmartDashboard.putNumber("T1 PID Error", T1.getError());
		SmartDashboard.putNumber("T2 PID Error", T2.getError());
	}
	
	public void setSpeed(double speed) {
		T1.changeControlMode(TalonControlMode.PercentVbus);
		T1.set(-speed);
		
		T2.changeControlMode(TalonControlMode.PercentVbus);
		T2.set(speed);
	}
	
	public void updatePIDf() {
		p1 = SmartDashboard.getNumber("Shooter1 P", 0.22);
		i1 = SmartDashboard.getNumber("Shooter1 I", 0);
		d1 = SmartDashboard.getNumber("Shooter1 D", 0);
		f1 = SmartDashboard.getNumber("Shooter1 f", 0.1097);
		
		T1.setPID(p1, i1, d1);
		T1.setF(f1);
		p1 = SmartDashboard.getNumber("Shooter2 P", 0.22);
		i1 = SmartDashboard.getNumber("Shooter2 I", 0);
		d1 = SmartDashboard.getNumber("Shooter2 D", 0);
		f1 = SmartDashboard.getNumber("Shooter2 f", 0.1097);
		
		T2.setPID(p1, i1, d1);
		T2.setF(f1);
	}
	
	
	
	
	
}


