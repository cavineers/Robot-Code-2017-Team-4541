package org.usfirst.frc.team4541.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearMagic {
	AHRS gyro;
	static NetworkTable table;
	AngleController control;
	NetworkTable gripTable;
	MecanumDrive drive;
	double error = 2.0;
	int countTillBreak = 50;
	private static double targetwidth = 8.25; // Target Center to Center
												// Distance from FRC field
												// configuration
	
	public static final int X_OBJ = 0;
	public static final int Y_OBJ = 3;
	public double xError = 0;
	public double yError = 0;
	private static double cva = 68.5; // Camera viewing angle from camera specs
	static double pix_range = 480; // Pixel count of camera viewing field
	static int[] stationdeg = { -45, 0, 45 };
	double kP = 0.04;
	double xP = 0;
	double yP = 0;
	Joystick s;
	public GearMagic(AHRS g, AngleController controller, NetworkTable table, MecanumDrive d, Joystick stick) {
		gyro = g;
		control = controller;
		gripTable = table;
		drive = d;
		SmartDashboard.putNumber("GearII X R", 0.04);
		SmartDashboard.putNumber("GearII X P", 0.45);
		SmartDashboard.putNumber("GearII Y P", 0);
		s = stick;
	}

	int rangeXCounter = 0;
	int rangeYCounter = 0;
	int maxDistance   = 12 * 4; //should work about 4 feet away from wall;
	public void placeGear() {
		int station = getStationID();
		if (station == -1) return;
		this.updateP();
		while (!s.getRawButton(8)) {
			this.updateP();
			double[] centerxs = gripTable.getNumberArray("centerX", new double[] { 0, 0, 0 });
			double[] centerys = gripTable.getNumberArray("centerY", new double[] { 0, 0, 0 });
			double centerX = getAvg(centerxs);
			if (centerX == 0) break;
			//Get X speed required
			xError = centerX - X_OBJ;
			double XSpeed = mapToXMotor(centerX);
			XSpeed *= xP;

			//Get Y speed required
			double YDist = getYDistance(1, centerxs[0], centerxs[1]);
			SmartDashboard.putNumber("Y DISTANCE:", YDist);
			double YSpeed = mapToYMotor(YDist);
			YSpeed *= yP;
			
			double[] area = NetworkTable.getTable("GRIP/myContoursReport").getNumberArray("area", new double[] { 0, 0, 0 });
			if (area.length != 2) break;
			double areaTotal = area[0] + area[1];
			if (areaTotal > 1000) {
				YSpeed -= 0.45;
			} else {
				YSpeed -= 0.6;
			}
			
			//get R speed required
			double angle = gyro.getYaw();
			if (station == 1) angle -= 60;
			else if (station == 2) angle += 60;
//			angle -= rotatedAngle; //USE THIS TO PLACE TO PEGS OTHER THAN THE CENTER
//			angle -= 60;
			//Drive at suggested speed
			drive.drive(XSpeed, angle * -kP, YSpeed);
		}
		drive.updateDefaultAngle(gyro);
	}


	public boolean isWithinRange(double value) {
		return (Math.abs(value) < xError);
	}
	
	public int getStationID() {
		if (Math.abs(gyro.getYaw()) < 25) return 0;
		if (Math.abs(gyro.getYaw() - 60) < 25) return 1;
		if (Math.abs(gyro.getYaw() + 60) < 25) return 2;
		else return -1;
	}

	public boolean isWithinYRange(double value) {
		return (Math.abs(value) - Y_OBJ) > error;
	}
	public double getAvg(double[] centers) {
		if (centers.length != 2) return 0;
		return (centers[0] + centers[1]) / 2;
	}

	public double getYDistance(int station, double x1, double x2) { //Returns distance to wall in inches
		double awd = targetwidth / 2 / Math.tan(cva / 2 * Math.PI / 180); // actual
		
		double pix_range = 480; // grip x pixels output range
		double cva = 68.5; //camera field of view
		double distance; //distance from wall	inches														
		double targetwidth = 8.25;		

		double ax1 = Math.atan(Math.tan(((cva / 2) * Math.PI) / 180) / (pix_range / 2) * (x1 - pix_range / 2)) * 180.0 / Math.PI;
		System.out.println("AX1 =" + ax1);
		double ax2 = Math.atan(Math.tan((cva / 2) * Math.PI / 180) / (pix_range / 2) * (x2 - pix_range / 2)) * 180
				/ Math.PI;
		System.out.println("AX2 =" + ax2);
		double a = targetwidth * Math.sin((90 - ax2) * Math.PI / 180)
				/ Math.sin((90 + ax2) * Math.PI / 180);
		double b = a * Math.sin((90 - ax2) * Math.PI / 180) / Math.sin((ax2 - ax1) * Math.PI / 180);
		distance = b * Math.cos(ax1);
		return distance;
	}
	
	public void updateP() {
		kP = SmartDashboard.getNumber("GearII X R", 0);
		xP = SmartDashboard.getNumber("GearII X P", 0);
		yP = SmartDashboard.getNumber("GearII Y P", 0);
	}
	public double mapToXMotor(double x) {
		if (x > 302) x = 302;
		if (x < 22) x = 22;
		return (0.0071428571*x - 1.157142857);
	}
	public double mapToYMotor(double y) {
		if (y < 0) y = 0;
		if (y > 48) y = 48;
		return (y / 48);
	}
	public boolean canSeeContours() {
		return gripTable.getNumberArray("centerX", new double[] { 0, 0, 0 }).length == 2;
	}
}
