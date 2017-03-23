package org.usfirst.frc.team4541.robot;

import java.text.DecimalFormat;
import java.util.Arrays;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.hal.DIOJNI;
import edu.wpi.first.wpilibj.hal.RelayJNI;

//The class that handles SPI communication between arduino due and rio
public class ArduinoDueInterface {
	private static I2C i2c;
	
	public static void init() {
		i2c = new I2C(I2C.Port.kOnboard, 4);
	}
	
	public static double getUltrasonic(int i) {
		try{
			int[] intArr = {255, 255, 255};
			byte[] byteArr = {(byte) i};
			byte[] recievedData = {0x00, 0x00};
			i2c.transaction(byteArr, byteArr.length, recievedData, recievedData.length);
		//	Timer.delay(.01);
			return normalizeBytes(recievedData)/10;
		}
		catch(NullPointerException e){
			System.out.println(e.getMessage());
			return -1;
		}
		
	}
	public static boolean isDueOnline() {
		int[] intArr = {255, 255, 255};
		byte[] byteArr = {(byte) 5};
		byte[] recievedData = {0x00, 0x00};
		i2c.transaction(byteArr, byteArr.length, recievedData, recievedData.length);
		System.out.println(recievedData[0]);
		return recievedData[0] == -1 && recievedData[1] == 0;
	} 
	
	public static double normalizeBytes(byte[] b) {
		int distance = (b[0] & 0xFF) << 8;
		distance = distance | (b[1] & 0xFF);
		
		double distanceFloat = (double) (distance);
		
		distanceFloat = Double.valueOf((new DecimalFormat(".#")).format(distanceFloat)); //A very roundabout way to round to one decimal place
		return distanceFloat; 
	}
	public static void resetDue() {
		
//		int portHandle = RelayJNI.getPort((byte) 0);
//		int forwardHandle = RelayJNI.initializeRelayPort(portHandle, true);
//
//		if (isDueOnline()) {
//			RelayJNI.setRelay(forwardHandle, true);
//		} else {
//			RelayJNI.setRelay(forwardHandle, false);
//			Timer.delay(0.1);
//			RelayJNI.setRelay(forwardHandle, true);
//		}
//		
	}
	

	
}
