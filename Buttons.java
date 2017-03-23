package org.usfirst.frc.team4541.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Buttons {
	/*
	 * FUNCTIONS:
	 * Shooter/Agitator - one button hold + agitator								 - A button
	 * 		- agitator reverse button 												 - Back button (7)
	 * Gear push - opens then closes w/ one button  								 - B button
	 * Gear intake - toggle w/ intake state boolean identifier in SmartDashboard	 - Y button
	 * Climber - press and hold														 - X button
	 * 	
	 */
	 static Joystick stick = new Joystick(0);

	Joystick pad;
	
	public static Button XButton = new JoystickButton(stick, 3);
	public static Button YButton = new JoystickButton(stick, 4);
	public static Button AButton = new JoystickButton(stick, 1);
	public static Button BButton = new JoystickButton(stick, 2);
	public static Button RBumper = new JoystickButton(stick, 5);
	public static Button LBumper = new JoystickButton(stick, 6);
	public static Button backButton = new JoystickButton(stick, 7);
	public static Button startButton = new JoystickButton(stick, 7);
	public static Button lStickPress = new JoystickButton(stick, 9);
	public static Button rStickPress = new JoystickButton(stick, 10);
	
	public static final int SHOOTER 	= 1;
	public static final int A_REVERSE   = 7;
	public static final int GEAR_PUSH   = 2;
	public static final int GEAR_INTAKE = 4;
	public static final int CLIMBER     = 3;
	
}
