/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

	private static XboxController driverJoystick = new XboxController(0);
	private static JoystickButton driverA = new JoystickButton(driverJoystick, 1);	
	private static JoystickButton driverB = new JoystickButton(driverJoystick, 2);
	private static JoystickButton driverX = new JoystickButton(driverJoystick, 3);

	private static JoystickButton driverY = new JoystickButton(driverJoystick, 4);
	private static JoystickButton driverLeftBump = new JoystickButton(driverJoystick, 5);
	private static JoystickButton driverRightBump= new JoystickButton(driverJoystick, 6);
	private static JoystickButton driverBack = new JoystickButton(driverJoystick, 7);
	private static JoystickButton driverStart = new JoystickButton(driverJoystick, 8);


	public static void initialize(){

	}

	public static double getDriverLeftXVal(){
		return driverJoystick.getRawAxis(0);
	}

	public static double getDriverLeftYVal(){
		return driverJoystick.getRawAxis(1);
	}

	public static double getDriverRightXVal(){
		return driverJoystick.getRawAxis(4);
	}

	public static double getDriverRightYVal(){
		return driverJoystick.getRawAxis(5);
	}

	public static boolean getDriverAButton() {
		return driverA.get();
	}
	public static boolean getDriverBButton() {
		return driverB.get();
	}

	public static boolean getDriverXButton() {
		return driverX.get();
	}

	public static boolean getDriverYButton() {
		return driverY.get();
	}

	public static boolean getDriverLeftBumper() {
		return driverLeftBump.get();
	}

	public static boolean getDriverRightBumper() {
		return driverRightBump.get();
	}

	public static boolean getDriverBackButton() {
		return driverBack.get();
	}

	public static boolean getDriverStartButton() {
		return driverStart.get();
	}



}