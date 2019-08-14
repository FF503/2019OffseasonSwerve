/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import com.team503.lib.io.Xbox;
import com.team503.robot.RobotState.ArmDirection;
import com.team503.robot.RobotState.TargetHeight;
import com.team503.robot.commands.ConstantIntakeCommand;
import com.team503.robot.commands.GameElementSwitcher;
import com.team503.robot.commands.MoveArmCommand;
import com.team503.robot.commands.ReleaseHatch;
import com.team503.robot.commands.ResetEncoderCommand;
import com.team503.robot.commands.SwitchArmDirection;
import com.team503.robot.commands.TargetHeightSwitcher;
import com.team503.robot.commands.ToggleIntake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

	public static Xbox driverJoystick = new Xbox(0);
	//public static Xbox operatorJoystick = new Xbox(1);
	public static XboxController driver = new XboxController(0);
	public static XboxController operator = new XboxController(1);
	private static JoystickButton driverA = new JoystickButton(driverJoystick, 1);
	private static JoystickButton driverB = new JoystickButton(driverJoystick, 2);
	private static JoystickButton driverX = new JoystickButton(driverJoystick, 3);
	private static JoystickButton driverY = new JoystickButton(driverJoystick, 4);
	private static JoystickButton driverLeftBump = new JoystickButton(driverJoystick, 5);
	private static JoystickButton driverRightBump = new JoystickButton(driverJoystick, 6);
	private static JoystickButton driverBack = new JoystickButton(driverJoystick, 7);
	private static JoystickButton driverStart = new JoystickButton(driverJoystick, 8);

	private static JoystickButton operatorY = new JoystickButton(operator, 4);
	private static JoystickButton operatorX = new JoystickButton(operator, 3);
	private static JoystickButton operatorB = new JoystickButton(operator, 2);
	private static JoystickButton operatorA = new JoystickButton(operator, 1);
	private static JoystickButton operatorMenu = new JoystickButton(operator, 8);
	private static JoystickButton operatorSelect = new JoystickButton(operator, 7);
	private static JoystickButton operatorLB = new JoystickButton(operator, 5);
	private static JoystickButton operatorRB = new JoystickButton(operator, 6);
	private static JoystickButton operatorRJ = new JoystickButton(operator, 10);

	public static void initialize() {
		
	}

	public static double getDriverLeftXVal() {
		return driverJoystick.getRawAxis(0);
	}

	public static double getDriverLeftYVal() {
		return driverJoystick.getRawAxis(1);
	}

	public static double getDriverRightXVal() {
		return driverJoystick.getRawAxis(4);
	}

	public static double getDriverRightYVal() {
		return driverJoystick.getRawAxis(5);
	}

	public static double getDriverLeftTriggerValue() {
		return driverJoystick.getRawAxis(2);
	}

	public static double getDriverRightTriggerValue() {
		return driverJoystick.getRawAxis(3);
	}

	public static boolean getDriverLeftTriggerPressed() {
		return getDriverLeftTriggerValue() > 0.5;
	}

	public static boolean getDriverRightTriggerPressed() {
		return getDriverRightTriggerValue() > 0.5;
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

	public static Button passiveIntake = new Button() {

		@Override
		public boolean get() {
			return getDriverRightTriggerPressed();
		}
	};

	public static Button setToHPC = new Button() {
		@Override
		public boolean get() {
			return operator.getRawAxis(2) >= 0.9;
		}
	};

	public static Button setToCargo = new Button() {
		@Override
		public boolean get() {
			return operator.getRawAxis(3) >= 0.9;
		}
	};

	public static boolean getOperatorA(){
		return operatorA.get();
	}

	public static boolean getOperatorB(){
		return operatorB.get();
	}
	
	public static boolean getOperatorY(){
		return operatorY.get();
	}

	public static boolean getOperatorX(){
		return operatorX.get();
	}

	public static boolean getOperatorRightBumper(){
		return operatorRB.get();
	}

	public static boolean getOperatorLeftBumper(){
		return operatorLB.get();
	}

	public static boolean getOperatorHatchSwitch(){
		return setToHPC.get();
	}

	public static boolean getOperatorCargoSwitch(){
		return setToCargo.get();
	}

	public static boolean getOperatorMenu(){
		return operatorMenu.get();
	}

	public static boolean getOperatorSelect(){
		return operatorSelect.get();
	}

	public static boolean getOperatorRJ(){
		return operatorRJ.get();
	}


	
}