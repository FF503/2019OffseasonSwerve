/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import com.team503.lib.io.Xbox;
import com.team503.lib.util.SnappingPosition;
import com.team503.robot.RobotState.ArmDirection;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.RobotState.TargetHeight;
import com.team503.robot.commands.EjectCube;
import com.team503.robot.commands.SimpleFollowTarget;
import com.team503.robot.commands.GameElementSwitcher;
import com.team503.robot.commands.MoveArmCommand;
import com.team503.robot.commands.PassiveIntake;
import com.team503.robot.commands.ReleaseHatchCommand;
import com.team503.robot.commands.ResetEncoderCommand;
import com.team503.robot.commands.SetHeadingCommand;
import com.team503.robot.commands.SwitchArmDirection;
import com.team503.robot.commands.TargetHeightSwitcher;
import com.team503.robot.commands.ToggleControlModeCommand;
import com.team503.robot.commands.ToggleIntakeCommand;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {

	public static Xbox driverJoystick = new Xbox(0);
	public static Xbox operatorJoystick = new Xbox(1);

	// Driver
	private static JoystickButton driverA = new JoystickButton(driverJoystick, 1);
	private static JoystickButton driverB = new JoystickButton(driverJoystick, 2);
	private static JoystickButton driverX = new JoystickButton(driverJoystick, 3);
	private static JoystickButton driverY = new JoystickButton(driverJoystick, 4);
	private static JoystickButton driverLeftBump = new JoystickButton(driverJoystick, 5);
	private static JoystickButton driverRightBump = new JoystickButton(driverJoystick, 6);
	private static JoystickButton driverBack = new JoystickButton(driverJoystick, 7);
	private static JoystickButton driverStart = new JoystickButton(driverJoystick, 8);

	private static Button driverRightTrigger = new Button() {
		@Override
		public boolean get() {
			return getDriverRightTriggerPressed();
		}
	};

	// Operator
	private static JoystickButton low = new JoystickButton(operatorJoystick, 1);
	private static JoystickButton mid = new JoystickButton(operatorJoystick, 2);
	private static JoystickButton bus = new JoystickButton(operatorJoystick, 3);
	private static JoystickButton high = new JoystickButton(operatorJoystick, 4);
	private static JoystickButton flipArm = new JoystickButton(operatorJoystick, 5);
	private static JoystickButton home = new JoystickButton(operatorJoystick, 6);
	private static JoystickButton manual = new JoystickButton(operatorJoystick, 7);

	private static Button cargo = new Button() {
		@Override
		public boolean get() {
			return operatorJoystick.getRawAxis(3) >= 0.5;
		}
	};

	private static Button hatch = new Button() {
		@Override
		public boolean get() {
			return operatorJoystick.getRawAxis(2) >= 0.5;
		}
	};

	private static Button resetEnc = new Button() {
		@Override
		public boolean get() {
			return operatorJoystick.rightCenterClick.isBeingPressed();
		}
	};

	public static void initialize() {
		// Driver
		driverA.whenPressed(new ReleaseHatchCommand());
		driverB.whileHeld(new EjectCube());
		driverX.toggleWhenPressed(new ToggleIntakeCommand());
		driverY.whenPressed(new SimpleFollowTarget());
		driverRightTrigger.whileHeld(new PassiveIntake());

		driverJoystick.POV0.getPressed().whenPressed(new SetHeadingCommand(1));
		driverJoystick.POV90.getPressed().whenPressed(new SetHeadingCommand(90));
		driverJoystick.POV270.getPressed().whenPressed(new SetHeadingCommand(270));
		driverJoystick.POV180.getPressed().whenPressed(new SetHeadingCommand(179));
		driverJoystick.leftBumper.getShortTapButton().whenPressed(new SetHeadingCommand(SnappingPosition.LEFT_NEAR_ROCKET));
		driverJoystick.leftBumper.getLongHoldButton().whenPressed(new SetHeadingCommand(SnappingPosition.LEFT_FAR_ROCKET));
		driverJoystick.rightBumper.getShortTapButton().whenPressed(new SetHeadingCommand(SnappingPosition.RIGHT_NEAR_ROCKET));
		driverJoystick.rightBumper.getLongHoldButton().whenPressed(new SetHeadingCommand(SnappingPosition.RIGHT_FAR_ROCKET));

		// Operator
		low.whenPressed(new TargetHeightSwitcher(TargetHeight.LOW));
		mid.whenPressed(new TargetHeightSwitcher(TargetHeight.MIDDLE));
		high.whenPressed(new TargetHeightSwitcher(TargetHeight.HIGH));
		bus.whenPressed(new TargetHeightSwitcher(TargetHeight.BUS));

		home.whenPressed(new MoveArmCommand(ArmDirection.FRONT, TargetHeight.HOME));
		flipArm.whenPressed(new SwitchArmDirection());

		cargo.whenPressed(new GameElementSwitcher(GameElement.CARGO));
		hatch.whenPressed(new GameElementSwitcher(GameElement.HATCH));

		manual.whenPressed(new ToggleControlModeCommand());
		resetEnc.whenPressed(new ResetEncoderCommand());
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

}