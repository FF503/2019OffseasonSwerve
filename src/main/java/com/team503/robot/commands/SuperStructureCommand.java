
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.robot.OI;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.RobotState.ArmDirection;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.RobotState.SuperStructurePreset;
import com.team503.robot.RobotState.TargetHeight;
import com.team503.robot.subsystems.Arm;
import com.team503.robot.subsystems.Extension;
import com.team503.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class SuperStructureCommand extends Command {

	private double aTgt, eTgt, wTgt, eLim = 0;
	private boolean eIsMax, eIsMin = false;
	private int manualIdx = 0;

	public SuperStructureCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Arm.getInstance());
		// requires(Wrist.getInstance());
		// requires(Extension.getInstance());
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		aTgt = Arm.getInstance().getEncoderDeg();
		eTgt = Robot.bot.gExtMinLim;
		wTgt = 90.0;
		RobotState.getInstance().setIsManual(false);
		RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if ((DriverStation.getInstance().isAutonomous() || !RobotState.getInstance().getIsManual())) {
			manualIdx = 0;
			if (RobotState.getInstance().getPositionChanged()) {
				ArmDirection armDirection = RobotState.getInstance().getArmDirection();
				TargetHeight tgtHeight = RobotState.getInstance().getTargetHeight();

				// ArmPosition armPos = RobotState.getInstance().getArmPosition();
				// ExtensionPosition extPos = RobotState.getInstance().getExtPosition();
				if (armDirection.equals(ArmDirection.FRONT)) {
					if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
						switch (tgtHeight) {
						case INTAKE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_INTAKE);
							break;
						case LOW:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_LOW);
							break;
						case MIDDLE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_MID);
							break;
						case HIGH:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_HIGH);
							break;
						case BUS:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_BUS);
							break;
						case HOME:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.CARGO_HOME);
							break;
						}

					} else {
						switch (tgtHeight) {
						case INTAKE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_INTAKE);
							break;
						case LOW:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_LOW);
							break;
						case MIDDLE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_MID);
							break;
						case HIGH:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_HIGH);
							break;
						case BUS:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_BUS);
							break;
						case HOME:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
							break;
						}
					}
				} else if (armDirection.equals(ArmDirection.BACK)) {
					if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
						switch (tgtHeight) {
						case INTAKE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_INTAKE);
							break;
						case LOW:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_LOW);
							break;
						case MIDDLE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_MID);
							break;
						case HIGH:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_HIGH);
							break;
						case BUS:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_BUS);
							break;
						case HOME:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
							break;
						}
					} else {
						switch (tgtHeight) {
						case INTAKE:
							if (DriverStation.getInstance().isAutonomous()) {
								RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.VIEW_AUTO_REAR);
								break;
							}
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_INTAKE);
							break;
						case LOW:
							if (DriverStation.getInstance().isAutonomous()) {
								RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_LOW);
								break;
							}
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_LOW);
							break;
						case MIDDLE:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_MID);
							break;
						case HIGH:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_HIGH);
							break;
						case BUS:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_BUS);
							break;
						case HOME:
							RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
							break;
						}

					}
				}
				RobotState.getInstance().setPositionChanged(false);

				SuperStructurePreset preset = RobotState.getInstance().getSuperStructurePreset();
				wTgt = preset.getWristPosition();
				aTgt = preset.getArmPosition();
				eTgt = preset.getExtPosition();
			}
			if (Robot.bot.hasWrist()) {
				double wristPower = Wrist.getInstance().getTalon().getOutputCurrent()
						* Wrist.getInstance().getTalon().getMotorOutputVoltage();
				// System.out.println("Wrist Power Watts: " + wristPower);
				if (wristPower > Robot.bot.MAX_WRIST_POWER) {
					Wrist.getInstance().setMotorOutput(0.0);
					System.out.println("WRIST POWER TOO HIGH BURN OUT WARNING");
				} else {
					if (!(RobotState.getInstance().getSuperStructurePreset() == SuperStructurePreset.FRONT_CARGO_BUS
							&& Arm.getInstance().getEncoderDeg() < -30)) {
						Wrist.getInstance().setTargetPosition(wTgt);
					}

				}
				// if (RobotState.getInstance().getGameElement() == GameElement.CARGO
				// && (Arm.getInstance().getEncoderDeg() > -37)) {
				// Wrist.getInstance().setTargetPosition(wTgt);
				// } else {
				// Wrist.getInstance().setTargetPosition(RobotState.SuperStructurePreset.FRONT_CARGO_INTAKE.getWristPosition());
				// }
				// }
			}
			if (Robot.bot.hasArm()) {
				Arm.getInstance().setTargetPosition(aTgt);
				if (Arm.getInstance().getEncoderDeg() > -40) {
					Extension.getInstance().setTargetPosition(eTgt);
				} else {
					Extension.getInstance().setTargetPosition(0.0);
				}
			}
		} else {
			if (manualIdx == 0) {
				// (new RumbleOperatorJoystick(1.0)).start();
				manualIdx++;
			}
			if (Robot.bot.hasArm()) {

				if (Arm.getInstance().getEncoderDeg() > 0.0 && Arm.getInstance().getEncoderDeg() < 180.0) {
					eLim = Robot.bot.gArmExtLength
							* (1 / Math.cos(Math.toRadians(Arm.getInstance().getEncoderDeg())) - 1);
					eLim = Math.abs(eLim);
				} else {
					eLim = 0.0;
				}

				aTgt = Arm.getInstance().getEncoderDeg();
				wTgt = Wrist.getInstance().getHRelEncoderDeg();
				eTgt = Extension.getInstance().getExtPosition();

				// eIsMax = Extension.getInstance().getExtPosition() > eLim;
				// eIsMin = Extension.getInstance().getExtPosition() < Robot.bot.gExtMinLim;
				Arm.getInstance().setMotorOutput(-OI.operatorJoystick.getY(Hand.kLeft));
				Wrist.getInstance().setMotorOutput(-OI.operatorJoystick.getY(Hand.kRight));

				// Extension.getInstance().setTargetPosition(eLim);

				if (OI.operatorJoystick.getPOV() == 0) {// && !eIsMax) {
					Extension.getInstance().setMotorOutput(0.50);
				} else if (OI.operatorJoystick.getPOV() == 180 /* && !eIsMin */) {
					Extension.getInstance().setMotorOutput(-0.50);
				} else {
					Extension.getInstance().setMotorOutput(0.00);
				}
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return DriverStation.getInstance().isDisabled();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Arm.getInstance().setMotorOutput(0.0);
		Wrist.getInstance().setMotorOutput(0.0);
		Extension.getInstance().setMotorOutput(0.0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
