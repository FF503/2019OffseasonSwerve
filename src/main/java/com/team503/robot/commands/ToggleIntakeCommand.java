/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.robot.OI;
import com.team503.robot.RobotState;
import com.team503.robot.RobotState.ArmDirection;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.RobotState.TargetHeight;
import com.team503.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ToggleIntakeCommand extends Command {

  private double startTime;
  private boolean hasCargo;

  public ToggleIntakeCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Intake.getInstance());

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // if (!RobotState.getInstance().getIntakeRunning()) {
    // gameElement = RobotState.getInstance().getGameElement();
    startTime = Timer.getFPGATimestamp();
    // // switch (gameElement) {

    // // case CARGO:
    // // Intake.getInstance().intakeCargo();
    // // break;
    // // case HATCH_R:
    // // Intake.getInstance().intakeHatch();
    // // break;
    // // default:

    // // Intake.getInstance().stopIntake();
    // // }
    // Intake.getInstance().intakeCargo();
    // RobotState.getInstance().setIntakeRunning(true);
    // } else {
    // Intake.getInstance().stopIn take();
    // RobotState.getInstance().setIntakeRunning(false);
    // finish = true;
    // }
    Intake.getInstance().intakeCargo();
    RobotState.getInstance().setHatchDependence(false);
    new MoveArmCommand(ArmDirection.FRONT, GameElement.CARGO, TargetHeight.INTAKE).start();
    // Intake.getInstance().stopVacuum();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    hasCargo = (Timer.getFPGATimestamp() - startTime > 0.75 && Intake.getInstance().hasCargo()) /* || OI.getRunOuttake() */;
    return hasCargo;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Intake.getInstance().stopIntake();
    // Intake.getInstance().startVacuum();
    if (hasCargo) {
      RobotState.getInstance().setHasElement(true);
      new MoveArmCommand(ArmDirection.FRONT ,TargetHeight.HOME).start();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
