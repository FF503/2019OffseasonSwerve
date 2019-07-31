/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.commands;

import com.team503.robot.RobotState;
import com.team503.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ReleaseHatchCommand extends Command {

  double startTime = 0.0;
  double timeout = 1.25;

  public ReleaseHatchCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Intake.getInstance());
  }

  public ReleaseHatchCommand(double timeout) {
    this.timeout = timeout;
    requires(Intake.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    Intake.getInstance().releaseHatch();
    RobotState.getInstance().setHasElement(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > timeout;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Intake.getInstance().closeReleaseValve();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
