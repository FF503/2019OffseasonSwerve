/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.robot.RobotState;
import com.team503.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class SwerveByTime extends Command {

  private double startTime, time, str, fwd, theta;
  private boolean done;

  private SwerveDrive mSwerve;

  public SwerveByTime(double str, double fwd, double theta, double time, boolean autonDone) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.str = str;
    this.fwd = fwd;
    this.theta = theta;
    this.done = autonDone;
    this.time = time;

    mSwerve = SwerveDrive.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotState.getInstance().setAutonDone(false);
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mSwerve.rotate(theta);
    double rcw = mSwerve.getRotationalOutput();

    mSwerve.drive(str, fwd, 0.0, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > time;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mSwerve.stop();
    RobotState.getInstance().setAutonDone(done);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
