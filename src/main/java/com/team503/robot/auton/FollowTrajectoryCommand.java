/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.auton;

import com.team503.lib.controllers.PurePursuitController;
import com.team503.robot.RobotState;
import com.team503.robot.subsystems.SwerveDrive;
import com.team503.robot.subsystems.SwerveDrive.DriveMode;

import edu.wpi.first.wpilibj.command.Command;
import motionProfiling.Trajectory;

public class FollowTrajectoryCommand extends Command {
  private PurePursuitController controller;
  private SwerveDrive mSwerve = SwerveDrive.getInstance();

  public FollowTrajectoryCommand(Trajectory traj, double lookahead) {
    controller = new PurePursuitController(traj, lookahead);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mSwerve.setMode(DriveMode.PurePursuit);
    mSwerve.setFieldCentric(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    mSwerve.drive(controller.calculateDriveVector(RobotState.getInstance().getCurrentPose()));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return controller.isDone();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mSwerve.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
