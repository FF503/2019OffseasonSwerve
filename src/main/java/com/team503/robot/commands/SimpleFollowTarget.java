/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.lib.controllers.VisionFollowerController;
import com.team503.robot.OI;
import com.team503.robot.Loops.LimelightProcessor;
import com.team503.robot.Loops.LimelightProcessor.Pipeline;
import com.team503.robot.subsystems.SwerveDrive;
import com.team503.robot.subsystems.SwerveDrive.DriveMode;

import edu.wpi.first.wpilibj.command.Command;

public class SimpleFollowTarget extends Command {
  private VisionFollowerController controller;
  private SwerveDrive mSwerve;

  public SimpleFollowTarget() {
    // Use requires() here to declare subsystem dependencies
    controller = new VisionFollowerController();
    mSwerve = SwerveDrive.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    LimelightProcessor.getInstance().setVisionMode();
    LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);
    mSwerve.setFieldCentric(false);
    mSwerve.setMode(DriveMode.Vision);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    final double ta = LimelightProcessor.getInstance().getTA(), tx = LimelightProcessor.getInstance().getTX();
    mSwerve.drive(controller.getVectorToTarget(ta, tx));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (mSwerve.getMode() != DriveMode.Vision)
        || (timeSinceInitialized() > 0.2 && !LimelightProcessor.getInstance().seesTarget())
        || (LimelightProcessor.getInstance().hasReachedAreaThreshold());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    mSwerve.stop();
    mSwerve.setFieldCentric(true);
    mSwerve.setMode(DriveMode.TeleopDrive);
    LimelightProcessor.getInstance().setPipeline(Pipeline.DRIVER);
    OI.driverJoystick.rumble(1, 0.25);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
