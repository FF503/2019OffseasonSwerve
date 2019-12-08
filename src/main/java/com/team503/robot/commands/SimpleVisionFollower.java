/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.lib.controllers.VisionFollowerController;
import com.team503.lib.util.SnappingPosition;
import com.team503.robot.loops.LimelightLoop;
import com.team503.robot.subsystems.LimelightProcessor;
import com.team503.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class SimpleVisionFollower extends Command {
  VisionFollowerController controller = new VisionFollowerController();

  public SimpleVisionFollower() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SwerveDrive.getInstance().setFieldCentric(false);
    SwerveDrive.getInstance().rotate(SnappingPosition.LEFT_NEAR_ROCKET);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    var x = controller
        .getVectorToTarget(LimelightProcessor.getInstance().getTA(), LimelightProcessor.getInstance().getTX())
        .unaryMinus();
    System.out.println(x);
    SwerveDrive.getInstance().drive(x);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return DriverStation.getInstance().isDisabled() || LimelightProcessor.getInstance().getTA() > 2.7
        || !LimelightProcessor.getInstance().seesTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SwerveDrive.getInstance().setFieldCentric(true);

  }

}
