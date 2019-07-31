/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;


import com.team503.robot.RobotState;
import com.team503.robot.subsystems.Arm;
import com.team503.robot.subsystems.Extension;
import com.team503.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.command.Command;

public class ResetEncoderCommand extends Command {
  public ResetEncoderCommand() {
    // Use requires() here to declare subsystem dependencies
    // requires(Arm.getInstance());
    // requires(Wrist.getInstance());
    // requires(Extension.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (RobotState.getInstance().getIsManual()) {
      Arm.getInstance().resetEncoder();
      Wrist.getInstance().resetEncoder();
      Extension.getInstance().resetEncoder();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
