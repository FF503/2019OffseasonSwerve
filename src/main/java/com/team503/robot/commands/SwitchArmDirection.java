/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands
;


import com.team503.robot.RobotState;
import com.team503.robot.RobotState.ArmDirection;

import edu.wpi.first.wpilibj.command.Command;

public class SwitchArmDirection extends Command {
  private static final RobotState RobotState = null;

public SwitchArmDirection() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotState.getInstance().setIsArmFlip(true);
    ArmDirection armDirection = RobotState.getInstance().getArmDirection();
    if (armDirection.equals(ArmDirection.FRONT)) {
      RobotState.getInstance().setArmDirection(ArmDirection.BACK);
      // LimelightTurret.getInstance().turnToFront();
    } else if (armDirection.equals(ArmDirection.BACK)) {
      RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
      // LimelightTurret.getInstance().turnToBack();
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
  }
}
