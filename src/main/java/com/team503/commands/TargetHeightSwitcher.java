/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.commands;

import com.team503.robot.RobotState;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.RobotState.TargetHeight;

import edu.wpi.first.wpilibj.command.Command;

public class TargetHeightSwitcher extends Command {

  private TargetHeight height;
  private boolean gotoBus;

  public TargetHeightSwitcher(TargetHeight h) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.height = h;
    this.gotoBus = false;
  }

  public TargetHeightSwitcher(boolean bus) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.gotoBus = bus;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (this.gotoBus) {
      if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
        RobotState.getInstance().setTargetHeight(TargetHeight.MIDDLE);
      } else {
        RobotState.getInstance().setTargetHeight(TargetHeight.LOW);
      }
    } else {
      RobotState.getInstance().setTargetHeight(height);
    }

    // if(RobotState.getInstance().getArmDirection() == ArmDirection.FRONT) {
    // //  LimelightTurret.getInstance().turnToFront();
    // } else {
    // //  LimelightTurret.getInstance().turnToBack();
    // }

    RobotState.getInstance().setPositionChanged(true);
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
