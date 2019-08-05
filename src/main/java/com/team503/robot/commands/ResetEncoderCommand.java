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

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ResetEncoderCommand extends InstantCommand {
  public ResetEncoderCommand() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (RobotState.getInstance().getIsManual()) {
      Arm.getInstance().resetEncoder();
      Wrist.getInstance().resetEncoder();
      Extension.getInstance().resetEncoder();
    }
  }

}
