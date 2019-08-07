/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.robot.RobotState.ArmDirection;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.RobotState.TargetHeight;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class MoveArmCommand extends CommandGroup {

  public MoveArmCommand(ArmDirection armDirection, TargetHeight targetHeight) {
    addSequential(new SetArmDirection(armDirection));
   // addSequential(new TargetHeightSwitcher(targetHeight));
  }

  public MoveArmCommand(ArmDirection armDirection, GameElement gameElement, TargetHeight targetHeight) {
    addSequential(new SetArmDirection(armDirection));
    addSequential(new GameElementSwitcher(gameElement));
  //  addSequential(new TargetHeightSwitcher(targetHeight));
  }

  public MoveArmCommand(ArmDirection armDirection, TargetHeight targetHeight, double delay) {
    addSequential(new WaitCommand(delay));
    addSequential(new SetArmDirection(armDirection));
  //  addSequential(new TargetHeightSwitcher(targetHeight));
  }

  public MoveArmCommand(ArmDirection armDirection, GameElement gameElement, TargetHeight targetHeight, double delay) {
    addSequential(new WaitCommand(delay));
    addSequential(new SetArmDirection(armDirection));
    addSequential(new GameElementSwitcher(gameElement));
  //  addSequential(new TargetHeightSwitcher(targetHeight));
  }
}
