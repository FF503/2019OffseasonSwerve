/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.robot.RobotState;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.RobotState.TargetHeight;

import edu.wpi.first.wpilibj.command.Command;

public class TargetHeightSwitcher{

  public static void set(TargetHeight h) {
    RobotState.getInstance().setTargetHeight(h);
    RobotState.getInstance().setPositionChanged(true);
  }

  public static void set(boolean gotoBus) {
    if (gotoBus) {
      if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
        RobotState.getInstance().setTargetHeight(TargetHeight.MIDDLE);
      } else {
        RobotState.getInstance().setTargetHeight(TargetHeight.LOW);
      }
    } 
    RobotState.getInstance().setPositionChanged(true);
  }

}
