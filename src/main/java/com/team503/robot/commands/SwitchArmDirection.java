/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.robot.RobotState;
import com.team503.robot.RobotState.ArmDirection;

import edu.wpi.first.wpilibj.Timer;

public class SwitchArmDirection {

  private static double lastPress = 0.0;

  public static void flip() {
    lastPress = Timer.getFPGATimestamp();
    // if (!(Timer.getFPGATimestamp() - lastPress > 0.8)) {
      
    // }

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

  public static void set(ArmDirection dir) {
    RobotState.getInstance().setArmDirection(dir);
  }

}
