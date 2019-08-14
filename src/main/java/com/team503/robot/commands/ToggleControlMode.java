/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;


import com.team503.robot.RobotState;

import edu.wpi.first.wpilibj.Timer;

public class ToggleControlMode{
  private static double lastPress = 0.0;

  public static void toggle() {
    
    if (Timer.getFPGATimestamp() - lastPress > 0.8){
      if (RobotState.getInstance().getIsManual()) {
        RobotState.getInstance().setIsManual(false);
      } else {
        RobotState.getInstance().setIsManual(true);
      }
      lastPress = Timer.getFPGATimestamp();
    }
    
  }

}
