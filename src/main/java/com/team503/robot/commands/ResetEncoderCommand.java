/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;


import com.team503.robot.RobotState;
import com.team503.robot.subsystems.AndyArm;
import com.team503.robot.subsystems.Extension;
import com.team503.robot.subsystems.AndyWrist;

public class ResetEncoderCommand{
  
  public static void resetEncs() {
    if (RobotState.getInstance().getIsManual()) {
      AndyArm.getInstance().resetEncoder();
      AndyWrist.getInstance().resetEncoder();
      Extension.getInstance().resetEncoder();
    }
  }

}
