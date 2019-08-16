/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;


import com.team503.robot.RobotState;
import com.team503.robot.subsystems.Intake;

public class EjectBall{
 
  public static void eject() {
    Intake.getInstance().outtakeCargo();
  }
  public static void stopEject() {
    if (!ToggleIntake.getRunning()){
      Intake.getInstance().stopIntake();
      RobotState.getInstance().setHatchDependence(true);
    }
    
  }

}
