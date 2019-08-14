/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.robot.RobotState;
import com.team503.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ReleaseHatch {

  private static double startTime = 0.0;
  private static double timeout = 1.25;
  private static boolean running = false;

  public static void startRelease() {
    if (!running){
      startTime = Timer.getFPGATimestamp();
      Intake.getInstance().releaseHatch();
      RobotState.getInstance().setHasElement(false);
      running = true;
      Intake.getInstance().setVacuumPower(0.0);
    }
  }

  private static boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > timeout;
  }

  
  private static void end() {
    Intake.getInstance().closeReleaseValve();
    running = false;
  }


  public static void handleFinish(){
    if (running && isFinished()){
      end();
    }
    else{
      Intake.getInstance().setVacuumPower(0.8);
    }
  }
}
