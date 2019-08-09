/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.robot.OI;
import com.team503.robot.RobotState;
import com.team503.robot.RobotState.ArmDirection;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.RobotState.TargetHeight;
import com.team503.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class ToggleIntake  {

  private static double startTime = 0.0;
  private static boolean hasCargo;
  private static boolean running = false;
  public static boolean getRunning(){
    return running;
  }


  public static void toggleIntake() {
   
    if (Timer.getFPGATimestamp() - startTime > 1.0){
      startTime = Timer.getFPGATimestamp();
      if (running){
        end();
        
      }
      else{
        System.out.println("setting power");
        
        Intake.getInstance().intakeCargo();
        RobotState.getInstance().setHatchDependence(false);
        SwitchArmDirection.set(ArmDirection.FRONT);
        GameElementSwitcher.setGameElement(GameElement.CARGO);
        TargetHeightSwitcher.set(TargetHeight.INTAKE);
        running = true;
      }
    }
  
  }

  private static boolean isFinished() {
    hasCargo = (Timer.getFPGATimestamp() - startTime > 0.75 && Intake.getInstance().hasCargo()) /* || OI.getRunOuttake() */;
    return hasCargo;
  }

 
  public static void handleIntakeFinish() {
    if (running && isFinished()){
      System.out.println("handle stop");
      end();
    }
   
  }

  private static void end(){
    Intake.getInstance().stopIntake();
    System.out.println("off step 1");
      if (hasCargo) {
        System.out.println("Turning off");
        RobotState.getInstance().setHasElement(true);
        SwitchArmDirection.set(ArmDirection.FRONT);
        TargetHeightSwitcher.set(TargetHeight.HOME);
      }
      running = false;
  }

 
}
