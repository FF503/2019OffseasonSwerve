/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;



import com.team503.robot.RobotState;
import com.team503.robot.RobotState.GameElement;

public class GameElementSwitcher  {
  
  private GameElement element;

  public static void setGameElement(GameElement e) {
    RobotState.getInstance().setGameElement(e);
  }

}
