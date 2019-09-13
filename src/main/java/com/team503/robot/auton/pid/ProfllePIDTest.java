/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.auton.pid;

import com.team503.lib.geometry.Pose;
import com.team503.robot.auton.FroggyAuton;

public class ProfllePIDTest extends FroggyAuton {

  @Override
  protected void initAuton() {
    addSequential(new TrapProfileToPoseCommand(new Pose(0.0, 100.0, 180.0), 100, 100));
  }

  @Override
  protected AutonStartingLocation getStartingLocation() {
    return AutonStartingLocation.Origin;
  }
}
