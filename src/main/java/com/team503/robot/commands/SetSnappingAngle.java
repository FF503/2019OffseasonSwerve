/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.lib.util.SnappingPosition;
import com.team503.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class SetSnappingAngle extends InstantCommand {
  /**
   * Add your docs here.
   */

   private SnappingPosition pos;
  public SetSnappingAngle(SnappingPosition pos) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.pos = pos;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    SwerveDrive.getInstance().rotate(pos);
  }

}
