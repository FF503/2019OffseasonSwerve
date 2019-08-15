/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands.autons;

import com.team503.robot.commands.SwerveByTime;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftRocketAuton extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftRocketAuton() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.
    // addSequential(new SwerveByTime(0.0, 1.0, 1.0, 0.75, false));
    // addSequential(new SwerveByTime(-0.9, 0.1, 1.0, 1.0, true));

    addSequential(new SwerveByTime(0.0, 1.0, 1.0, 0.5, false));
    addSequential(new SwerveByTime(-1.0, 0.0, 1.0, 0.7, false));
    addSequential(new SwerveByTime(-0.2, 0.6, 1.0, 1.2, true));

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
