/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import com.team503.lib.geometry.Pose;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TestMultiple extends CommandGroup {
  /**
   * Add your docs here.
   */
  public TestMultiple() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // Pose target = new Pose(-34.0, 9, 0.0);
    // Pose target1 = new Pose(-48.0,79.0,-30.0);
    

    // P SLICK 2 HATCH AND 1 BALL
    // Pose target = new Pose(19, 154, -90);
    // Pose target1 = new Pose(-54.0,-29,0);
    // Pose target2= new Pose(40. ,172.0,-90.0);
    // Pose target3 = new Pose(39.0,182.0,90);

    Pose target = new Pose(47, 80, 0);
    Pose interTarget = new Pose(-18, 44, -178);
    Pose target1 = new Pose(-38.0,-15,-178);
    Pose target2= new Pose(19. ,49.0,0.0);
    Pose target3 = new Pose(68.0,75.0,0);

    addSequential(new DriveToPosePID(target), 3.1);
    addSequential(new DriveToPosePID(interTarget), 3.1);
    addSequential(new DriveToPosePID(target1), 3.1);
    addSequential(new DriveToPosePID(target2), 3.1);
    addSequential(new DriveToPosePID(target3), 3.1);




    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
