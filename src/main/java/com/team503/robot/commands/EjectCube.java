/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;


import com.team503.robot.RobotState;
import com.team503.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

public class EjectCube extends Command {
  public EjectCube() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Intake.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // GameElement gameElement = RobotState.getInstance().getGameElement();
    // switch (gameElement) {
    // case CARGO:
    //   Intake.getInstance().outtakeCargo();
    //   break;

    // case HATCH_R:
    //   Intake.getInstance().outtakeHatch();
    //   break;

    // case HATCH_C:
    // Intake.getInstance().stopIntake();
    //   break;
    // }
    
    Intake.getInstance().outtakeCargo();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Intake.getInstance().stopIntake();
    RobotState.getInstance().setHatchDependence(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
