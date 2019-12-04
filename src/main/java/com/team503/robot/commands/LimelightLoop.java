/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.commands;

import java.util.Arrays;

import com.team503.robot.RobotState;
import com.team503.robot.loops.LimelightProcessor;
import com.team503.robot.subsystems.Pigeon;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class LimelightLoop extends Command {
  private LimelightProcessor processor = new LimelightProcessor();

  public LimelightLoop() {

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double[] trans = processor.getTranslation();
    double x, y, angle;
    if (!Arrays.equals(trans, new double[] { 0, 0, 0, 0, 0, 0 })) {
      // use trans
      x = trans[0];
      y = trans[2];
      angle = trans[4];
    } else {
      // use tx/ta...
      x = y = angle = 0; // defualt until updated later
    }

    processor.updateData(x, y, angle, trans);
    RobotState.getInstance().setLimelightSeesTarget(processor.sawTarget(3));
    System.out.println(processor.getCollectedData().getLast());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return DriverStation.getInstance().isDisabled();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
