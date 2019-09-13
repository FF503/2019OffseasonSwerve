/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.auton.pid;

import com.team503.lib.controllers.PIDProfileController;
import com.team503.lib.controllers.TrapezoidProfile;
import com.team503.lib.geometry.Pose;
import com.team503.lib.geometry.Translation2d;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.subsystems.SwerveDrive;
import com.team503.robot.subsystems.SwerveDrive.DriveMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrapProfileToPoseCommand extends Command {
  private Pose target;
  private PIDProfileController forwardController, strafeController;

  /**
   * 
   * @param target should be in terms of unit circle
   */
  public TrapProfileToPoseCommand(Pose target) {
    this(target, Robot.bot.kPathFollowingMaxVel, Robot.bot.kPathFollowingMaxAccel);
  }

  public TrapProfileToPoseCommand(Pose target, double maxVelocity, double maxAcceleration) {
    this.target = target;
    var constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

    strafeController = new PIDProfileController(0.005, 0, 0, constraints);
    forwardController = new PIDProfileController(1.5, 0, 0, constraints);

    forwardController.setGoal(target.getY());
    strafeController.setGoal(target.getX());

    forwardController.setTolerance(2.0);
    strafeController.setTolerance(2.0);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SwerveDrive.getInstance().setMode(DriveMode.ProfilePID);
    SwerveDrive.getInstance().rotate(target.getTheta() - 90.0);
  }

  int counter = 0;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Pose currentPose = RobotState.getInstance().getCurrentPose().getTranslatedPose();
    double strafe = strafeController.calculate(currentPose.getX());
    double fwd = forwardController.calculate(currentPose.getY());
    SmartDashboard.putNumber("Forward Power", fwd);
    SmartDashboard.putNumber("Strafe Power", strafe);
    SmartDashboard.putNumber("Run Counter", counter);
    Translation2d driveVector = new Translation2d(strafe, fwd);
    SwerveDrive.getInstance().drive(driveVector);
    counter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (forwardController.atGoal() && strafeController.atGoal()) || DriverStation.getInstance().isDisabled();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SwerveDrive.getInstance().stop();
    SwerveDrive.getInstance().setMode(DriveMode.TeleopDrive);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
