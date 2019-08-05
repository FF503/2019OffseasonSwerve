/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import java.util.Arrays;

import com.team254.lib.geometry.Translation2d;
import com.team503.robot.RobotState.Bot;
import com.team503.robot.Loops.PoseController;
import com.team503.robot.subsystems.Arm;
import com.team503.robot.subsystems.Extension;
import com.team503.robot.subsystems.Intake;
import com.team503.robot.subsystems.Pigeon;
import com.team503.robot.subsystems.SubsystemManager;
import com.team503.robot.subsystems.SwerveDrive;
import com.team503.robot.subsystems.SwerveDrive.DriveMode;
import com.team503.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SwerveDrive mSwerve;
  private SubsystemManager subsystems;
  public static RobotHardware bot;
  private PoseController poseEngine;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    RobotState.getInstance().setCurrentRobot(Bot.Automatic);
    bot = RobotHardware.getInstance();
    mSwerve = SwerveDrive.getInstance();
    subsystems = new SubsystemManager(mSwerve, Pigeon.getInstance(), Arm.getInstance(), Extension.getInstance(),
        Intake.getInstance(), Wrist.getInstance());
    Pigeon.getInstance().zeroSensors();
    OI.initialize();
    // poseEngine = PoseController.getInstance();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    subsystems.outputToSmartDashboard();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called one time during operator control.
   */
  @Override
  public void teleopInit() {
    // poseEngine.start();
    mSwerve.setBrakeMode();
  }

  /*
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    OI.driverJoystick.update();
    RobotState.getInstance().setCurrentTheta(Pigeon.getInstance().getYaw());

    switch (mSwerve.getMode()) {
    case TeleopDrive:
      joystickInput();
      break;
    case Defense:
      if (!OI.driverJoystick.getStartButton()) {
        mSwerve.setMode(DriveMode.TeleopDrive);
        break;
      }
      mSwerve.defensePosition();
      break;
    case Vision:
      if (!OI.driverJoystick.yButton.isBeingPressed()) {
        mSwerve.setMode(DriveMode.TeleopDrive);
        break;
      }
      break;
    default:
      break;
    }

    if (OI.driverJoystick.leftCenterClick.isBeingPressed()) {
      mSwerve.setMode(DriveMode.TeleopDrive);
    }

    // String pose = RobotState.getInstance().getCurrentPose().toString();
    // SmartDashboard.putString("pose", pose);
    // System.out.println(pose);

    // mSwerve.updateTeleopControl();
    Scheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    subsystems.resetSensor();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    mSwerve.setCoastMode();
    subsystems.stop();
  }

  @Override
  public void disabledPeriodic() {
    // Pigeon.getInstance().outputToSmartDashboard();
    Scheduler.getInstance().run();
  }

  private void joystickInput() {
    double swerveYInput = -OI.getDriverLeftYVal();
    double swerveXInput = OI.getDriverLeftXVal();
    double swerveRotationInput = OI.getDriverRightXVal();
    double deadband = 0.010;

    if (swerveRotationInput > -deadband && swerveRotationInput < deadband) {
      swerveRotationInput = mSwerve.getRotationalOutput();// 0.0;
      // SmartDashboard.putNumber("Rotational Output", mSwerve.getRotationalOutput());
    } else {
      mSwerve.rotate(RobotState.getInstance().getCurrentTheta());
    }

    if (OI.driverJoystick.getStartButtonPressed()) {
      mSwerve.setMode(DriveMode.Defense);
    }

    mSwerve.setFieldCentric(!OI.getDriverLeftTriggerPressed());
    mSwerve.drive(new Translation2d(swerveXInput, swerveYInput), swerveRotationInput, OI.getDriverLeftTriggerPressed());
  }
}