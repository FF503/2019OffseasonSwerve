/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import java.util.Arrays;

import com.team503.lib.util.SwerveHeadingController;
import com.team503.lib.util.Util;
import com.team503.robot.subsystems.Pigeon;
import com.team503.robot.subsystems.SubsystemManager;
import com.team503.robot.subsystems.SwerveDrive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;

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

  public static OI m_oi;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    m_oi = new OI();
    mSwerve = SwerveDrive.getInstance();
    subsystems = new SubsystemManager(Arrays.asList(mSwerve, Pigeon.getInstance()));
    Pigeon.getInstance().zeroSensors();

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

  }

  /**
   * This function is called one time during operator control.
   */
  @Override
  public void teleopInit() {

  }

  /*
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    RobotState.getInstance().setCurrentTheta(Pigeon.getInstance().getYaw());
    double swerveYInput = -OI.getDriverLeftYVal();
    double swerveXInput = OI.getDriverLeftXVal();
    double swerveRotationInput = OI.getDriverRightXVal();
    double deadband = 0.010;

    // check for deadband in controller
    swerveYInput = Util.deadBand(swerveYInput, deadband);
    swerveXInput = Util.deadBand(swerveXInput, deadband);
    // if (swerveYInput > -deadband && swerveYInput < deadband) {
    // swerveYInput = 0.0;
    // }
    // if (swerveXInput > -deadband && swerveXInput < deadband) {
    // swerveXInput = 0.0;
    // }
    if (swerveRotationInput > -deadband && swerveRotationInput < deadband) {
      swerveRotationInput = mSwerve.getRotationalOutput();// 0.0;
    } else {
      mSwerve.stabilize(RobotState.getInstance().getCurrentTheta());
    }

    if (OI.driverJoystick.getBumperPressed(Hand.kRight)) {
      mSwerve.setFieldCentric(!mSwerve.isFieldCentric());
    }

    if (OI.driverJoystick.getAButtonPressed()) {
      mSwerve.rotate(0);
    }

    mSwerve.drive(swerveXInput, swerveYInput, swerveRotationInput);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
    // Pigeon.getInstance().outputToSmartDashboard();
  }
}