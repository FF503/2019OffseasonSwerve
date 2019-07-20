/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.controlAlgorithms.FrogPID;
import frc.controlAlgorithms.FrogPID.ControlMode;
import frc.controlAlgorithms.SwerveTeleopHeadingController;
import frc.subsystem.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private SwerveDrive swerveDrive;

  private List<Subsystem> subsystems = Arrays.asList(Pigeon.getInstance());

  public static OI m_oi;
  private FrogPID rotationalHoldPID = new FrogPID(503, 503, 503, 0, ControlMode.Position_Control);
  private SwerveTeleopHeadingController teleopHeadingController;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    swerveDrive = new SwerveDrive();
    Pigeon.getInstance().zeroSensors();
    this.teleopHeadingController = new SwerveTeleopHeadingController(rotationalHoldPID);

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
    subsystems.forEach((s) -> s.outputTelemetry());
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
    if (swerveYInput > -deadband && swerveYInput < deadband) {
      swerveYInput = 0.0;
    }
    if (swerveRotationInput > -deadband && swerveRotationInput < deadband) {
      swerveRotationInput = teleopHeadingController.getRotationalOutput();// 0.0;
    } else {
      teleopHeadingController.setRotationalSetpoint(RobotState.getInstance().getCurrentTheta());
    }

    if (swerveXInput > -deadband && swerveXInput < deadband) {
      swerveXInput = 0.0;
    }

    swerveDrive.drive(swerveXInput, swerveYInput, swerveRotationInput);

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