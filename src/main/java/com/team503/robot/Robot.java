/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import java.util.Arrays;

import com.team503.robot.RobotState.ArmDirection;
import com.team503.robot.RobotState.Bot;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.auton.ForwardTest;
import com.team503.robot.commands.EjectBall;
import com.team503.robot.commands.GameElementSwitcher;
import com.team503.robot.commands.ReleaseHatch;
import com.team503.robot.commands.ResetEncoderCommand;
import com.team503.robot.commands.SwitchArmDirection;
import com.team503.robot.commands.TargetHeightSwitcher;
import com.team503.robot.commands.ToggleControlMode;
import com.team503.robot.commands.ToggleIntake;
import com.team503.robot.loops.FroggyPoseController;
import com.team503.robot.loops.LimelightProcessor;
import com.team503.robot.loops.LimelightProcessor.Pipeline;
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

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    RobotState.getInstance().setCurrentRobot(Bot.Automatic);
    bot = RobotHardware.getInstance();
    OI.initialize();

    mSwerve = SwerveDrive.getInstance();

    // Subsytem Manager
    if (RobotState.getInstance().getCurrentRobot().equals(Bot.FFSwerve)) {
      subsystems = new SubsystemManager(Arrays.asList(mSwerve, Pigeon.getInstance()));
    } else {
      subsystems = new SubsystemManager(Arrays.asList(mSwerve, Pigeon.getInstance(), Arm.getInstance(),
          Wrist.getInstance(), Extension.getInstance(), Intake.getInstance()));

    }
    subsystems.resetSensor();
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
    System.out.println("ROBOT"+ RobotState.getInstance().getCurrentRobot().name());
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
    Pigeon.getInstance().zeroSensors();
    mSwerve.setBrakeMode();
    Intake.getInstance().startVacuum();
    LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);

    new ForwardTest().initAndStartAuton();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    RobotState.getInstance().setCurrentTheta(Pigeon.getInstance().getYaw());
    OI.driverJoystick.update();
    switch (SwerveDrive.getInstance().getMode()) {
      case TeleopDrive:
        OI.joystickInput(mSwerve, mLime);

      break;
    case Defense:
      if (!OI.driverJoystick.getStartButton()) {
        mSwerve.setMode(DriveMode.TeleopDrive);
        break;
      }
      mSwerve.defensePosition();
      break;
    default:
      break;
    }
    OI.operatorInput();

    FroggyPoseController.updateOdometry();
    FroggyPoseController.outputPoseToDashboard();
    Arm.getInstance().updateSuperstruture();

    Scheduler.getInstance().run();
  }

  /**
   * This function is called one time during operator control.
   */
  @Override
  public void teleopInit() {
    mSwerve.setBrakeMode();
    Intake.getInstance().startVacuum();
    LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);
  }

  /*
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    OI.driverJoystick.update();
    RobotState.getInstance().setCurrentTheta(Pigeon.getInstance().getYaw());

    switch (SwerveDrive.getInstance().getMode()) {
    case TeleopDrive:
      OI.joystickInput(mSwerve, mLime);
      break;
    case Defense:
      if (!OI.driverJoystick.getStartButton()) {
        mSwerve.setMode(DriveMode.TeleopDrive);
        break;
      }
      mSwerve.defensePosition();
      break;
    default:
      break;
    }
    if (RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
      OI.operatorInput();
      Arm.getInstance().updateSuperstruture();
    }

    if (RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
      operatorInput();
      Arm.getInstance().updateSuperstruture();
    }

    FroggyPoseController.updateOdometry();
    FroggyPoseController.outputPoseToDashboard();
    subsystems.outputToSmartDashboard();
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
    mSwerve.setBrakeMode();
    subsystems.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  
}