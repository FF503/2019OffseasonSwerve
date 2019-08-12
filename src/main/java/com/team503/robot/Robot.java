/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import java.util.Arrays;

import com.team503.robot.RobotState.Bot;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.commands.EjectBall;
import com.team503.robot.commands.GameElementSwitcher;
import com.team503.robot.commands.ReleaseHatch;
import com.team503.robot.commands.ResetEncoderCommand;
import com.team503.robot.commands.SwitchArmDirection;
import com.team503.robot.commands.TargetHeightSwitcher;
import com.team503.robot.commands.ToggleControlMode;
import com.team503.robot.commands.ToggleIntake;
import com.team503.robot.commands.VacuumPowerCommand;
import com.team503.robot.Loops.PoseController;
import com.team503.robot.subsystems.Arm;
import com.team503.robot.subsystems.Extension;
import com.team503.robot.subsystems.Pigeon;
import com.team503.robot.subsystems.SubsystemManager;
import com.team503.robot.subsystems.SwerveDrive;
import com.team503.robot.subsystems.SwerveDrive.DriveMode;
import com.team503.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.TimedRobot;
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
  private Arm mArm;
  private Wrist mWrist;
  private Extension mExtension;

  private SubsystemManager subsystems;
  public static OI m_oi;
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
    m_oi = new OI(); m_oi.initialize();
    OI.initialize();
    mSwerve = SwerveDrive.getInstance();
    mArm = Arm.getInstance();
    mWrist = Wrist.getInstance();
    mExtension = Extension.getInstance();
    subsystems = new SubsystemManager(Arrays.asList(mSwerve, Pigeon.getInstance(), mArm, mWrist, mExtension));
    subsystems.resetSensor();
    Pigeon.getInstance().zeroSensors();

    (new VacuumPowerCommand()).start();
   // poseEngine = new PoseController();
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

    switch (SwerveDrive.getInstance().getMode()) {
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
    default:
      break;
    }
    //String pose = RobotState.getInstance().getCurrentPose().toString();
    //SmartDashboard.putString("pose", pose);
    //System.out.println(pose);
    // mSwerve.updateTeleopControl();
    operatorInput();
    mArm.updateSuperstruture();

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
  }

  private void joystickInput() {
    double swerveYInput = OI.getDriverLeftYVal();
    double swerveXInput = -OI.getDriverLeftXVal();
    double swerveRotationInput = OI.getDriverRightXVal();
    boolean lowPower = OI.getDriverLeftTriggerPressed();
    double deadband = 0.010;

    if (swerveRotationInput > -deadband && swerveRotationInput < deadband) {
      swerveRotationInput = mSwerve.getRotationalOutput();// 0.0;
      // SmartDashboard.putNumber("Rotational Output", mSwerve.getRotationalOutput());
    } else {
      mSwerve.rotate(RobotState.getInstance().getCurrentTheta());
    }

    // if (OI.driverJoystick.leftBumper.shortReleased()) {
    //   mSwerve.rotate(-24);
    //   swerveRotationInput = mSwerve.getRotationalOutput();
    // } else if (OI.driverJoystick.leftBumper.longPressed()) {
    //   mSwerve.rotate(-151.0);
    //   swerveRotationInput = mSwerve.getRotationalOutput();
    // } else if (OI.driverJoystick.rightBumper.shortReleased()) {
    //   mSwerve.rotate(24);
    //   swerveRotationInput = mSwerve.getRotationalOutput();
    // } else if (OI.driverJoystick.rightBumper.longPressed()) {
    //   mSwerve.rotate(151.0);
    //   swerveRotationInput = mSwerve.getRotationalOutput();
    // } else if (OI.driverJoystick.getAButtonPressed()) {
    //   mSwerve.rotate(180);
    //   swerveRotationInput = mSwerve.getRotationalOutput();
    // } else if (OI.driverJoystick.getBButtonPressed()) {
    //   mSwerve.rotate(90);
    //   swerveRotationInput = mSwerve.getRotationalOutput();
    // } else if (OI.driverJoystick.getXButtonPressed()) {
    //   mSwerve.rotate(270);
    //   swerveRotationInput = mSwerve.getRotationalOutput();
    // } else if (OI.driverJoystick.getYButtonPressed()) {
    //   mSwerve.rotate(0);
    //   swerveRotationInput = mSwerve.getRotationalOutput();
    //} else 
    System.out.println(OI.driverJoystick.getPOV());
    if (OI.driverJoystick.getStartButtonPressed()) {
      mSwerve.setMode(DriveMode.Defense);
    }
    mSwerve.setFieldCentric(!OI.getDriverLeftTriggerPressed());
    // mSwerve.inputDrive(swerveXInput, swerveYInput, swerveRotationInput,
    // lowPower);
    mSwerve.drive(swerveXInput, swerveYInput, swerveRotationInput, lowPower);
  }
  public void operatorInput(){
    if (OI.getOperatorA()){
      TargetHeightSwitcher.set(RobotState.TargetHeight.LOW);
    }
    else if (OI.getOperatorB()){
      TargetHeightSwitcher.set(RobotState.TargetHeight.MIDDLE);
    }
    else if (OI.getOperatorX()){
      TargetHeightSwitcher.set(RobotState.TargetHeight.BUS);
    }
    else if (OI.getOperatorY()){
      TargetHeightSwitcher.set(RobotState.TargetHeight.HIGH);
    }
    else if(OI.getOperatorMenu()){
      TargetHeightSwitcher.set(RobotState.TargetHeight.INTAKE);
    }
    else if (OI.getOperatorRightBumper()){
      TargetHeightSwitcher.set(RobotState.TargetHeight.HOME);
    }
    else if (OI.getOperatorLeftBumper()){
      SwitchArmDirection.flip();
    }
    else if (OI.getOperatorHatchSwitch()){
      GameElementSwitcher.setGameElement(GameElement.HATCH);
    }
    else if (OI.getOperatorCargoSwitch()){
      GameElementSwitcher.setGameElement(GameElement.CARGO);
    }
    else if (OI.getOperatorSelect()){
      ToggleControlMode.toggle();
    }
    else if (OI.getDriverXButton()){
      System.out.println("x");
      ToggleIntake.toggleIntake();
    }
    ToggleIntake.handleIntakeFinish();
    if (OI.getDriverBButton()){
      EjectBall.eject();
    }
    else{
      EjectBall.stopEject();
    }
    if (OI.getDriverAButton()){
      ReleaseHatch.startRelease();
    }
    ReleaseHatch.handleFinish();
    if (OI.getOperatorRJ()){
      ResetEncoderCommand.resetEncs();
    }
  }
}