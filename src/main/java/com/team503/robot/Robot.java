/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot; 

import java.util.Arrays;

import com.team503.robot.RobotState.Bot;
import com.team503.robot.loops.LimelightProcessor;
import com.team503.robot.loops.LimelightProcessor.Pipeline;
import com.team503.robot.subsystems.AndyArm;
import com.team503.robot.subsystems.AndyWrist;
import com.team503.robot.subsystems.Arm;
import com.team503.robot.subsystems.BallIntake;
import com.team503.robot.subsystems.DiskIntake;
import com.team503.robot.subsystems.Elevator;
import com.team503.robot.subsystems.Extension;
import com.team503.robot.subsystems.Intake;
import com.team503.robot.subsystems.Pigeon;
import com.team503.robot.subsystems.SubsystemManager;
import com.team503.robot.subsystems.Superstructure;
import com.team503.robot.subsystems.SwerveDrive;
import com.team503.robot.subsystems.SwerveDrive.DriveMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Call all subsystem instances // all extend Subsystem
  private SwerveDrive mSwerve;
  private Elevator mElevator;
  private Arm mArm;
  private BallIntake mBallIntake;
  private DiskIntake mDiskIntake;
  private Superstructure mS;

  private SubsystemManager subsystems; // encloses the subsystem array list and operations
  public static RobotHardware bot;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    RobotState.getInstance().setCurrentRobot(Bot.FFSwerve);
    bot = RobotHardware.getInstance();
    OI.initialize();

    mSwerve = SwerveDrive.getInstance(); // Construct current instances
    mElevator = Elevator.getInstance();
    mArm = Arm.getInstance();
    mBallIntake = BallIntake.getInstance();
    mDiskIntake = DiskIntake.getInstance();
    mS = Superstructure.getInstance();

    // Subsytem Manager

    if (RobotState.getInstance().getCurrentRobot().equals(Bot.FFSwerve)) {
      subsystems = new SubsystemManager( // Construct subsystem manager array
          Arrays.asList(Pigeon.getInstance(), mElevator, mArm, mBallIntake, mDiskIntake, mS));
    } else if (RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
      subsystems = new SubsystemManager(Arrays.asList(mSwerve, Pigeon.getInstance(), AndyArm.getInstance(),
          AndyWrist.getInstance(), Extension.getInstance(), Intake.getInstance()));
    }
    subsystems.resetSensor();
    CameraServer.getInstance().startAutomaticCapture();
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
    subsystems.outputToSmartDashboard(); // always printing data for EVERY SUBSYSTEM
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
    // mSwerve.setBrakeMode();
    mSwerve.snapForward();
    // Intake.getInstance().startVacuum();
    // LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);
    // PrecisionDriveController.activatePrecisionDrive();
    mBallIntake.onStart(Timer.getFPGATimestamp());
    mDiskIntake.onStart(Timer.getFPGATimestamp());
    mS.onStart(Timer.getFPGATimestamp());
    ;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    // FroggyPoseController.updateOdometry();
    // FroggyPoseController.outputPoseToDashboard();

    teleopControl();
  }

  /**
   * This function is called one time during operator control.
   */

  @Override
  public void teleopInit() {
    // mSwerve.setBrakeMode();
    mSwerve.snapForward();
    // Intake.getInstance().startVacuum();
    // LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);
    // PrecisionDriveController.activatePrecisionDrive();

    mBallIntake.onStart(Timer.getFPGATimestamp()); // executes init sequences 
    mDiskIntake.onStart(Timer.getFPGATimestamp()); // (initialize current states and local variables)
    mS.onStart(Timer.getFPGATimestamp());
  }

  /*
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    teleopControl();
  }

  public void teleopControl() {

    OILoop(); // Driver control
    OI.driverJoystick.update();
    OI.operator.update();
    mDiskIntake.stateRequest(DiskIntake.State.INTAKING);
    mBallIntake.onLoop(Timer.getFPGATimestamp()); // Updates reads and writes
    mDiskIntake.onLoop(Timer.getFPGATimestamp());
    mS.onLoop(Timer.getFPGATimestamp());
    mElevator.onLoop(Timer.getFPGATimestamp());
    mArm.onLoop(Timer.getFPGATimestamp());

    mArm.readPeriodicInputs();
    mElevator.readPeriodicInputs();
    mS.readPeriodicInputs();

    mArm.writePeriodicOutputs();
    mElevator.writePeriodicOutputs();
    Scheduler.getInstance().run();

    if (OI.operator.bButton.wasActivated()) { // B pressed or held
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(50.0, 33.0); // Presets for middle
      } else {
        mS.diskScoringState(50.0, -21.0);
      }
    } else if (OI.operator.aButton.wasActivated()) { // A
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(34.0, 0.0); // LOW
      } else { 
        mS.diskScoringState(Robot.bot.kElevatorHumanLoaderHeight, 0.0);
      }
    } else if (OI.operator.xButton.wasActivated()) { // X
      mS.ballScoringState(45.5, 0.0); // goes to cargo cargo ship regardless of element
    }
    if (OI.driverJoystick.aButton.wasActivated()) { // iff driver A presser
      mS.ballIntakingState(); // cargo intake
    } else if (OI.driverJoystick.yButton.wasActivated()) { // y
      mS.ballIntakingLoaderState();// human loader ball
    } else if (OI.driverJoystick.bButton.shortReleased()) { // B
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mBallIntake.conformToState(BallIntake.State.EJECTING); // SPIT BALL
      } else {
        mDiskIntake.conformToState(DiskIntake.State.RELEASING); // SPIT HATCH
      }
    } else if (OI.driverJoystick.xButton.wasActivated()) { // X
      mS.diskReceivingState(); // hatch intake
    } else if (OI.operator.yButton.wasActivated()) { 
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(45.5, 49.0); // BALL SAFE
      } else {
        mS.diskScoringState(45.5, -60.0); // HATCH SAFE
      }
    }

    // THIS SHOULD WORK TO RUMBLE WHEN SUCCESSFULLY PICKED UP
    if (mBallIntake.needsToNotifyDrivers() || mDiskIntake.needsToNotifyDivers()) {
      OI.driverJoystick.rumble(1.0, 2.0);
      OI.operator.rumble(1.0, 2.0);
    }

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
    // mSwerve.setBrakeMode();
    // subsystems.stop();
    // PrecisionDriveController.disablePrecisionDrive();

    mBallIntake.onStop(Timer.getFPGATimestamp());
  }

  @Override
  public void disabledPeriodic() {
    subsystems.outputToSmartDashboard();
  }

  private void OILoop() {
    OI.driverJoystick.update();
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
    default:
      break;
    }

    if (OI.driverJoystick.leftCenterClick.isBeingPressed()) {
      mSwerve.setMode(DriveMode.TeleopDrive);
    }

    if (RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
      operatorInput();
      AndyArm.getInstance().updateSuperstruture();
    }

  }
  boolean pressed = false;
  double timeReleased = -503.0;
  private void joystickInput() {
    double swerveYInput = -OI.getDriverLeftYVal();
    double swerveXInput = OI.getDriverLeftXVal();
    double swerveRotationInput = OI.getDriverRightXVal() * 0.5;
    boolean lowPower = OI.getDriverRightTriggerPressed();
    double deadband = 0.015;
    double rotDeadband = 0.1;
    double lastSnapTarget = 0;

    if (swerveRotationInput > -deadband && swerveRotationInput < rotDeadband) {
      swerveRotationInput = mSwerve.getRotationalOutput();

    } else {
      mSwerve.rotate(RobotState.getInstance().getCurrentTheta());
      pressed = true;
    }

    if (OI.getDriverYButton()) {
      mSwerve.visionFollow();
    } else {
      LimelightProcessor.getInstance().setPipeline(Pipeline.DRIVER);

      if (OI.driverJoystick.leftBumper.shortReleased()) {
        mSwerve.rotateButton(-30);
        swerveRotationInput = mSwerve.getRotationalOutput();
      } else if (OI.driverJoystick.leftBumper.longPressed()) {
        mSwerve.rotateButton(-150.0);
        swerveRotationInput = mSwerve.getRotationalOutput();
      } else if (OI.driverJoystick.rightBumper.shortReleased()) {
        mSwerve.rotateButton(30);
        swerveRotationInput = mSwerve.getRotationalOutput();
      } else if (OI.driverJoystick.rightBumper.longPressed()) {
        mSwerve.rotateButton(150.0);
        swerveRotationInput = mSwerve.getRotationalOutput();
      } else if (OI.driverJoystick.getPOV() == 180|| OI.driverJoystick.leftTrigger.wasActivated()) {
        mSwerve.rotateButton(179);
        swerveRotationInput = mSwerve.getRotationalOutput();
      } else if (OI.driverJoystick.getPOV() == 90) {
        mSwerve.rotateButton(90);
        swerveRotationInput = mSwerve.getRotationalOutput();
      } else if (OI.driverJoystick.getPOV() == 270) {
        mSwerve.rotateButton(270);
        swerveRotationInput = mSwerve.getRotationalOutput();
      } else if (OI.driverJoystick.getPOV() == 0) {
        mSwerve.rotateButton(1);
        swerveRotationInput = mSwerve.getRotationalOutput();
      } else if (OI.driverJoystick.getStartButtonPressed()) {
        mSwerve.setMode(DriveMode.Defense);
      }
      mSwerve.setFieldCentric(!OI.getDriverRightTriggerPressed());
      mSwerve.drive(swerveXInput, swerveYInput, swerveRotationInput, lowPower);
    }
  }

  private void operatorInput() {
    // if (OI.getOperatorA()) {
    // RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
    // TargetHeightSwitcher.set(RobotState.TargetHeight.LOW);
    // } else if (OI.getOperatorB()) {
    // if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
    // RobotState.getInstance().setArmDirection(ArmDirection.BACK);
    // } else {
    // RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
    // }
    // TargetHeightSwitcher.set(RobotState.TargetHeight.MIDDLE);
    // } else if (OI.getOperatorX()) {
    // RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
    // TargetHeightSwitcher.set(RobotState.TargetHeight.BUS);
    // } else if (OI.getOperatorY()) {
    // if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
    // RobotState.getInstance().setArmDirection(ArmDirection.BACK);
    // } else {
    // RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
    // }
    // TargetHeightSwitcher.set(RobotState.TargetHeight.HIGH);
    // } else if (OI.getOperatorMenu()) {
    // RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
    // TargetHeightSwitcher.set(RobotState.TargetHeight.INTAKE);
    // } else if (OI.getOperatorRightBumper()) {
    // TargetHeightSwitcher.set(RobotState.TargetHeight.HOME);
    // } else if (OI.getOperatorLeftBumper()) {
    // SwitchArmDirection.flip();
    // } else if (OI.getOperatorHatchSwitch()) {
    // GameElementSwitcher.setGameElement(GameElement.HATCH);
    // } else if (OI.getOperatorCargoSwitch()) {
    // GameElementSwitcher.setGameElement(GameElement.CARGO);
    // } else if (OI.getOperatorSelect()) {
    // ToggleControlMode.toggle();
    // } else if (OI.getDriverXButton()) {
    // ToggleIntake.toggleIntake();
    // }
    // ToggleIntake.handleIntakeFinish();
    // if (OI.getDriverBButton()) {
    // EjectBall.eject();
    // } else {
    // EjectBall.stopEject();
    // }
    // if (OI.getDriverAButton()) {
    // ReleaseHatch.startRelease();
    // }
    // ReleaseHatch.handleFinish();
    // if (OI.getOperatorRJ()) {
    // ResetEncoderCommand.resetEncs();
    // }

  }
}