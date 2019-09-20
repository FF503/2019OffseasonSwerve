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
import com.team503.robot.RobotState.SuperStructurePreset;
import com.team503.robot.commands.EjectBall;
import com.team503.robot.commands.GameElementSwitcher;
import com.team503.robot.commands.ReleaseHatch;
import com.team503.robot.commands.ResetEncoderCommand;
import com.team503.robot.commands.SwitchArmDirection;
import com.team503.robot.commands.TargetHeightSwitcher;
import com.team503.robot.commands.ToggleControlMode;
import com.team503.robot.commands.ToggleIntake;
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
// import com.team503.robot.subsystems.SwerveDrive.DriveMode;

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

  private SwerveDrive mSwerve;
  private Elevator mElevator;
  private Arm mArm;
  private BallIntake mBallIntake;
  private DiskIntake mDiskIntake;
  private Superstructure mS;

  private SubsystemManager subsystems;
  public static RobotHardware bot;
  private double theta = Math.PI / 2;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    RobotState.getInstance().setCurrentRobot(Bot.FFSwerve);
    bot = RobotHardware.getInstance();
    OI.initialize();

    mSwerve = SwerveDrive.getInstance();
    mElevator = Elevator.getInstance();
    mArm = Arm.getInstance();
    mBallIntake = BallIntake.getInstance();
    mDiskIntake = DiskIntake.getInstance();
    mS = Superstructure.getInstance();

    // Subsytem Manager

    if (RobotState.getInstance().getCurrentRobot().equals(Bot.FFSwerve)) {
      subsystems = new SubsystemManager(
          Arrays.asList(Pigeon.getInstance(), mElevator, mArm, mBallIntake, mDiskIntake, mS));
    } else if (RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
      subsystems = new SubsystemManager(Arrays.asList(mSwerve, Pigeon.getInstance(), AndyArm.getInstance(),
          AndyWrist.getInstance(), Extension.getInstance(), Intake.getInstance()));
    }
    // subsystems.resetSensor();
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
    // System.out.println("ROBOT"+
    // RobotState.getInstance().getCurrentRobot().name());
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
    // Pigeon.getInstance().zeroSensors();
    // FroggyPoseController.resetPose(new Pose(0,0,0));
    // mSwerve.setBrakeMode();
    // Intake.getInstance().startVacuum();
    // mSwerve.resetDriveEncoder();
    // LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);
    // Pose target = new Pose(0.0,100.0,270.0);
    // DriveToPose driveCommand = new DriveToPose(target);
    // driveCommand.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    RobotState.getInstance().setCurrentTheta(Pigeon.getInstance().getYaw());
    OI.driverJoystick.update();

    oneControllerMode();
  }

  /**
   * This function is called one time during operator control.
   */

  @Override
  public void teleopInit() {
    // mSwerve.setBrakeMode();
    // mSwerve.snapForward();
    // Intake.getInstance().startVacuum();
    // LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);
    // PrecisionDriveController.activatePrecisionDrive();
    mBallIntake.onStart(Timer.getFPGATimestamp());
    mDiskIntake.onStart(Timer.getFPGATimestamp());
    mS.onStart(Timer.getFPGATimestamp());
  }

  /*
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // subsystems.outputToSmartDashboard();

    RobotState.getInstance().setCurrentTheta(Pigeon.getInstance().getYaw());

    
    mArm.outputTelemetry();
    mS.outputTelemetry();
    mElevator.outputTelemetry();
    mBallIntake.outputTelemetry();
    mDiskIntake.outputTelemetry();

    // double targetHeight, targetAngle;

    // OILoop();
    // mElevator.setOpenLoop(-OI.getDriverLeftYVal());
    OI.driverJoystick.update();
    OI.operator.update();
    mDiskIntake.stateRequest(DiskIntake.State.INTAKING);
    // mDiskIntake.getSpark().set(0.4);

    // if(OI.getDriverAButton()) {

    // targetHeight = 45.5;
    // targetAngle = 0.0;
    // } else {
    // targetHeight = 23.0;
    // targetAngle = 0.0;
    // }

    // if(OI.driverJoystick.bButton.isBeingPressed()) {
    // mIntake.conformToState(State.EJECTING);
    // }
    // else if(OI.driverJoystick.xButton.shortReleased()) {
    // mIntake.conformToState(State.INTAKING);
    // } else if(OI.driverJoystick.yButton.isBeingPressed()) {
    // // mIntake.setRelease(true);
    // } else {
    // // mIntake.setRelease(false);

    // }

    // mElevator.setTargetHeight(targetHeight);
    // mElevator.readPeriodicInputs();
    // mElevator.writePeriodicOutputs();
    // mElevator.outputTelemetry();

    // // mArm.setOpenLoop(-OI.getDriverRightYVal());
    // mArm.setAngle(targetAngle);
    // mArm.readPeriodicInputs();
    // mArm.writePeriodicOutputs();
    // mArm.outputTelemetry();

    mBallIntake.onLoop(Timer.getFPGATimestamp());
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

    oneControllerMode();

    if (OI.operator.bButton.wasActivated()) {
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(50.0, 33.0);
      } else {
        mS.diskScoringState(50.0, -21.0);
      }
    } else if (OI.operator.aButton.wasActivated()) {
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(34.0, 0.0);
      } else {
        mS.diskScoringState(Robot.bot.kElevatorHumanLoaderHeight, 0.0);
      }
    } else if (OI.operator.xButton.wasActivated()) {
      // if (mS.getCurrentElement() == Superstructure.Element.BALL) {
      mS.ballScoringState(45.5, 0.0);
      // }
    }
    if (OI.driverJoystick.aButton.wasActivated()) {
      mS.ballIntakingState();
    } else if (OI.driverJoystick.bButton.shortReleased()) {
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mBallIntake.conformToState(BallIntake.State.EJECTING);
      } else {
        mDiskIntake.conformToState(DiskIntake.State.RELEASING);
      }
    } else if (OI.driverJoystick.xButton.wasActivated()) {
      mS.diskReceivingState();
    } else if (OI.driverJoystick.yButton.wasActivated()) {
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(45.5, 49.0);
      } else {
        mS.diskScoringState(45.5, -60.0);
      }
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
    subsystems.stop();
    mSwerve.onStop(Timer.getFPGATimestamp());
  }

  @Override
  public void disabledPeriodic() {
    // subsystems.outputToSmartDashboard();
  }

  private void azimuthDebugInput() {
    double swerveYInput = 0.3 * Math.sin(theta);
    double swerveXInput = 0.3 * Math.cos(theta);
    double swerveRotationInput = OI.getDriverRightXVal();

    theta += 0.1;
    theta %= 2 * Math.PI;

    mSwerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, false);

    // mSwerve.onLoop(Timer.getFPGATimestamp());
  }

  public void oneControllerMode() {
    double swerveYInput = -OI.getDriverLeftYVal();
    double swerveXInput = OI.getDriverLeftXVal();

    double swerveRotationInput = OI.getDriverRightXVal();
    boolean lowPower = OI.getDriverRightTriggerPressed();
    double deadband = 0.015;
    double lastSnapTarget = 0;

    if (OI.driverJoystick.startButton.shortReleased()) {
      mSwerve.toggleEvade();
    }

    mSwerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, lowPower, lowPower);

    mSwerve.onLoop(Timer.getFPGATimestamp());
  }

  public void twoControllerMode() {
    oneControllerMode();

    if (OI.operator.bButton.wasActivated()) {
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(50.0, 33.0);
      } else {
        mS.diskScoringState(50.0, -21.0);
      }
    } else if (OI.operator.aButton.wasActivated()) {
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(34.0, 0.0);
      } else {
        mS.diskScoringState(Robot.bot.kElevatorHumanLoaderHeight, 0.0);
      }
    } else if (OI.operator.xButton.wasActivated()) {
      mS.ballScoringState(45.5, 0.0);
    }
    if (OI.driverJoystick.aButton.wasActivated()) {
      mS.ballIntakingState();
    } else if (OI.driverJoystick.bButton.shortReleased()) {
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mBallIntake.conformToState(BallIntake.State.EJECTING);
      } else {
        mDiskIntake.conformToState(DiskIntake.State.RELEASING);
      }
    } else if (OI.driverJoystick.xButton.wasActivated()) {
      mS.diskReceivingState();
    } else if (OI.driverJoystick.yButton.wasActivated()) {
      if (mS.getCurrentElement() == Superstructure.Element.BALL) {
        mS.ballScoringState(45.5, 49.0);
      } else {
        mS.diskScoringState(45.5, -60.0);
      }
    }
  }

}
