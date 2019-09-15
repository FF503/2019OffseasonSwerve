/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import com.team503.robot.RobotState.ArmDirection;
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
import com.team503.robot.loops.LimelightProcessor;
import com.team503.robot.loops.LimelightProcessor.Pipeline;
import com.team503.robot.subsystems.AndyArm;
import com.team503.robot.subsystems.Arm;
import com.team503.robot.subsystems.BallIntake;
import com.team503.robot.subsystems.DiskIntake;
import com.team503.robot.subsystems.Elevator;
import com.team503.robot.subsystems.SubsystemManager;
import com.team503.robot.subsystems.Superstructure;
import com.team503.robot.subsystems.SwerveDrive;
import com.team503.robot.subsystems.BallIntake.State;
import com.team503.robot.subsystems.SwerveDrive.DriveMode;

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

  // private SwerveDrive mSwerve;
  private Elevator mElevator;
  private Arm mArm;
  private BallIntake mBallIntake;
  private DiskIntake mDiskIntake;
  private Superstructure s;

  private SubsystemManager subsystems;
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

    // mSwerve = SwerveDrive.getInstance();
    mElevator = Elevator.getInstance();
    mArm = Arm.getInstance();
    mBallIntake = BallIntake.getInstance();
    mDiskIntake = DiskIntake.getInstance();
    s = Superstructure.getInstance();

    // Subsytem Manager

    // if (RobotState.getInstance().getCurrentRobot().equals(Bot.FFSwerve)) {
    //   subsystems = new SubsystemManager(Arrays.asList(mSwerve, Pigeon.getInstance()));
    // } else if(RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)){
    //   subsystems = new SubsystemManager(Arrays.asList(mSwerve, Pigeon.getInstance(), AndyArm.getInstance(),
    //       AndyWrist.getInstance(), Extension.getInstance(), Intake.getInstance()));
    // }
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
    // subsystems.outputToSmartDashboard();
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
    //OILoop();

    // FroggyPoseController.updateOdometry();
    // FroggyPoseController.outputPoseToDashboard();

    // Scheduler.getInstance().run();
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
    s.onStart(Timer.getFPGATimestamp());
    s.enableCompressor(true);
  } 

  /*
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    // double targetHeight, targetAngle;
  
    // OILoop();
    // mElevator.setOpenLoop(-OI.getDriverLeftYVal());
    OI.driverJoystick.update();
    OI.operator.update();
    // mDiskIntake.stateRequest(DiskIntake.State.INTAKING);
    mDiskIntake.getSpark().set(0.4);



    // if(OI.getDriverAButton()) {

    //   targetHeight = 45.5;
    //   targetAngle = 0.0;
    // } else {
    //   targetHeight = 23.0;
    //   targetAngle = 0.0;
    // }

    // if(OI.driverJoystick.bButton.isBeingPressed()) {
    //   mIntake.conformToState(State.EJECTING);
    // }
    // else if(OI.driverJoystick.xButton.shortReleased()) {
    //   mIntake.conformToState(State.INTAKING);
    // } else if(OI.driverJoystick.yButton.isBeingPressed()) {
    //   // mIntake.setRelease(true);
    // } else {
    //   // mIntake.setRelease(false);

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
    s.onLoop(Timer.getFPGATimestamp());
    mElevator.onLoop(Timer.getFPGATimestamp());
    mArm.onLoop(Timer.getFPGATimestamp());

    mArm.readPeriodicInputs();
    mElevator.readPeriodicInputs();
    s.readPeriodicInputs();

    mArm.writePeriodicOutputs();
    mElevator.writePeriodicOutputs();
    // mIntake.setSuctionOutput(0.4);
    // FroggyPoseController.updateOdometry();
    // FroggyPoseController.outputPoseToDashboard();
    Scheduler.getInstance().run();



    if (OI.operator.yButton.wasActivated()) {
      s.ballScoringState(45.5, 0.0);
    } 
    if (OI.driverJoystick.aButton.wasActivated()) {
      s.ballIntakingState();
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
    // subsystems.outputToSmartDashboard();
  }

  // private void OILoop() {
  //   OI.driverJoystick.update();
  //   switch (mSwerve.getMode()) {
  //   case TeleopDrive:
  //     joystickInput();
  //     break;
  //   case Defense:
  //     if (!OI.driverJoystick.getStartButton()) {
  //       mSwerve.setMode(DriveMode.TeleopDrive);
  //       break;
  //     }
  //     mSwerve.defensePosition();
  //     break;
  //   default:
  //     break;
  //   }

  //   if(OI.driverJoystick.leftCenterClick.isBeingPressed()) {
  //     mSwerve.setMode(DriveMode.TeleopDrive);
  //   }

  //   if (RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
  //     operatorInput();
  //     AndyArm.getInstance().updateSuperstruture();
  //   }
    
  // }

  // private void joystickInput() {
  //   double swerveYInput = -OI.getDriverLeftYVal();
  //   double swerveXInput = OI.getDriverLeftXVal();
  //   double swerveRotationInput = OI.getDriverRightXVal();
  //   boolean lowPower = OI.getDriverRightTriggerPressed();
  //   double deadband = 0.015;
  //   double lastSnapTarget = 0;

  //   if (swerveRotationInput > -deadband && swerveRotationInput < deadband) {
  //     swerveRotationInput = mSwerve.getRotationalOutput();
  //   } else {
  //     mSwerve.rotate(RobotState.getInstance().getCurrentTheta());
  //   }

  //   if (OI.getDriverYButton()) {
  //     mSwerve.visionFollow();
  //   } else {
  //     LimelightProcessor.getInstance().setPipeline(Pipeline.DRIVER);
  //     if (OI.driverJoystick.leftBumper.shortReleased()) {
  //       mSwerve.rotate(-30);
  //       swerveRotationInput = mSwerve.getRotationalOutput();
  //     } else if (OI.driverJoystick.leftBumper.longPressed()) {
  //       mSwerve.rotate(-150.0);
  //       swerveRotationInput = mSwerve.getRotationalOutput();
  //     } else if (OI.driverJoystick.rightBumper.shortReleased()) {
  //       mSwerve.rotate(30);
  //       swerveRotationInput = mSwerve.getRotationalOutput();
  //     } else if (OI.driverJoystick.rightBumper.longPressed()) {
  //       mSwerve.rotate(150.0);
  //       swerveRotationInput = mSwerve.getRotationalOutput();
  //     } else if (OI.driverJoystick.getPOV() == 180) {
  //       mSwerve.rotate(179);
  //       swerveRotationInput = mSwerve.getRotationalOutput();
  //     } else if (OI.driverJoystick.getPOV() == 90) {
  //       mSwerve.rotate(90);
  //       swerveRotationInput = mSwerve.getRotationalOutput();
  //     } else if (OI.driverJoystick.getPOV() == 270) {
  //       mSwerve.rotate(270);
  //       swerveRotationInput = mSwerve.getRotationalOutput();
  //     } else if (OI.driverJoystick.getPOV() == 0) {
  //       mSwerve.rotate(1);
  //       swerveRotationInput = mSwerve.getRotationalOutput();
  //     } else if (OI.driverJoystick.getStartButtonPressed()) {
  //       mSwerve.setMode(DriveMode.Defense);
  //     }
  //     mSwerve.setFieldCentric(!OI.getDriverRightTriggerPressed());
  //     mSwerve.drive(swerveXInput, swerveYInput, swerveRotationInput, lowPower);
  //   }
  // }

  private void operatorInput() {
    if (OI.getOperatorA()) {
      RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
      TargetHeightSwitcher.set(RobotState.TargetHeight.LOW);
    } else if (OI.getOperatorB()) {
      if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
        RobotState.getInstance().setArmDirection(ArmDirection.BACK);
      } else {
        RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
      }
      TargetHeightSwitcher.set(RobotState.TargetHeight.MIDDLE);
    } else if (OI.getOperatorX()) {
      RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
      TargetHeightSwitcher.set(RobotState.TargetHeight.BUS);
    } else if (OI.getOperatorY()) {
      if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
        RobotState.getInstance().setArmDirection(ArmDirection.BACK);
      } else {
        RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
      }
      TargetHeightSwitcher.set(RobotState.TargetHeight.HIGH);
    } else if (OI.getOperatorMenu()) {
      RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
      TargetHeightSwitcher.set(RobotState.TargetHeight.INTAKE);
    } else if (OI.getOperatorRightBumper()) {
      TargetHeightSwitcher.set(RobotState.TargetHeight.HOME);
    } else if (OI.getOperatorLeftBumper()) {
      SwitchArmDirection.flip();
    } else if (OI.getOperatorHatchSwitch()) {
      GameElementSwitcher.setGameElement(GameElement.HATCH);
    } else if (OI.getOperatorCargoSwitch()) {
      GameElementSwitcher.setGameElement(GameElement.CARGO);
    } else if (OI.getOperatorSelect()) {
      ToggleControlMode.toggle();
    } else if (OI.getDriverXButton()) {
      ToggleIntake.toggleIntake();
    }
    ToggleIntake.handleIntakeFinish();
    if (OI.getDriverBButton()) {
      EjectBall.eject();
    } else {
      EjectBall.stopEject();
    }
    if (OI.getDriverAButton()) {
      ReleaseHatch.startRelease();
    }
    ReleaseHatch.handleFinish();
    if (OI.getOperatorRJ()) {
      ResetEncoderCommand.resetEncs();
    }

  }
}