/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.subsystems;

import com.team503.robot.Robot;
import com.team503.robot.subsystems.requests.Prerequisite;
import com.team503.robot.subsystems.requests.Request;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallIntake extends Subsystem {
  private static BallIntake instance = null;

  public static BallIntake getInstance() {
    if (instance == null)
      instance = new BallIntake();
    return instance;
  }

  boolean hasBall = false;

  public synchronized boolean hasBall() {
    return hasBall;
  }

  public void feignBall() {
    hasBall = true;
  }

  private Spark intake; // feeder;
  private Spark suction;
  private Solenoid release;
  private DigitalInput banner;
  private PowerDistributionPanel pdp;
  private Compressor c;

  // public LazyTalonSRX getPigeonTalon() {
  // return feeder;
  // }

  public boolean getBanner() {
    return banner.get();
  }

  private BallIntake() {
    intake = new Spark(Robot.bot.BALL_INTAKE);
    // feeder = new LazyTalonSRX(Robot.bot.BALL_FEEDER);
    banner = new DigitalInput(Robot.bot.BALL_INTAKE_BANNER);
    suction = new Spark(8);
    release = new Solenoid(0);

    c = new Compressor();

    intake.setInverted(true);

    // intake.setNeutralMode(NeutralMode.Brake);

    // intake.configVoltageCompSaturation(12.0, 10);
    // intake.enableVoltageCompensation(true);

    // intake.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 10);
    // intake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);

    // feeder.setInverted(false);

    // feeder.setNeutralMode(NeutralMode.Brake);

    // feeder.configVoltageCompSaturation(12.0, 10);
    // feeder.enableVoltageCompensation(true);

    // feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 10);
    // feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, 10);

    // setCurrentLimit(30);
    pdp = new PowerDistributionPanel(0);
  }

  // public void setCurrentLimit(int amps) {
  // intake.configContinuousCurrentLimit(amps, 10);
  // intake.configPeakCurrentLimit(amps);
  // intake.configPeakCurrentDuration(10, 10);
  // intake.enableCurrentLimit(true);

  // feeder.configContinuousCurrentLimit(amps, 10);
  // feeder.configPeakCurrentLimit(amps);
  // feeder.configPeakCurrentDuration(10, 10);
  // feeder.enableCurrentLimit(true);
  // }

  // public void enableCurrrentLimit(boolean enable) {
  // intake.enableCurrentLimit(enable);
  // feeder.enableCurrentLimit(enable);
  // }

  // private void setRampRate(double secondsToMax) {
  // intake.configOpenloopRamp(secondsToMax, 0);
  // feeder.configOpenloopRamp(secondsToMax, 0);
  // }

  public enum State {
    OFF(0), INTAKING(Robot.bot.kIntakingOutput), EJECTING(Robot.bot.kIntakeEjectOutput),
    HOLDING(Robot.bot.kIntakingOutput), PULLING(Robot.bot.kIntakePullOutput),
    FEEDING(Robot.bot.kIntakeWeakHoldingOutput), POST_FEEDING(0);

    public double intakeOutput = 0;
    // public double feederOutput = 0;

    private State(double grabberSpeed) {// , double feederSpeed) {
      intakeOutput = grabberSpeed;
      // feederOutput = feederSpeed;
    }
  }

  private State currentState = State.OFF;
  private boolean stateChanged = false;
  private double bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
  private double stateEnteredTimestamp = 0;
  private double holdingOutput = Robot.bot.kIntakeWeakHoldingOutput;
  private boolean isConstantSuck = false;

  public State getState() {
    return currentState;
  }

  private synchronized void setState(State newState) {
    if (newState != currentState)
      stateChanged = true;
    currentState = newState;
    stateEnteredTimestamp = Timer.getFPGATimestamp();
  }

  public void setHoldingOutput(double output) {
    holdingOutput = output;
  }

  public void setSuctionOutput(double output) {
    suction.set(output);
    c.start();
  }

  public void setRelease(boolean release) {
    this.release.set(release);
  }

  private boolean needsToNotifyDrivers = false;

  public boolean needsToNotifyDrivers() {
    if (needsToNotifyDrivers) {
      needsToNotifyDrivers = false;
      return true;
    }
    return false;
  }

  private void setIntakeSpeed(double output) {
    // setRampRate(0.0);
    intake.set(output);
  }

  // private void setFeederSpeed(double output) {
  // feeder.set(ControlMode.PercentOutput, output);
  // }

  private void holdRollers() {
    setIntakeSpeed(holdingOutput);
  }

  // private final Loop loop = new Loop() {

  // @Override
  public void onStart(double timestamp) {
    hasBall = false;
    needsToNotifyDrivers = false;
    setState(State.OFF);
    stop();
  }

  // @Override
  public void onLoop(double timestamp) {
    switch (currentState) {
    case OFF:
      break;
    case INTAKING:
      if (stateChanged)
        hasBall = false;
      if ((pdp.getCurrent(11) >= 10) && ((timestamp - stateEnteredTimestamp) >= 0.5)) {
        // if (Double.isInfinite(bannerSensorBeganTimestamp)) {
        // bannerSensorBeganTimestamp = timestamp;
        // } else {
        // if (timestamp - bannerSensorBeganTimestamp > 0.3) {
        // hasBall = true;
        // needsToNotifyDrivers = true;
        // }
        // }
        // } else if (!Double.isFinite(bannerSensorBeganTimestamp)) {
        // bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
        setState(State.HOLDING);
      }
      break;
    case EJECTING:
      if (stateChanged) {
        // setRampRate(0.0);
        hasBall = false;
      }
      if (timestamp - stateEnteredTimestamp > 2.0) {
        stop();
        // setRampRate(Robot.bot.kIntakeRampRate);
      }
      break;
    case HOLDING:
      /*
       * if (banner.get()) { if (isConstantSuck) { holdRollers(); isConstantSuck =
       * false; } } else { if (!isConstantSuck) {
       * setGrabberSpeed(Robot.bot.kIntakingResuckingOutput); isConstantSuck = true; }
       * }
       */
      if ((timestamp - stateEnteredTimestamp) >= 1.0) {
        setIntakeSpeed(Robot.bot.kIntakeWeakHoldingOutput);
      }
      break;
    // case CLIMBING:
    // break;
    case POST_FEEDING:
      if (stateChanged)
        hasBall = false;
      break;
    default:
      break;
    }
    if (stateChanged)
      stateChanged = false;
  }

  // @Override
  public void onStop(double timestamp) {
    setState(State.OFF);
    stop();
  }

  // };

  public void eject(double output) {
    setState(State.EJECTING);
    setIntakeSpeed(output);
    hasBall = false;
  }

  public void conformToState(State desiredState) {
    setState(desiredState);
    setIntakeSpeed(desiredState.intakeOutput);
    // setFeederSpeed(desiredState.feederOutput);
  }

  public void conformToState(State desiredState, double outputOverride) {
    setState(desiredState);
    setIntakeSpeed(outputOverride);
    // setFeederSpeed(outputOverride);
  }

  public Request stateRequest(State desiredState) {
    return new Request() {

      @Override
      public void act() {
        conformToState(desiredState);
      }

    };

  }

  public Request waitForBallRequest() {
    return new Request() {

      @Override
      public void act() {
        conformToState(State.INTAKING);
      }

      @Override
      public boolean isFinished() {
        return !stateChanged && hasBall;
      }

    };
  }

  public Request ejectRequest(double output) {
    return new Request() {

      @Override
      public void act() {
        conformToState(State.EJECTING, output);
      }
    };
  }

  public Prerequisite ballRequisite() {
    return new Prerequisite() {

      @Override
      public boolean met() {
        return hasBall();
      }
    };
  }

  @Override
  public void outputTelemetry() {
    if (Robot.bot.kDebuggingOutput) {
      // SmartDashboard.putNumber("Intake Grabber Current",
      // intake.getOutputCurrent());
      // SmartDashboard.putNumber("Intake Grabber Voltage",
      // intake.getMotorOutputVoltage());
      // SmartDashboard.putNumber("Intake Feeder Current", feeder.getOutputCurrent());
      // SmartDashboard.putNumber("Intake Feeder Voltage",
      // feeder.getMotorOutputVoltage());
      SmartDashboard.putBoolean("Intake Has Ball", hasBall);
      SmartDashboard.putBoolean("Intake Banner", banner.get());
    }

  }

  @Override
  public synchronized void stop() {
    conformToState(State.OFF);
  }

  // @Override
  // public void registerEnabledLoops(ILooper enabledLooper) {
  // enabledLooper.register(loop);
  // }

}
