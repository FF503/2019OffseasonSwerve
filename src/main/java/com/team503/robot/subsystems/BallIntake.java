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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
  private AnalogInput banner;
  private PowerDistributionPanel pdp;

  public boolean getBanner() {
    return banner.getVoltage() > 10.0;
  }

  private BallIntake() {
    intake = new Spark(Robot.bot.BALL_INTAKE);
    banner = new AnalogInput(Robot.bot.BALL_INTAKE_BANNER);

    intake.setInverted(true);

    pdp = new PowerDistributionPanel(0);
  }

  public enum State {
    OFF(0), INTAKING(Robot.bot.kIntakingOutput), EJECTING(Robot.bot.kIntakeEjectOutput),
    HOLDING(Robot.bot.kIntakeWeakHoldingOutput);

    public double intakeOutput = 0;

    private State(double grabberSpeed) {
      intakeOutput = grabberSpeed;
    }
  }

  private State currentState = State.OFF;
  private boolean stateChanged = false;
  private double bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
  private double stateEnteredTimestamp = 0;
  private double holdingOutput = Robot.bot.kIntakeWeakHoldingOutput;

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

  private boolean needsToNotifyDrivers = false;

  public boolean needsToNotifyDrivers() {
    if (needsToNotifyDrivers) {
      needsToNotifyDrivers = false;
      return true;
    }
    return false;
  }

  private void setIntakeSpeed(double output) {
    intake.set(output);
  }

  private void holdRollers() {
    setIntakeSpeed(holdingOutput);
  }

  public void onStart(double timestamp) {
    hasBall = false;
    needsToNotifyDrivers = false;
    setState(State.OFF);
    stop();
  }

  public void onLoop(double timestamp) {
    switch (currentState) {
    case OFF:
      break;
    case INTAKING:
      if (stateChanged)
        hasBall = false;
      if ((pdp.getCurrent(11) >= 10) && ((timestamp - stateEnteredTimestamp) >= 0.5)) {
        if (Double.isInfinite(bannerSensorBeganTimestamp)) {
          bannerSensorBeganTimestamp = timestamp;
        } else {
          if (timestamp - bannerSensorBeganTimestamp > 0.3) {
            hasBall = true;
            needsToNotifyDrivers = true;
          }
        }
      } else if (!Double.isFinite(bannerSensorBeganTimestamp)) {
        bannerSensorBeganTimestamp = Double.POSITIVE_INFINITY;
      }
      break;
    case EJECTING:
      if (stateChanged) {
        // setRampRate(0.0);
        hasBall = false;
      }
      if (timestamp - stateEnteredTimestamp > 1.0) {
        conformToState(State.OFF);
      }
      break;
    case HOLDING:
      if ((timestamp - stateEnteredTimestamp) >= 1.0) {
        setIntakeSpeed(Robot.bot.kIntakeWeakHoldingOutput);
      }
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


  public void conformToState(State desiredState) {
    setState(desiredState);
    setIntakeSpeed(desiredState.intakeOutput);
  }

  public void conformToState(State desiredState, double outputOverride) {
    setState(desiredState);
    setIntakeSpeed(outputOverride);
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
      SmartDashboard.putNumber("Intake Banner", banner.getVoltage());
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
