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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages the station suction intake
 **/
public class DiskIntake extends Subsystem {
  private static DiskIntake instance = null;

  public static DiskIntake getInstance() {
    if (instance == null)
      instance = new DiskIntake();
    return instance;
  }

  private boolean hasDisk = false;

  public synchronized boolean hasDisk() {
    return hasDisk;
  }

  public void feignDisk() {
    hasDisk = false;
  }

  private Spark intake;

  public Spark getSpark() {
    return intake;
  }

  private Solenoid release;

  private AnalogInput pressureGauge;

  private DiskIntake() {
    intake = new Spark(Robot.bot.DISK_INTAKE);
    release = new Solenoid(Robot.bot.DISK_RELEASE);
    pressureGauge = new AnalogInput(Robot.bot.DISK_SENSOR);

    intake.setInverted(false);
  }

  public enum State {
    OFF(0, false), INTAKING(Robot.bot.kDiskIntakingOutput, false), RELEASING(Robot.bot.kDiskIntakeEjectOutput, true),
    // HANDOFF_COMPLETE(0, true),
    HOLDING(Robot.bot.kDiskIntakingOutput, false),
    // DEPLOYED(0, false),
    // DELIVERING(4.0/12.0, true),
    DISABLED(0.0, false);

    public double diskIntakeOutput = 0;
    public boolean releasing = false;

    private State(double output, boolean release) {
      diskIntakeOutput = output;
      releasing = release;
    }

  }

  private State currentState = State.OFF;

  public State getState() {
    return currentState;
  }

  private double stateEnteredTimestamp = 0;
  private boolean stateChanged = false;
  private double pressureSpikeTimestamp = Double.POSITIVE_INFINITY;
  // private boolean isResucking = false;
  private double holdingOutput = Robot.bot.kDiskIntakingOutput;
  private boolean needsToNotifyDrivers = false;

  private synchronized void setState(State newState) {
    if (newState != currentState) {
      stateChanged = true;
    }
    currentState = newState;
    stateEnteredTimestamp = Timer.getFPGATimestamp();
  }

  // public void setHoldingOutput(double output) {
  // holdingOutput = output;
  // }

  public boolean needsToNotifyDivers() {
    if (needsToNotifyDrivers) {
      needsToNotifyDrivers = false;
      return true;
    }
    return false;
  }

  public void fireRelease(boolean fire) {
    release.set(fire);
  }

  private void setVacuum(double speed) {
    // setRampRate(0.0);
    intake.set(speed);
  }

  private void holdRollers() {
    setVacuum(holdingOutput);
  }

  boolean hasPower = true;

  // public void shiftPower(boolean shiftToJacks) {
  // hasPower = !shiftToJacks;
  // conformToState(shiftToJacks ? State.DISABLED : State.OFF);
  // if (!shiftToJacks)
  // configureTalon();
  // }

  // private final Loop loop = new Loop() {

  // @Override
  public void onStart(double timestamp) {
    hasDisk = false;
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
        hasDisk = false;
      if (pressureGauge.getVoltage() >= 10.0 && (timestamp - stateEnteredTimestamp) >= 0.5) {
        if (Double.isInfinite(pressureSpikeTimestamp)) {
          pressureSpikeTimestamp = timestamp;
        } else {
          if (timestamp - pressureSpikeTimestamp > 0.375) {
            hasDisk = true;
            needsToNotifyDrivers = true;
          }
        }
      } else if (!Double.isInfinite(pressureSpikeTimestamp)) {
        pressureSpikeTimestamp = Double.POSITIVE_INFINITY;
      }
      break;
    case RELEASING:
      if (stateChanged) {
        // setRampRate(0.0);
        hasDisk = false;
      }
      if (timestamp - stateEnteredTimestamp > 2.0) {
        stop();
        // setRampRate(Robot.bot.kDiskIntakeRampRate);
      }
      break;
    // case HANDOFF_COMPLETE:
    // if (stateChanged)
    // hasDisk = false;
    // break;
    case HOLDING:
      /*
       * if(banner.get()) { if(isResucking) { holdRollers(); isResucking = false; } }
       * else { if (!isResucking) {
       * setRollers(Robot.bot.kDiskIntakingResuckingOutput); isResucking = true; } }
       */
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

  public void release(double output) {
    setState(State.RELEASING);
    setVacuum(output);
    fireRelease(true);
    hasDisk = false;
  }

  public void conformToState(State desiredState) {
    conformToState(desiredState, desiredState.diskIntakeOutput);
  }

  public void conformToState(State desiredState, double outputOverride) {
    // if (hasPower || (!hasPower && desiredState == State.DISABLED)) {
    setState(desiredState);
    setVacuum(outputOverride);
    fireRelease(desiredState.releasing);
    // } else {
    // DriverStation.reportError("Disk intake state change not allowed", false);
    // }
  }

  public Request stateRequest(State desiredState) {
    return new Request() {

      @Override
      public void act() {
        conformToState(desiredState);
      }
    };
  }

  public Request waitForDiskRequest() {
    return new Request() {

      @Override
      public void act() {
        conformToState(State.INTAKING);
      }

      @Override
      public boolean isFinished() {
        return !stateChanged && hasDisk;
      }

    };
  }

  public Request ejectRequest(double output) {
    return new Request() {

      @Override
      public void act() {
        conformToState(State.RELEASING, output);
      }

    };
  }

  public Prerequisite diskRequisite() {
    return new Prerequisite() {

      @Override
      public boolean met() {
        return hasDisk();
      }
    };
  }

  @Override
  public synchronized void stop() {
    conformToState(State.OFF);
  }

  // @Override
  // public void registerEnabledLoops(ILooper enabledLooper) {
  // enabledLooper.register(loop);
  // }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putString("Disk intake state", currentState.toString());
    if (Robot.bot.kDebuggingOutput) {
      SmartDashboard.putNumber("Disk Intake Pressure", pressureGauge.getVoltage());
      SmartDashboard.putNumber("Disk Intake Voltage", intake.get() * 12.0);
      SmartDashboard.putBoolean("Disk Intake Has Disk", hasDisk);
    }

  }
}
