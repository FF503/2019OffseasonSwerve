/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team503.lib.util.FFDashboard;
import com.team503.lib.util.MotionMagicPID;
import com.team503.robot.Robot;
import com.team503.robot.commands.SuperStructureCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for arm motion magic trajectories
 */
public class Arm extends Subsystem implements SuperStructureSystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX armMaster, armSlave;
  private FFDashboard table = new FFDashboard("Arm");
  private MotionMagicPID armPID;
  private double lastVel, currVel, accel, maxVel, maxAccel, minAccel = 0;

  public Arm() {
    // if (!RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
    armMaster = new TalonSRX(Robot.bot.armMasterID);
    armSlave = new TalonSRX(Robot.bot.armSlaveID);
    armMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative /* CTRE_MagEncoder_Absolute */,
        Robot.bot.gSlotIdx, Robot.bot.gTimeoutMs);

    armMaster.setSensorPhase(Robot.bot.armMasterSensorPhase);
    armMaster.setInverted(Robot.bot.armMasterInverted);
    armSlave.setInverted(Robot.bot.armSlaveInverted);
    armMaster.setNeutralMode(NeutralMode.Brake);
    armSlave.setNeutralMode(NeutralMode.Brake);
    armMaster.selectProfileSlot(0, 0);

    // armMaster.getSensorCollection().setPulseWidthPosition(0, 10);

    armSlave.set(ControlMode.Follower, Robot.bot.armMasterID);
    armPID = new MotionMagicPID(this, Robot.bot.kArmP, Robot.bot.kArmI, Robot.bot.kArmD, Robot.bot.kArmF,
        Robot.bot.kArmCruiseVel, Robot.bot.kArmAcceleration);

    armPID.configPIDs();
    armMaster.config_IntegralZone(Robot.bot.gSlotIdx, 50);
    // }
  }

  private static Arm instance = new Arm();

  public static Arm getInstance() {

    return instance;
  }

  public double getEncoderCounts() {
    if (Robot.bot.hasArm()) {
      return armMaster.getSelectedSensorPosition(0);
    } else {
      return 0;
    }
  }

  public double getEncoderDeg() {
    if (Robot.bot.hasArm()) {
      return c2d(getEncoderCounts()) - Robot.bot.gArmAngularOffset;
    } else {
      return 0.0;
    }
  }

  public void resetEncoder() {
    if (Robot.bot.hasArm()) {
      armMaster.setSelectedSensorPosition(0);
      // armMaster.getSensorCollection().setPulseWidthPosition(0, 10);
    }
  }

  public double getMotorOutput() {
    if (Robot.bot.hasArm()) {
      return armMaster.getMotorOutputVoltage() / armMaster.getBusVoltage();
    } else {
      return 0;
    }
  }

  /**
   * Sets power to the arm motors
   *
   * @param power Desired percent output to assign to the arm motors
   */
  public void setMotorOutput(double power) {
    if (Robot.bot.hasArm()) {
      armMaster.set(ControlMode.PercentOutput, power);
    }
  }

  public double getEncoderVelocity() {
    if (Robot.bot.hasArm()) {
      return armMaster.getSelectedSensorVelocity(0);
    } else {
      return 0;
    }
  }

  public double getAngularVelocity() {
    return c2d(getEncoderVelocity()) * 10;
  }

  /**
   * Takes the derivative of the velocity of the sensor collection relative to
   * units per 100ms per second. Prefered to only use this data in enhanced sensor
   * collection in kinematics testing
   *
   * @return Acceleration in units per 100ms per second
   */
  public double getEncoderAcceleration() {
    currVel = getEncoderVelocity();
    accel = (currVel - lastVel) / .02;
    lastVel = currVel;
    return accel;
  }

  /**
   * Determines the highest achieved velocity so far
   *
   *
   * @return Highest recorded velocity in Units per 100ms.
   */
  public double getMaxVel() {
    double mCurrVel = getEncoderVelocity();
    maxVel = (mCurrVel > maxVel) ? mCurrVel : maxVel;
    return maxVel;
  }

  /**
   * Determines the highest achieved acceleration so far
   *
   *
   * @return Highest recorded acceleration in Units per 100ms per second.
   */
  public double getMaxAccel() {
    double mCurrAccel = getEncoderAcceleration();
    maxAccel = (mCurrAccel > maxAccel) ? mCurrAccel : maxAccel;
    return maxAccel;
  }

  /**
   * Determines the highest achieved deceleration so far
   *
   *
   * @return Highest recorded deceleration in Units per 100ms per second.
   */
  public double getMaxDecel() {
    double mCurrAccel = getEncoderAcceleration();
    minAccel = (mCurrAccel < minAccel) ? mCurrAccel : minAccel;
    return minAccel;
  }

  @Override
  public void setTargetPosition(double tgt) {
    double encTgt = d2c(tgt/* +Robot.bot.gArmAngularOffset */);
    // System.err.println("enc targ:" + encTgt);
    armMaster.set(ControlMode.MotionMagic, encTgt);
  }

  @Override
  public boolean getMagicStall() {
    return getMotorOutput() < 0.05;
  }

  @Override
  public double getMagicError() {
    if (Robot.bot.hasArm()) {
      return armMaster.getClosedLoopError(Robot.bot.gSlotIdx);
    } else {
      return 0;
    }
  }

  @Override
  public TalonSRX getTalon() {
    return armMaster;
  }

  /**
   * 
   * Enters a shorter velocity measurement period of 1ms compared to the default
   * 100ms. Used to measure better data collection especially acceleration.
   * 
   */
  private void enhancedVelMeas() {
    armMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
    armMaster.configVelocityMeasurementWindow(32);
  }

  /**
   * Returns the selected motor's output current
   * 
   * @return The motor output current in amps ()
   */
  public double getMotorCurrent() {
    if (Robot.bot.hasArm()) {
      return armMaster.getOutputCurrent();
    } else {
      return 0.0;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new SuperStructureCommand());
  }

  public boolean isEncoderFault() {
    return getEncoderDeg() > 0.0 && Extension.getInstance().getExtPosition() < 8.0;
  }

  private double d2c(double val) {
    return (val + Robot.bot.gArmAngularOffset) * Robot.bot.kEncoderUnitsPerRev / 360;
  }

  private double c2d(double val) {
    return (val / Robot.bot.kEncoderUnitsPerRev) * 360;
  }

  public void sendDashboardData() {
    table.putNumber("Arm Position", getEncoderDeg());
    table.putNumber("Arm Velocity", getAngularVelocity());
    table.putNumber("Arm Enc Position", getEncoderCounts());
    table.putNumber("Arm Enc Velocity", getEncoderVelocity());
    table.putNumber("Arm Enc Acceleration", getEncoderAcceleration());
    table.putNumber("Arm Enc Max Vel", getMaxVel());
    table.putNumber("Arm Enc Max Accel", getMaxAccel());
    table.putNumber("Arm Enc Max Decel", getMaxDecel());
    table.putNumber("Arm Percent Output", getMotorOutput());
    table.putBoolean("Arm Stall", getMagicStall());
    SmartDashboard.putNumber("Arm Error", getMagicError());
    // SmartDashboard.putNumber("Arm Current", getMotorCurrent());
    SmartDashboard.putNumber("Arm Position", getEncoderDeg());
    // SmartDashboard.putNumber("Arm Velocity", getAngularVelocity());
    // SmartDashboard.putNumber("Arm Enc Position", getEncoderCounts());
    if (Robot.bot.hasArm()) {
      table.putNumber("Arm Output Voltage", armMaster.getMotorOutputVoltage());
    } else {
      table.putNumber("Arm Output Voltage", 0);
    }
  }

  @Override
  public void outputTelemetry() {
    sendDashboardData();
  }

  @Override
  public void stop() {
    setMotorOutput(0);
  }
}
