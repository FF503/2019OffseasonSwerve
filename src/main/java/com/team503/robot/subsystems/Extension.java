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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team503.lib.util.FFDashboard;
import com.team503.lib.util.MotionMagicPID;
import com.team503.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for extension motion magic trajectories
 */
public class Extension extends Subsystem implements SuperStructureSystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX extMotor;
  private MotionMagicPID extPID;
  private double lastVel, currVel, accel, maxVel, maxAccel = 0;
  private boolean runExtension = true;
  private FFDashboard table = new FFDashboard("Extension");

  public Extension() {
    if (Robot.bot.hasExtension()) {
      extMotor = new TalonSRX(Robot.bot.extensionID);
      extMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative/* CTRE_MagEncoder_Absolute */, Robot.bot.gSlotIdx,
          Robot.bot.gTimeoutMs);
      extMotor.setInverted(Robot.bot.extensionMotorInverted);
      extMotor.setSensorPhase(Robot.bot.extensionSensorPhase);
      extMotor.setNeutralMode(NeutralMode.Brake);

      // resetEncoder();

      extMotor.selectProfileSlot(0, 0);
      extPID = new MotionMagicPID(this, Robot.bot.kExtP, Robot.bot.kExtI, Robot.bot.kExtD, Robot.bot.kExtF,
          (int) Robot.bot.kExtCruiseVel, (int) Robot.bot.kExtAcceleration);
      extPID.configPIDs();
      // extensionMotor.config_IntegralZone(Robot.bot.gSlotIdx, 50);
    }

  }

  private static Extension instance = new Extension();

  public static Extension getInstance() {
    return instance;
  }

  public void setMotorPower(double power) {
    if(Robot.bot.hasArm()) {
      extMotor.set(ControlMode.PercentOutput, power);
    }
  }

  public double getEncoderCounts() {
    if(Robot.bot.hasArm()) {
      return extMotor.getSelectedSensorPosition(0);
    } else {
      return 0.0;
    }
  }

  public double getExtPosition() {
    return c2i(getEncoderCounts()) + Robot.bot.gExtOffset;
  }

  public double getMotorOutput() {
    if(Robot.bot.hasArm()) {
      return extMotor.getMotorOutputVoltage() / extMotor.getBusVoltage();
    } else {
      return 0.0;
    }
  }

  /**
   * Sets power to the extension motor
   *
   * @param power Desired percent output to assign to the extension motors
   */
  public void setMotorOutput(double power) {
    extMotor.set(ControlMode.PercentOutput, power);
  }

  public double getEncoderVelocity() {
    if(Robot.bot.hasArm()) {
      return extMotor.getSelectedSensorVelocity(0);
    } else {
      return 0.0;
    }
  }

  public double getEncoderLinearVelocity() {
    return c2i(getEncoderVelocity()) * 10;
  }

  public void resetEncoder() {
    extMotor.setSelectedSensorPosition(0);
  }

  /**
   * Sets the extension control mode
   * 
   * @param applyControl Desired control mode: 0 for inactive, 1 for active
   */
  @Override
  public void setTargetPosition(double tgt) {
    table.putNumber("Extension Target", tgt);
    extMotor.set(ControlMode.MotionMagic, i2c(minLim(maxLim(tgt, Robot.bot.gExtMaxLim), Robot.bot.gExtMinLim)));

  }

  // extMotor.set(ControlMode.MotionMagic, i2c(applyControl));
  // System.out.println("target:"+i2c(minLim(maxLim((Robot.bot.gArmExtLength /
  // Math.cos(Math.toRadians(Arm.getInstance().getEncoderDeg()))
  // - Robot.bot.gArmExtLength), Robot.bot.gExtMaxLim), Robot.bot.gExtMaxLim)));

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
  // public double getMaxDecel() {
  //   double mCurrAccel = getEncoderAcceleration();
  //   minAccel = (mCurrAccel < minAccel) ? mCurrAccel : minAccel;
  //   return minAccel;
  // }

  @Override
  public boolean getMagicStall() {
    if(Robot.bot.hasArm()) {
      return (extMotor.getMotorOutputVoltage() / extMotor.getBusVoltage() < 0.05);
    } else {
      return true; 
    }
  
  }

  @Override
  public TalonSRX getTalon() {
    return extMotor;
  }

  @Override
  public double getMagicError() {
    if(Robot.bot.hasArm()) {
      return extMotor.getClosedLoopError();
    } else {
      return 0.0;
    }
  
  }

  /**
   * Returns the selected motor's output current
   * 
   * @return The motor output current in amps ()
   */
  public double getMotorCurrent() {
    if(Robot.bot.hasArm()) {
      return extMotor.getOutputCurrent();
    } else {
      return 0.0;
  }
  
  }

  public double c2i(double counts) {
    return counts / Robot.bot.kEncoderUnitsPerRev * Robot.bot.gExtGearRatio * Math.PI * Robot.bot.gExtSpoolDiameter;          
  }

  public double i2c(double inches) {
    return (inches + Robot.bot.gExtOffset) * Robot.bot.kEncoderUnitsPerRev
        / (Robot.bot.gExtGearRatio * Math.PI * Robot.bot.gExtSpoolDiameter);
  }

  public double maxLim(double value, double limit) {
    return value > limit ? limit : value;
  }

  public double minLim(double value, double limit) {
    return value < limit ? limit : value;
  }

  public void sendDashboardData() {
    table.putNumber("Ext Position", getExtPosition());
    table.putNumber("Extension Velocity", getEncoderLinearVelocity());
    table.putNumber("Extension Enc Position", getEncoderCounts());
    table.putNumber("Extension Enc Velocity", getEncoderVelocity());
    table.putNumber("Extension Enc Acceleration", getEncoderAcceleration());
    table.putNumber("Extension Enc Max Vel", getMaxVel());
    table.putNumber("Extension Enc Max Accel", getMaxAccel());
    table.putNumber("Extension Percent Output", getMotorOutput());
    table.putBoolean("Extension Stall", getMagicStall());
    table.putNumber("Extension Error", getMagicError());
    SmartDashboard.putNumber("Extension Current", getMotorCurrent());
    SmartDashboard.putNumber("Ext Position", getExtPosition());
    if(Robot.bot.hasArm()) {
      table.putNumber("Extension Output Voltage", extMotor.getMotorOutputVoltage());
    }else {
      table.putNumber("Extension Output Voltage", 0.0);
    }
  }

  @Override
  protected void initDefaultCommand() {
  }

}
