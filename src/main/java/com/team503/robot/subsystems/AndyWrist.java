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
import com.team503.robot.RobotState;
import com.team503.robot.RobotState.GameElement;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for wrist motion magic trajectories
 */
public class AndyWrist extends Subsystem implements SuperStructureSystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX wristMotor;
  private FFDashboard table = new FFDashboard("Wrist");
  private MotionMagicPID wristPID;
  private double lastVel, currVel, accel, maxVel, maxAccel, minAccel = 0;

  public AndyWrist() {
    if (Robot.bot.hasWrist()) {
      wristMotor = new TalonSRX(Robot.bot.wristID);
      wristMotor.configSelectedFeedbackSensor(/*FeedbackDevice.CTRE_MagEncoder_Relative*/ FeedbackDevice.CTRE_MagEncoder_Absolute , Robot.bot.gSlotIdx,
          Robot.bot.gTimeoutMs);
      wristMotor.setInverted(Robot.bot.wristMotorInverted);
      wristMotor.setSensorPhase(Robot.bot.wristSensorPhase);
      wristMotor.setNeutralMode(NeutralMode.Brake);

      // resetEncoder();

      wristMotor.selectProfileSlot(0, 0);
      wristPID = new MotionMagicPID(this, Robot.bot.kWristP, Robot.bot.kWristI, Robot.bot.kWristD, Robot.bot.kWristF,
          (int) Robot.bot.kWristCruiseVel, (int) Robot.bot.kWristAcceleration);
      wristPID.configPIDs();
      wristMotor.config_IntegralZone(Robot.bot.gSlotIdx, 50);
      //wristMotor.configPeakCurrentLimit(amps)
    }
  }

  private static AndyWrist instance = new AndyWrist();

  public static AndyWrist getInstance() {
    return instance;
  }

  public double getEncoderCounts() {
    if(Robot.bot.hasWrist()) {
      return wristMotor.getSelectedSensorPosition(0);
    } else {
      return 0;
    }
  }

  public double getEncoderDeg() {
    return getEncoderCounts() / 4096.0 * 360.0 + Robot.bot.gWristGroundOffset;// * 1.3;
  }

  public double getHRelEncoderDeg() {
    return d2h(getEncoderDeg());
  }

  public double getMotorOutput() {
    if(Robot.bot.hasWrist()) {
      return wristMotor.getMotorOutputVoltage() / wristMotor.getBusVoltage();
    } else {
      return 0;
    }  
  }

  /**
   * Sets power to the wrist motor
   *
   * @param power Desired percent output to assign to the wrist motor
   */
  public void setMotorOutput(double power) {
    if(Robot.bot.hasWrist()) {
      wristMotor.set(ControlMode.PercentOutput, power);
    }
  }

  public double getAngularVelocity() {
    return c2d(getEncoderVelocity()) * 10;
  }

  public double getEncoderVelocity() {
    if(Robot.bot.hasWrist()) {
      return wristMotor.getSelectedSensorVelocity(0);
    } else {
      return 0;
    }
  }

  public void resetEncoder() {
    if (Robot.bot.hasWrist()){
    wristMotor.setSelectedSensorPosition(0);
    }
  //  wristMotor.setSelectedSensorPosition(0);
    // if (getEncoderDeg()<0){
    // wristMotor.getSensorCollection().setPulseWidthPosition((int)d2c(-90), 10);
    // }
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
    double mCurrAccel = Math.abs(getEncoderAcceleration());
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
  public void setTargetPosition(double hRtgt) {

    table.putNumber("Wrist Target", hRtgt);

    // if (Arm.getInstance().getEncoderDeg() > 90 && hRtgt < 180.0) {
    //   hRtgt += 180.0;
    // } else if (Arm.getInstance().getEncoderDeg() < 90 && hRtgt > 180.0) {
    //   hRtgt -= 180.0;
    // }

    double maxLim;
    if(RobotState.getInstance().getGameElement() == GameElement.CARGO) {
      maxLim = Robot.bot.gWristMaxLimitCargo;
    } else {
      maxLim = Robot.bot.gWristMaxLimit;
    }

    double encTgt = d2c(minLim(maxLim(h2d(hRtgt), maxLim), Robot.bot.gWristMinLimit));
    wristMotor.set(ControlMode.MotionMagic, encTgt);
  }

  @Override
  public boolean getMagicStall() {
    return getMotorOutput() < 0.05;
  }

  @Override
  public double getMagicError() {
    if(Robot.bot.hasWrist()) {
      return wristMotor.getClosedLoopError(Robot.bot.gSlotIdx);
    } else { 
      return 0.0;
    }
  }

  @Override
  public TalonSRX getTalon() {
    return wristMotor;
  }

  private double d2c(double val) {
    return (val - Robot.bot.gWristGroundOffset) * Robot.bot.kEncoderUnitsPerRev / 360;
  }

  private double c2d(double val) {
    return (val / Robot.bot.kEncoderUnitsPerRev) * 360;
  }

  private double d2h(double val) {
    return 180 - Robot.bot.gWristAngularOffset + val + AndyArm.getInstance().getEncoderDeg();
  }

  private double h2d(double val) {
    return val - 180 + Robot.bot.gWristAngularOffset - AndyArm.getInstance().getEncoderDeg();
  }

  public double maxLim(double value, double limit) {
    return value > limit ? limit : value;
  }

  public double minLim(double value, double limit) {
    return value < limit ? limit : value;
  }


  /**
   * Returns the selected motor's output current
   * 
   * @return The motor output current in amps ()
   */
  public double getMotorCurrent() {
    if(Robot.bot.hasWrist()) {
      return wristMotor.getOutputCurrent();
    } else {
      return 0.0;
    }
  }

  public void sendDashboardData() {
    table.putNumber("Wrist Position", getEncoderDeg());
    table.putNumber("Wrist H Rel", getHRelEncoderDeg());
    table.putNumber("Wrist Velocity", getAngularVelocity());
    table.putNumber("Wrist Enc Position", getEncoderCounts());
    table.putNumber("Wrist Enc Velocity", getEncoderVelocity());
    table.putNumber("Wrist Enc Acceleration", getEncoderAcceleration());
    table.putNumber("Wrist Enc Max Vel", getMaxVel());
    table.putNumber("Wrist Enc Max Accel", getMaxAccel());
    table.putNumber("Wrist Enc Max Decel", getMaxDecel());
    table.putNumber("Wrist Percent Output", getMotorOutput());
    table.putBoolean("Wrist Stall", getMagicStall());
    if(Robot.bot.hasWrist()) {
      table.putNumber("Wrist Power", wristMotor.getMotorOutputVoltage()*getMotorCurrent());
    } else {  
      table.putNumber("Wrist Power", 0.0);
    }
    // SmartDashboard.putNumber("Wrist Error", getMagicError());
    // SmartDashboard.putNumber("Wrist Current", getMotorCurrent());
    SmartDashboard.putNumber("Wrist Position", getEncoderDeg());
    // SmartDashboard.putNumber("Wrist H Rel", getHRelEncoderDeg());
    // SmartDashboard.putNumber("Wrist Velocity", getAngularVelocity());
    if(Robot.bot.hasWrist()) {
       table.putNumber("Wrist Output Voltage", wristMotor.getMotorOutputVoltage());
    } else {
       table.putNumber("Wrist Output Voltage", 0.0);
    }   
  }

  @Override
  public void outputTelemetry() {
    sendDashboardData();
  }

  @Override
  public void stop() {

  }

  @Override
  public void zeroSensors() {
    resetEncoder();
  }

}
