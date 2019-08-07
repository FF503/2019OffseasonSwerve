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
import com.team503.robot.OI;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.RobotState.ArmDirection;
import com.team503.robot.RobotState.GameElement;
import com.team503.robot.RobotState.SuperStructurePreset;
import com.team503.robot.RobotState.TargetHeight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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

  private static double aTgt, eTgt, wTgt, eLim = 0;
	private static boolean eIsMax, eIsMin = false;
	private static int manualIdx = 0;

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

    aTgt = Arm.getInstance().getEncoderDeg();
    eTgt = Robot.bot.gExtMinLim;
    wTgt = 90.0;
    RobotState.getInstance().setIsManual(false);
    RobotState.getInstance().setArmDirection(ArmDirection.FRONT);
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


  public boolean isEncoderFault() {
    return getEncoderDeg() > 0.0 && Extension.getInstance().getExtPosition() < 8.0;
  }

  private double d2c(double val) {
    return (val + Robot.bot.gArmAngularOffset) * Robot.bot.kEncoderUnitsPerRev / 360;
  }

  private double c2d(double val) {
    return (val / Robot.bot.kEncoderUnitsPerRev) * 360;
  }

  public static void updateSuperstruture() {
    if ((DriverStation.getInstance().isAutonomous() || !RobotState.getInstance().getIsManual())) {
      manualIdx = 0;
      if (RobotState.getInstance().getPositionChanged()) {
        ArmDirection armDirection = RobotState.getInstance().getArmDirection();
        TargetHeight tgtHeight = RobotState.getInstance().getTargetHeight();

        // ArmPosition armPos = RobotState.getInstance().getArmPosition();
        // ExtensionPosition extPos = RobotState.getInstance().getExtPosition();
        if (armDirection.equals(ArmDirection.FRONT)) {
          if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
            switch (tgtHeight) {
            case INTAKE:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_INTAKE);
              break;
            case LOW:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_LOW);
              break;
            case MIDDLE:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_MID);
              break;
            case HIGH:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_HIGH);
              break;
            case BUS:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_BUS);
              break;
            case HOME:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.CARGO_HOME);
              break;
            }

          } else {
            switch (tgtHeight) {
            case INTAKE:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_INTAKE);
              break;
            case LOW:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_LOW);
              break;
            case MIDDLE:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_MID);
              break;
            case HIGH:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_HATCH_HIGH);
              break;
            case BUS:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.FRONT_CARGO_BUS);
              break;
            case HOME:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
              break;
            }
          }
        } else if (armDirection.equals(ArmDirection.BACK)) {
          if (RobotState.getInstance().getGameElement() == GameElement.CARGO) {
            switch (tgtHeight) {
            case INTAKE:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_INTAKE);
              break;
            case LOW:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_LOW);
              break;
            case MIDDLE:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_MID);
              break;
            case HIGH:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_HIGH);
              break;
            case BUS:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_BUS);
              break;
            case HOME:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
              break;
            }
          } else {
            switch (tgtHeight) {
            case INTAKE:
              if (DriverStation.getInstance().isAutonomous()) {
                RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.VIEW_AUTO_REAR);
                break;
              }
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_INTAKE);
              break;
            case LOW:
              if (DriverStation.getInstance().isAutonomous()) {
                RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_LOW);
                break;
              }
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_LOW);
              break;
            case MIDDLE:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_MID);
              break;
            case HIGH:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_HATCH_HIGH);
              break;
            case BUS:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.BACK_CARGO_BUS);
              break;
            case HOME:
              RobotState.getInstance().setSuperStructurePreset(SuperStructurePreset.HATCH_HOME);
              break;
            }

          }
        }
        RobotState.getInstance().setPositionChanged(false);

        SuperStructurePreset preset = RobotState.getInstance().getSuperStructurePreset();
        wTgt = preset.getWristPosition();
        aTgt = preset.getArmPosition();
        eTgt = preset.getExtPosition();
      }
      if (Robot.bot.hasWrist()) {
        double wristPower = Wrist.getInstance().getTalon().getOutputCurrent()
            * Wrist.getInstance().getTalon().getMotorOutputVoltage();
        // System.out.println("Wrist Power Watts: " + wristPower);
        if (wristPower > Robot.bot.MAX_WRIST_POWER) {
          Wrist.getInstance().setMotorOutput(0.0);
          System.out.println("WRIST POWER TOO HIGH BURN OUT WARNING");
        } else {
          if (!(RobotState.getInstance().getSuperStructurePreset() == SuperStructurePreset.FRONT_CARGO_BUS
              && Arm.getInstance().getEncoderDeg() < -30)) {
            Wrist.getInstance().setTargetPosition(wTgt);
          }

        }
        // if (RobotState.getInstance().getGameElement() == GameElement.CARGO
        // && (Arm.getInstance().getEncoderDeg() > -37)) {
        // Wrist.getInstance().setTargetPosition(wTgt);
        // } else {
        // Wrist.getInstance().setTargetPosition(RobotState.SuperStructurePreset.FRONT_CARGO_INTAKE.getWristPosition());
        // }
        // }
      }
      if (Robot.bot.hasArm()) {
        Arm.getInstance().setTargetPosition(aTgt);
        if (Arm.getInstance().getEncoderDeg() > -40) {
          Extension.getInstance().setTargetPosition(eTgt);
        } else {
          Extension.getInstance().setTargetPosition(0.0);
        }
      }
    } else {
      if (manualIdx == 0) {
        // (new RumbleOperatorJoystick(1.0)).start();
        manualIdx++;
      }
      if (Robot.bot.hasArm()) {

        if (Arm.getInstance().getEncoderDeg() > 0.0 && Arm.getInstance().getEncoderDeg() < 180.0) {
          eLim = Robot.bot.gArmExtLength * (1 / Math.cos(Math.toRadians(Arm.getInstance().getEncoderDeg())) - 1);
          eLim = Math.abs(eLim);
        } else {
          eLim = 0.0;
        }

        aTgt = Arm.getInstance().getEncoderDeg();
        wTgt = Wrist.getInstance().getHRelEncoderDeg();
        eTgt = Extension.getInstance().getExtPosition();

        // eIsMax = Extension.getInstance().getExtPosition() > eLim;
        // eIsMin = Extension.getInstance().getExtPosition() < Robot.bot.gExtMinLim;
        Arm.getInstance().setMotorOutput(-OI.operatorJoystick.getY(Hand.kLeft));
        Wrist.getInstance().setMotorOutput(-OI.operatorJoystick.getY(Hand.kRight));

        // Extension.getInstance().setTargetPosition(eLim);

        if (OI.operatorJoystick.getPOV() == 0) {// && !eIsMax) {
          Extension.getInstance().setMotorOutput(0.50);
        } else if (OI.operatorJoystick.getPOV() == 180 /* && !eIsMin */) {
          Extension.getInstance().setMotorOutput(-0.50);
        } else {
          Extension.getInstance().setMotorOutput(0.00);
        }
      }
    }
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
  public void zeroSensors() {
    resetEncoder();
  }

  @Override
  public void outputTelemetry() {
    sendDashboardData();
  }

  @Override
  public void stop() {
    Arm.getInstance().setMotorOutput(0.0);
		Wrist.getInstance().setMotorOutput(0.0);
		Extension.getInstance().setMotorPower(0.0);
  }
}
