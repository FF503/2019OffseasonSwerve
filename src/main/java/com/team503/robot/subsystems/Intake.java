/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.team503.lib.util.FFDashboard;
import com.team503.robot.Robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for the -duawl oper- ating intake.
 */
public class Intake extends Subsystem {

  private Spark hatchVac, cargoIntake;
  private Solenoid releaser;
  private FFDashboard table = new FFDashboard("Intake");
  private double cCur, lCur = 0.0;

  private PowerDistributionPanel pdp;
  private static Intake instance = new Intake();

  public Intake() {
    if (Robot.bot.hasIntake()) {
      hatchVac = new Spark(Robot.bot.hatchVacId);
      cargoIntake = new Spark(Robot.bot.rollerIntakeID);
      pdp = new PowerDistributionPanel(Robot.bot.PdpID);
      releaser = new Solenoid(Robot.bot.releaseId);
    }
  }

  public static Intake getInstance() {
    return instance;
  }

  public void setMotorPower(double power) {
    cargoIntake.set(power);
  }

  public double getOutputCurrent() {
    if (Robot.bot.hasIntake()) {
      return pdp.getCurrent(Robot.bot.intakePdpChannel);
    } else {
      return 0.0;
    }
  }

  public double getVacuumCurrent() {
    return pdp.getCurrent(Robot.bot.vacuumPdpChannel);
  }

  public double getChannelCurrent(int pdpChannel) {
    return pdp.getCurrent(pdpChannel);
  }

  public double getDeltaCurrent() {
    cCur = getOutputCurrent();
    double delta = cCur - lCur;
    lCur = cCur;
    return delta;
  }

  public boolean hasCargo() {
    return getOutputCurrent() > Robot.bot.rollerCurrentThres;
  }

  public boolean hasHatch() {
    return getVacuumCurrent() > Robot.bot.vacuumCurrentThres;
  }

  public double getMotorPower() {
    if (Robot.bot.hasIntake()) {
      return hatchVac.get();
    } else {
      return 0.0;
    }
  }

  public boolean isCargo() {
    return (getMotorPower() > 0.01);
  }

  public boolean isHatch() {
    return (getMotorPower() < 0.01);
  }

  public boolean getIntakeRunning() {
    return Math.abs(getMotorPower()) > 0.0;
  }

  public void intakeCargo() {
    setMotorPower(Robot.bot.intakePower);
  }

  public void intakeHatch() {
    setMotorPower(-1.0);
  }

  public void outtakeCargo() {
    setMotorPower(Robot.bot.intakeOutPower);
  }

  public void outtakeHatch() {
    setMotorPower(1.0);
  }

  public void stopIntake() {
    setMotorPower(Robot.bot.intakeStallPower);
  }

  public void startVacuum() {
    hatchVac.set(Robot.bot.intakeVaccPower);
  }

  public void setVacuumPower(double power) {
    hatchVac.set(power);
  }

  public void stopVacuum() {
    hatchVac.set(0.0);
  }

  public void releaseHatch() {
    releaser.set(true);
  }

  public void closeReleaseValve() {
    releaser.set(false);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putBoolean("Intaking running", getIntakeRunning());
    SmartDashboard.putBoolean("Has Hatch", hasHatch());
    SmartDashboard.putBoolean("Has Cargo", hasCargo());
    SmartDashboard.putNumber("Motor Output Power", getMotorPower());
  }

  @Override
  public void stop() {
    stopVacuum();
    stopIntake();
  }
}
