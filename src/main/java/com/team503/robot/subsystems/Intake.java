/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.subsystems;


import com.team503.lib.util.FFDashboard;
import com.team503.robot.Robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for the -duawl oper- ating intake.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Spark hatchVac, cargoIntake;
  private Solenoid releaser;
  private FFDashboard table = new FFDashboard("Intake");
  private double cCur, lCur = 0.0;
  // private DigitalInput beam;

  private PowerDistributionPanel pdp;
  private static Intake instance = new Intake();

  public Intake() {
    if (Robot.bot.hasIntake()) {
      hatchVac = new Spark(Robot.bot.hatchVacId);
      cargoIntake = new Spark(Robot.bot.rollerIntakeID);
      // beam = new DigitalInput(Robot.bot.beamBreakID);
      pdp = new PowerDistributionPanel(Robot.bot.PdpID);
      releaser = new Solenoid(Robot.bot.releaseId);
      // startVacuum();
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
    System.out.println("raw stop");
    setMotorPower(Robot.bot.intakeStallPower);
  }

  // public void grabHatch() {
  // RobotState.getInstance().setGrabberDeployed(true);
  // }

  // public void setVacuumOutput(boolean state) {
  // hatchVac.set(state ? 1.0 : 0.0);
  // }

  // public void release(boolean state) {
  // releaser.set(state);
  // }

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

  public void sendDashboardData() {
    table.putBoolean("Intaking running", getIntakeRunning());
    table.putBoolean("Has Hatch", hasHatch());
    table.putBoolean("Has Cargo", hasCargo());
    table.putNumber("Motor Output Power", getMotorPower());
    // table.putNumber("Intake Motor Current", getOutputCurrent());
    // table.putNumber("Vacuum Motor Current", getVacuumCurrent());

    // SmartDashboard.putNumber("Intake Motor Current", getOutputCurrent());
    // SmartDashboard.putNumber("Vacuum Motor Current", getVacuumCurrent());

    for (int i=0; i<=15; i++) {
      table.putNumber("Motor "+i+" Current", getChannelCurrent(i));
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
