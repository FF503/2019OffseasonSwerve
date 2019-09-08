
package com.team503.lib.controllers;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid
 * profile.
 */
public class PIDProfileController extends SendableBase {
  private PIDController m_controller;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.Constraints m_constraints;

  /**
   * @param Kp          The proportional coefficient.
   * @param Ki          The integral coefficient.
   * @param Kd          The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public PIDProfileController(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
    this(Kp, Ki, Kd, constraints, 0.02);
  }

  /**
   * @param Kp          The proportional coefficient.
   * @param Ki          The integral coefficient.
   * @param Kd          The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   * @param period      The period between controller updates in seconds. The
   *                    default is 0.02 seconds.
   */
  public PIDProfileController(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints,
      double period) {
    m_controller = new PIDController(Kp, Ki, Kd, period);
    m_constraints = constraints;
  }

  /**
   * 
   * @param Kp Proportional coefficient
   * @param Ki Integral coefficient
   * @param Kd Differential coefficient
   */

  public void setPID(double Kp, double Ki, double Kd) {
    m_controller.setPID(Kp, Ki, Kd);
  }

  /**
   * @param Kp proportional coefficient
   */
  public void setP(double Kp) {
    m_controller.setP(Kp);
  }

  /**
   * @param Ki integral coefficient
   */
  public void setI(double Ki) {
    m_controller.setI(Ki);
  }

  /**
   * @param Kd differential coefficient
   */
  public void setD(double Kd) {
    m_controller.setD(Kd);
  }

  /**
   * @return proportional coefficient
   */
  public double getP() {
    return m_controller.getP();
  }

  /**
   * @return integral coefficient
   */
  public double getI() {
    return m_controller.getI();
  }

  /**
   * @return differential coefficient
   */
  public double getD() {
    return m_controller.getD();
  }

  /**
   * @return The period of the controller.
   */
  public double getPeriod() {
    return m_controller.getPeriod();
  }

  public void setGoal(double goal) {
    m_goal = new TrapezoidProfile.State(goal, 0);
  }

  public double getGoal() {
    return m_goal.position;
  }

  public boolean atGoal() {
    return atSetpoint() && m_goal.equals(m_setpoint);
  }

  /**
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    m_constraints = constraints;
  }

  /**
   * @return The current setpoint.
   */
  public double getSetpoint() {
    return m_controller.getSetpoint();
  }

  /**
   * Returns true if the error is within the tolerance of the error.
   *
   * <p>
   * This will return false until at least one input value has been computed.
   */
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  /**
   * Enables continuous input.
   *
   * <p>
   * Rather then using the max and min input range as constraints, it considers
   * them to be the same point and automatically calculates the shortest route to
   * the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_controller.enableContinuousInput(minimumInput, maximumInput);
  }

  public void disableContinuousInput() {
    m_controller.disableContinuousInput();
  }

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * <p>
   * When the cap is reached, the integrator value is added to the controller
   * output rather than the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
  }

  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_controller.setTolerance(positionTolerance, velocityTolerance);
  }

  public double getPositionError() {
    return m_controller.getPositionError();
  }

  public double getVelocityError() {
    return m_controller.getVelocityError();
  }

  public double calculate(double measurement) {
    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(getPeriod());
    return m_controller.calculate(measurement, m_setpoint.position);
  }

  public double calculate(double measurement, double goal) {
    setGoal(goal);
    return calculate(measurement);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal        The new goal of the controller.
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public double calculate(double measurement, double goal, TrapezoidProfile.Constraints constraints) {
    setConstraints(constraints);
    return calculate(measurement, goal);
  }

  /**
   * Reset
   */
  public void reset() {
    m_controller.reset();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ProfiledPIDController");
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("goal", this::getGoal, this::setGoal);
  }
}