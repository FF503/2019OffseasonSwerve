package com.team503.lib.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FrogPIDF {
    private final double p, i, d, f;
    private double state;
    private double tolerance;
    private double setPoint;
    private double lastTime, lastError = 0;
    private double integral;
    private ControlMode control;

    public enum ControlMode {
        Velocity_Control, Position_Control;
    }

    public FrogPIDF(double p, double i, double d, ControlMode controlMode) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.control = controlMode;
        this.f = 0.0;
    }

    public FrogPIDF(double p, double i, double d, double f, ControlMode controlMode) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.control = controlMode;
        this.f = (controlMode == ControlMode.Velocity_Control) ? f : 0.0;
    }

    public FrogPIDF(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;

        this.control = (f != 0.0) ? ControlMode.Velocity_Control : ControlMode.Position_Control;

    }

    public void setSetpoint(double setPoint) {
        this.setPoint = setPoint;
        this.integral = 0;
        this.lastTime = Timer.getFPGATimestamp();
    }

    public double calculateOutput(double sensorState, boolean boundTo180) {
        this.state = sensorState;
        double error = boundTo180 ? boundHalfDegrees(setPoint - sensorState) : (setPoint - sensorState);
        double dError = error - lastError;
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        double derivative = dError / dt;
        integral += error * dt;
        double pOut = p * error;
        double iOut = i * integral;
        double dOut = d * derivative;
        double fOut = f * setPoint;

        SmartDashboard.putNumber("FrogPIDF Error", error);

        lastTime = time;
        lastError = error;

        return Math.max(-1, Math.min(pOut + iOut + dOut + fOut, 1));
    }

    public void setTolerance(double t) {
        this.tolerance = t;
    }

    public boolean onTarget() {
        return Math.abs(state - setPoint) < tolerance;
    }

    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0)
            angle_degrees -= 360.0;
        while (angle_degrees < -180.0)
            angle_degrees += 360.0;
        return angle_degrees;
    }

}