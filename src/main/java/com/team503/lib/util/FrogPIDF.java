package com.team503.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class FrogPIDF {
    private final double p, i, d, f;
    private double setPoint;
    private double lastTime, lastError;
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

    public void setSetpoint(double setPoint) {
        this.setPoint = setPoint;
        this.lastTime = Timer.getFPGATimestamp();
        this.lastError = 0;
        this.integral = 0;
    }

    public double calculateOutput(double sensorState) {
        double error = setPoint - sensorState;
        double dError = error - lastError;
        double dt = Timer.getFPGATimestamp() - lastTime;
        double derivative = dError / dt;
        integral += error * dt;
        double pOut = p * error;
        double iOut = i * integral;
        double dOut = d * derivative;
        double fOut = f * setPoint;
        return Math.max(-1, Math.min(pOut + iOut + dOut + fOut, 1));
    }

}