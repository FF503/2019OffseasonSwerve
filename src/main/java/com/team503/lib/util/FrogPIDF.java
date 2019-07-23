package com.team503.lib.util;

import edu.wpi.first.wpilibj.Timer;

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

    public void setSetpoint(double setPoint) {
        this.setPoint = setPoint;
        this.integral = 0;
    }

    public double calculateOutput(double sensorState) {
        this.state = sensorState;
        double error = setPoint - sensorState;
        double dError = error - lastError;
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        double derivative = dError / dt;
        integral += error * dt;
        double pOut = p * error;
        double iOut = i * integral;
        double dOut = d * derivative;
        double fOut = f * setPoint;
        
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

}