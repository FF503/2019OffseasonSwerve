package com.team503.lib.util;

import com.team503.lib.util.FrogPIDF.ControlMode;
import com.team503.robot.RobotState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveHeadingController {
    private double targetHeading;
    private double disabledTimestamp;
    private final double disableTimeLength = 0.2;
    private FrogPIDF stabilizationPID, rotateInPlace, snappingPID;

    public SwerveHeadingController() {
        this.stabilizationPID = new FrogPIDF(0.005, 0.0, 0.0005, 0.0, ControlMode.Position_Control);
        this.rotateInPlace = new FrogPIDF(0.004, 0.0, 0.0001, ControlMode.Position_Control);
        this.snappingPID = new FrogPIDF(0.015, 0.0, 0.000, 0.0, ControlMode.Position_Control);
        snappingPID.setTolerance(3);
    }

    public enum State {
        Off, Stabilize, TemporaryDisable, Stationary, Snap;
    }

    private State currentState = State.Off;

    public State getState() {
        return currentState;
    }

    private void setState(State newState) {
        currentState = newState;
    }

    public void setStabilizationTarget(double angle) {
        targetHeading = angle;
        stabilizationPID.setSetpoint(angle);
        setState(State.Stabilize);
    }

    public void setStationaryTarget(double angle) {
        targetHeading = angle;
        rotateInPlace.setSetpoint(angle);
        setState(State.Stationary);
    }

    public void setSnapTarget(double angle) {
        targetHeading = angle;
        snappingPID.setSetpoint(angle);
        setState(State.Snap);
    }

    public void temporarilyDisable() {
        setState(State.TemporaryDisable);
        disabledTimestamp = Timer.getFPGATimestamp();
    }

    public void disable() {
        setState(State.Off);
    }

    public double getRotationalOutput() {
        SmartDashboard.putNumber("Target Heading", targetHeading);
        SmartDashboard.putString("State", currentState.toString());
        double output = 0;
        double heading = RobotState.getInstance().getCurrentTheta();

        switch (currentState) {
        case Off:
            break;
        case Stabilize:
            output = stabilizationPID.calculateOutput(heading, true);
            break;
        case TemporaryDisable:
            targetHeading = heading;
            if (Timer.getFPGATimestamp() - disabledTimestamp >= disableTimeLength) {
                setState(State.Stabilize);
            }
            break;
        case Stationary:
            output = rotateInPlace.calculateOutput(heading, true);
            break;
        case Snap:
            if (snappingPID.onTarget()) {
                setState(State.Stabilize);
                break;
            }
            output = snappingPID.calculateOutput(heading, true);
            break;
        }

        return output;
    }
}