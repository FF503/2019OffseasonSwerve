package com.team503.lib.util;

import com.team503.lib.util.FrogPIDF.ControlMode;
import com.team503.robot.RobotState;

public class SwerveHeadingController {
    private double targetHeading = 0;
    private FrogPIDF stabilizationPID, rotateInPlace, snappingPID;

    public SwerveHeadingController() {
        this.stabilizationPID = new FrogPIDF(0.005, 0.0, 0.0005, 0.0, ControlMode.Position_Control);
        this.rotateInPlace = new FrogPIDF(0.01, 0.0, 0.002, ControlMode.Position_Control);
        this.snappingPID = new FrogPIDF(0.005, 0.0, 0.0005, 0.0, ControlMode.Position_Control);
        snappingPID.setTolerance(3);
    }

    public enum State {
        Off, Stabilize, TemporaryDisable, Stationary, Snapping;
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
        stabilizationPID.setSetpoint(targetHeading);
        setState(State.Stabilize);
    }

    public void setStationaryTarget(double angle) {
        targetHeading = angle;
        rotateInPlace.setSetpoint(angle);
        setState(State.Stationary);
    }

    public void setSnapTarget(double angle){
        targetHeading = angle;
        setState(State.Snapping);
    }

    public void temporarilyDisable() {
        setState(State.TemporaryDisable);
    }

    public double getRotationalOutput() {
        switch (currentState) {
        case Off:
            return 0;
        case Stabilize:
            return stabilizationPID.calculateOutput(RobotState.getInstance().getCurrentTheta());
        case TemporaryDisable:
            return 0;
        case Stationary:
            return rotateInPlace.calculateOutput(RobotState.getInstance().getCurrentTheta());
        case Snapping:
            if (stabilizationPID.onTarget()){
                setState(State.Stabilize);
            }
            return snappingPID.calculateOutput(targetHeading);
        default:
            return 0;
        }
    }

    public void setRotationalSetpoint(double setPoint) {
        this.targetHeading = setPoint;
    }
}