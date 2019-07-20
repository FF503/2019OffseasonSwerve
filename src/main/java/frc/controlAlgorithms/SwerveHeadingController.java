package frc.controlAlgorithms;

import frc.controlAlgorithms.FrogPIDF.ControlMode;
import frc.robot.RobotState;

public class SwerveHeadingController {
    private double targetHeading = 0;
    private FrogPIDF stabilizationPID;

    public SwerveHeadingController() {
        this.stabilizationPID = new FrogPIDF(0.005, 0.0, 0.0005, 0.0, ControlMode.Position_Control);
    }

    public enum State {
        Off, Stabilize;
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

    public double getRotationalOutput() {
        switch (currentState) {
        case Off:
            return 0;
        case Stabilize:
            return stabilizationPID.calculateOutput(RobotState.getInstance().getCurrentTheta());
        default:
            return 0;
        }
    }

    public void setRotationalSetpoint(double setPoint) {
        this.targetHeading = setPoint;
    }
}