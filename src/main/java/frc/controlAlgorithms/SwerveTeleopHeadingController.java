package frc.controlAlgorithms;

import frc.robot.RobotState;

public class SwerveTeleopHeadingController {
    private FrogPID pid;
    private double currentSetpoint = 0.0;
    public SwerveTeleopHeadingController(FrogPID pid){
        this.pid = pid;
        pid.setSetpoint(currentSetpoint);
    }
    public double getRotationalOutput(){
        return pid.calculateOutput(RobotState.getInstance().getCurrentTheta());
    }
    public void setRotationalSetpoint(double setPoint){
        this.currentSetpoint = setPoint;
    }
}