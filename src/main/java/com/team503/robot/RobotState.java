package com.team503.robot;

import com.team503.lib.util.Pose;

public class RobotState {
    private Bot currentRobot;
    private Pose currentPose;
    private double currentTheta;
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public void setCurrentPose(Pose currentPose) {
        this.currentPose = currentPose;
    }

    public synchronized double getCurrentTheta() {
        return currentTheta;
    }

    public synchronized void setCurrentTheta(double currentTheta) {
        this.currentTheta = currentTheta;
    }

    public void setCurrentRobot(final Bot currentRobot) {
        this.currentRobot = currentRobot;
    }

    public Bot getCurrentRobot() {
        return this.currentRobot;
    }
    
    public static enum Bot {
        Automatic, ProgrammingBot;
    }

    
}