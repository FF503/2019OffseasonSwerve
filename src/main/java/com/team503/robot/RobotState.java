package com.team503.robot;

import com.team254.lib.geometry.Pose2d;

public class RobotState {
    private Bot currentRobot;
    private Pose2d currentPose;
    private double currentTheta;
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public void setCurrentPose(Pose2d currentPose) {
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