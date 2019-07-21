package com.team503.robot;

public class RobotState {
    private double currentTheta, curX, curY;
    private static  RobotState instance = new RobotState();
    public static RobotState getInstance(){
        return instance;
    }
    public double getCurrentTheta() {
        return currentTheta;
    }

    public void setCurrentTheta(double currentTheta) {
        this.currentTheta = currentTheta;
    }

    public double getCurX() {
        return curX;
    }

    public void setCurX(double curX) {
        this.curX = curX;
    }

    public double getCurY() {
        return curY;
    }

    public void setCurY(double curY) {
        this.curY = curY;
    }

}