/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.controllers;

import java.util.ArrayList;

import com.team503.lib.geometry.Pose;
import com.team503.lib.geometry.Translation2d;
import com.team503.lib.util.FFDashboard;
import com.team503.robot.Robot;

/**
 * Add your docs here.
 */
public class VisionFollowerController {
    private FFDashboard table = new FFDashboard("Vision");

    public VisionFollowerController() {
    }

    // measure distance from centor of robot to limelight camera
    private final double offsetX = 0, offsetY = 0;

    // find conversion using real-life tests
    private double distFromTarget(double currentTa) {
        return 1.0 / currentTa;
    }

    public Translation2d getVectorToTarget(double currentTa, double currentTx) {
        Translation2d vector = new Translation2d(getXComponentArea(currentTa, currentTx),
                getYComponentArea(currentTa, currentTx));
        table.putNumber("currentTA", currentTa);
        table.putNumber("currentTX", currentTx);
        table.putNumber("x", vector.getX());
        table.putNumber("y", vector.getY());
        return vector;
    }

    // Area based
    private double getXComponentArea(double currentTa, double currentTx) {
        return Robot.bot.xVisionkP * distFromTarget(currentTa) * Math.sin(Math.toRadians(currentTx)) + offsetX;
    }

    // Area based
    private double getYComponentArea(double currentTa, double currentTx) {
        return Robot.bot.yVisionkP * distFromTarget(currentTa) * Math.cos(Math.toRadians(currentTx)) + offsetY;
    }

    // Tx based
    @Deprecated
    private double getXComponent(double currentTx, double d1, double d2) {
        return d2 - d1;
    }

    // Tx based
    @Deprecated
    private double getYComponent(double currentTx, double d1, double d2) {
        return (d2 - d1) / Math.tan(currentTx);
    }

    public enum VisionMode {
        Linear;
    }

    public ArrayList<VisionTarget> possibleVisibleTargets(Pose robotPose, double toleranceRadius) {
        ArrayList<VisionTarget> targets = new ArrayList<VisionTarget>();

        for (VisionTarget target : VisionTarget.values()) {
            double x = target.getX();
            double y = target.getY();
            double theta = target.getTheta();
            if (Math.abs(robotPose.getTheta() - theta) <= Math.PI / 3
                    && (Math.pow((x - robotPose.getX()), 2) + Math.pow((y - robotPose.getY()), 2)) <= Math
                            .pow(toleranceRadius, 2)) {
                targets.add(target);
            }
        }

        return targets;
    }

}

enum VisionTarget {
    LEFT_CARGO_BAY(0, 0, 0), RIGHT_CARGO_BAY(0, 0, 0);

    private double theta;
    private Translation2d translation;

    VisionTarget(double x, double y, double theta) {
        this.translation = new Translation2d(x, y);
        this.theta = theta;
    }

    double getX() {
        return this.translation.getX();
    }

    double getY() {
        return this.translation.getY();
    }

    double getTheta() {
        return this.theta;
    }

}
