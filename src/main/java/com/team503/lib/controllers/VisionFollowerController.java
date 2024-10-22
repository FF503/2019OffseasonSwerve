/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.controllers;

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

}
