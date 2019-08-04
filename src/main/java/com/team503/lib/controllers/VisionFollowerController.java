/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.controllers;

import com.team254.lib.geometry.Translation2d;

/**
 * Add your docs here.
 */
public class VisionFollowerController {

    public VisionFollowerController() {
    }

    // find conversion using real-life tests
    private double distFromTarget(double currentTa) {
        return 0.25 / currentTa;
    }

    public Translation2d getVectorToTarget(double currentTa, double currentTx) {
        return new Translation2d(getXComponentArea(currentTa, currentTx), getYComponentArea(currentTa, currentTx));
    }

    // Area based
    private double getXComponentArea(double currentTa, double currentTx) {
        return distFromTarget(currentTa) * Math.sin(Math.toRadians(currentTx));
    }

    // Area based
    private double getYComponentArea(double currentTa, double currentTx) {
        return distFromTarget(currentTa) * Math.cos(Math.toRadians(currentTx));
    }

    // Tx based
    private double getXComponent(double currentTx, double d1, double d2) {
        return d2 - d1;
    }

    // Tx based
    private double getYComponent(double currentTx, double d1, double d2) {
        return (d2 - d1) / Math.tan(currentTx);
    }

    public enum VisionMode {
        Linear;
    }

}