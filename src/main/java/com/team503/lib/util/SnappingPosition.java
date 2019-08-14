/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.util;

public enum SnappingPosition {
    LOADING_STATION(180), FRONT_CARGOBAY(0), RIGHT_CARGOBAY(270), LEFT_CARGOBAY(90), LEFT_NEAR_ROCKET(-24),
    LEFT_FAR_ROCKET(-151), RIGHT_NEAR_ROCKET(24), RIGHT_FAR_ROCKET(151), FORWARD(0), BACKWARD(180), LEFT(270),
    RIGHT(90);

    private double angle;

    SnappingPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return this.angle;
    }
}
