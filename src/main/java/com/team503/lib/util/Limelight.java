/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.util;

import com.team503.robot.RobotState;
import com.team503.robot.subsystems.SwerveDrive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Subsystem for vision follower algorithm used in teleop
 */
public class Limelight {

    public Limelight() {
    }

    private static Limelight instance = new Limelight();

    public static Limelight getInstance() {
        return instance;
    }

    public double getTX() {
        return getTable().getEntry("tx").getDouble(0.0);
    }

    public double getTA() {
        return getTable().getEntry("ta").getDouble(0.0);
    }

    // Limelight network table
    public NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable("limelight");

    }

    /**
     * Sets the index of the limelight pipeline w/ networktables, use this to (not
     * robotstate) to actually change values
     * 
     * @param pipeline index of the desired pipeline
     */
    public void setPipeline(int pipeline) {
        getTable().getEntry("pipeline").setNumber(pipeline);
        RobotState.getInstance().setCurrentPipeline(pipeline);

    }

    public double[] calculateVisionOffset() {
        double tx = Limelight.getInstance().getTX();
        double ta = Limelight.getInstance().getTA();
        final double k = 1.0;
        double tDist = k / ta;

        double xOffset = Math.sin(Math.toRadians(tx)) * tDist;
        double yOffset = Math.cos(Math.toRadians(tx)) * tDist;

        return new double[] { xOffset, yOffset };
    }
}
