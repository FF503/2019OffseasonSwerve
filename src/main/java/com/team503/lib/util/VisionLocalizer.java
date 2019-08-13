/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.util;

import com.team503.robot.RobotState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class VisionLocalizer {

    public VisionLocalizer() {

    }

    private static VisionLocalizer instance = new VisionLocalizer();

    public static VisionLocalizer getInstance() {
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
}
