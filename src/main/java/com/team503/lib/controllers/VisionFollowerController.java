/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.controllers;

/**
 * Add your docs here.
 */
public class VisionFollowerController {

    public VisionFollowerController() {

    }

    //find conversion using real-life tests
    private double distFromTarget(double currentTa){
        return 0;
    }

    //Area based
    public double getXComponentArea(double currentTa, double currentTx) {
        return distFromTarget(currentTa)*Math.sin(currentTx*Math.PI/180);
    }
    //Area based
    public double getYComponentArea(double currentTa, double currentTx) {
        return distFromTarget(currentTa)*Math.cos(currentTx*Math.PI/180);
    }

    //Tx based
    public double getXComponent(double currentTx, double d1, double d2) {
        return d2-d1;
    }
    //Tx based
    public double getYComponent(double currentTx, double d1, double d2) {
        return (d2-d1)/Math.tan(currentTx);
    }


    public enum VisionMode {
        Linear;
    }

}
