/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.auton.pure_pursuit;

import com.team503.robot.auton.FroggyAuton;

/**
 * Add your docs here.
 */
public class ForwardTest extends FroggyAuton{

    @Override
    protected void initAuton() {
        froggySequentialDrive("forwardTest");
    }

    @Override
    protected AutonStartingLocation getStartingLocation() {
        return AutonStartingLocation.Origin;
    }
}
