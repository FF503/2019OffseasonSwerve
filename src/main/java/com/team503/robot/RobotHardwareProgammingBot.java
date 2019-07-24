/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Translation2d;

public class RobotHardwareProgammingBot extends RobotHardware {
    /* All distance measurements are in inches, unless otherwise noted */
    // Swerve Module JSON file names
    private class SwerveFileNames {
        public static final String backLeft = "BackLeftAndyA";
        public static final String backRight = "BackRightAndyA";
        public static final String frontLeft = "FrontLeftAndyA";
        public static final String frontRight = "FrontRightAndyA";
    }

    @Override
    public void initalizeConstants() {
        // Swerve Calculations Constants (measurements are in inches)
        kWheelbaseLength = 21.0;
        kWheelbaseWidth = 21.0;

        // Swerve Module Positions (relative to the center of the drive base)
        kVehicleToModuleZero = new Translation2d(-kWheelbaseLength / 2, kWheelbaseWidth / 2);
        kVehicleToModuleOne = new Translation2d(-kWheelbaseLength / 2, -kWheelbaseWidth / 2);
        kVehicleToModuleTwo = new Translation2d(kWheelbaseLength / 2, -kWheelbaseWidth / 2);
        kVehicleToModuleThree = new Translation2d(kWheelbaseLength / 2, kWheelbaseWidth / 2);

        kModulePositions = Arrays.asList(kVehicleToModuleZero, kVehicleToModuleOne, kVehicleToModuleTwo,
                kVehicleToModuleThree);

    }

    @Override
    public List<Translation2d> getModulePositions() {
        return kModulePositions;
    }

    @Override
    public String getBackLeftName() {
        return SwerveFileNames.backLeft;
    }

    @Override
    public String getBackRightName() {
        return SwerveFileNames.backRight;
    }

    @Override
    public String getFrontLeftName() {
        return SwerveFileNames.frontLeft;
    }

    @Override
    public String getFrontRightName() {
        return SwerveFileNames.frontRight;
    }
}
