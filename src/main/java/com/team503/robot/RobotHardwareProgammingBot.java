/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

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
        wheelDiameter = 4.0;
        kTurnEncoderClicksperRevolution = 1024;
        requestDriveReversed = -1;

        // Swerve Module Positions (relative to the center of the drive base)
        // TODO: Update so it is actually correct
        kVehicleToBackLeft = new Translation2d(kWheelbaseLength / 2, -kWheelbaseWidth / 2);
        kVehicleToBackRight = new Translation2d(-kWheelbaseLength / 2, -kWheelbaseWidth / 2);
        kVehicleToFrontRight = new Translation2d(-kWheelbaseLength / 2, kWheelbaseWidth / 2);
        kVehicleToFrontLeft = new Translation2d(kWheelbaseLength / 2, kWheelbaseWidth / 2);

        kModulePositions = new Translation2d[] { kVehicleToFrontLeft, kVehicleToFrontRight, kVehicleToBackLeft,
                kVehicleToBackRight };

        gTimeoutMs = 0;
        gSlotIdx = 0;

        kEncoderUnitsPerRev = 4096;

        // Pure Pursuit
        lookaheadDistance = 20;
        kMaxVelocityInchesPerSec = 10;
        kV_PurePursuit = 1 / kMaxVelocityInchesPerSec;
        kA_PurePursuit = 0;
        kP_PurePursuit = 0;

        // Arm
        armMasterID = 11;
        armSlaveID = 10;
        kArmCruiseVel = 190;
        kArmAcceleration = 171;

        kArmF = 1023 / 285;
        kArmP = 3.0;
        kArmI = 0;
        kArmD = 10;

        gArmAngularOffset = 56.0;

        armMasterInverted = false;
        armSlaveInverted = false;
        armMasterSensorPhase = false;

        // Wrist/Intake
        rollerIntakeID = 2;

        intakePdpChannel = 8;
        vacuumPdpChannel = 7;

        intakePower = -0.8;
        intakeStallPower = -0.25;
        intakeOutPower = 0.7;// 0.8;
        intakeVaccPower = 0.45;

        hatchVacId = 1;
        releaseId = 0;

        rollerCurrentThres = 30.0;
        vacuumCurrentThres = 20.0;

        wristID = 13;

        kWristCruiseVel = 333.0;
        kWristAcceleration = 3330.0;

        kWristF = 1023 / 333.0;
        kWristP = 1.0;
        kWristI = 0;
        kWristD = 40;

        gWristMinLimit = -116.0;
        gWristMaxLimit = 93.0;

        gWristMaxLimitCargo = 93.0 - 15.;

        gWristAngularOffset = 135;
        gWristGroundOffset = 90.;

        wristMotorInverted = false;

        wristSensorPhase = true;
        MAX_WRIST_POWER = 149.0;

        // Extension
        extensionID = 12;

        extensionSensorPhase = true;
        extensionMotorInverted = false;

        kExtF = 1023 / 3148;
        kExtP = 0.40;
        kExtI = 0;
        kExtD = 0;

        kExtCruiseVel = 3148;
        kExtAcceleration = 31480;

        gExtGearRatio = 1.0;
        gExtSpoolDiameter = 1.353;
        gExtOffset = 0.; // extension starts 1.5 inches out to prevent grinding
        gExtMinLim = 0.;
        gExtMaxLim = 13.0;

        gArmExtLength = 24.75;

        // Power Distribution Panel
        PdpID = 0;
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

    // Gamespec

    @Override
    public boolean hasCompressor() {
        return true;
    }

    @Override
    public boolean hasArm() {
        return true;
    }

    @Override
    public boolean hasWrist() {
        return true;
    }

    @Override
    public boolean hasIntake() {
        return true;
    }

    @Override
    public boolean hasExtension() {
        return true;
    }

    @Override
    public boolean hasLimelight() {
        return true;
    }

}
