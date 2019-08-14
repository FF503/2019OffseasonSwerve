/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.List;

import com.team254.lib.geometry.Translation2d;
import com.team503.lib.util.Util;
import com.team503.robot.RobotState.Bot;

/**
 * Add your docs here.
 */
public abstract class RobotHardware {

    public final double POSE_LOOP_DT = 0.01;

    public abstract void initalizeConstants();

    // Constants

    // SwerveFileNames
    public abstract String getBackLeftName();

    public abstract String getBackRightName();

    public abstract String getFrontLeftName();

    public abstract String getFrontRightName();

    // Gamespec
    public abstract boolean hasArm();

    public abstract boolean hasWrist();

    public abstract boolean hasIntake();

    public abstract boolean hasCompressor();

    public abstract boolean hasExtension();

    // Limelight Pipelines
    public int DRIVE_VIEW = 0;
    public int TARGETTING_VIEW = 2;

    // Limelight Constants
    public double visionAreaConstant = 1.0;
    public double yVisionkP = 0.4;
    public double xVisionkP = 1.0;

    // Swerve Calculations Constants (measurements are in inches)
    public double kWheelbaseLength;
    public double kWheelbaseWidth;

    // Swerve Module Positions (relative to the center of the drive base)
    public Translation2d kVehicleToModuleZero;
    public Translation2d kVehicleToModuleOne;
    public Translation2d kVehicleToModuleTwo;
    public Translation2d kVehicleToModuleThree;

    public List<Translation2d> kModulePositions;
    public double kV_PurePursuit;
    public double kA_PurePursuit;
    public double kMaxVelocityInchesPerSec;

    /* Gamespec vars */

    public int kEncoderUnitsPerRev;

    public int gTimeoutMs;
    public int gSlotIdx;

    // Arm
    public int armMasterID;
    public int armSlaveID;
    public int kArmCruiseVel;
    public int kArmAcceleration;

    public double kArmF;
    public double kArmP;
    public double kArmI;
    public double kArmD;

    public double gArmAngularOffset;

    public boolean armMasterInverted;
    public boolean armSlaveInverted;
    public boolean armMasterSensorPhase;

    // Wrist/Intake
    public int rollerIntakeID;

    public int intakePdpChannel;
    public int vacuumPdpChannel;

    public double intakePower;
    public double intakeStallPower;
    public double intakeOutPower;
    public double intakeVaccPower;

    public int hatchVacId;
    public int releaseId;

    public double rollerCurrentThres;
    public double vacuumCurrentThres;

    public int wristID;

    public double kWristCruiseVel;
    public double kWristAcceleration;

    public double kWristF;
    public double kWristP;
    public double kWristI;
    public double kWristD;

    public double gWristMinLimit;
    public double gWristMaxLimit;

    public double gWristMaxLimitCargo;

    public double gWristAngularOffset;
    public double gWristGroundOffset;

    public boolean wristMotorInverted;

    public boolean wristSensorPhase;
    public double MAX_WRIST_POWER;

    // Extension
    public int extensionID;

    public boolean extensionSensorPhase;
    public boolean extensionMotorInverted;

    public double kExtF;
    public double kExtP;
    public double kExtI;
    public double kExtD;

    public int kExtCruiseVel;
    public int kExtAcceleration;

    public double gExtGearRatio;
    public double gExtSpoolDiameter;
    public double gExtOffset;
    public double gExtMinLim;
    public double gExtMaxLim;

    public double gArmExtLength;

    // Power Distribution Panel
    public int PdpID;

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }

    private static RobotHardware instance = null;

    public static RobotHardware getInstance() {
        if (instance == null) {
            if (RobotState.getInstance().getCurrentRobot().equals(Bot.Automatic)) {
                RobotState.getInstance().setCurrentRobot(Util.parseRobotNameToEnum(Util.readRobotName()));
            }
            switch (RobotState.getInstance().getCurrentRobot()) {
            case ProgrammingBot:
                instance = new RobotHardwareProgammingBot();
                break;
            case Automatic:
                System.err.println("Robot should not be set to automatic... something went wrong");
                break;
            }
            instance.initalizeConstants();
            //Util.setPseudoInverseForwardKinematicsMatrix();
        }
        return instance;
    }

}
