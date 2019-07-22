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

/**
 * Add your docs here.
 */
public abstract class RobotHardware {

    private static RobotHardware instance = null;

    public RobotHardware() {
    }

    public static RobotHardware getInstance() {
        if (instance == null) {
            switch (RobotState.getInstance().getCurrentRobot()) {
            case ProgrammingBot:
                instance = new RobotHardwareProgammingBot();
                break;
            }

            instance.initalizeConstants();
        }
        return instance;
    }

    public abstract void initalizeConstants();

    // Constants

    // SwerveFileNames
    public abstract String getBackLeftName();

    public abstract String getBackRightName();

    public abstract String getFrontLeftName();

    public abstract String getFrontRightName();

    // Swerve Calculations Constants (measurements are in inches)
    public double kWheelbaseLength;
    public double kWheelbaseWidth;

    // Swerve Module Positions (relative to the center of the drive base)
    public Translation2d kVehicleToModuleZero;
    public Translation2d kVehicleToModuleOne;
    public Translation2d kVehicleToModuleTwo;
    public Translation2d kVehicleToModuleThree;

    public List<Translation2d> kModulePositions;

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

}
