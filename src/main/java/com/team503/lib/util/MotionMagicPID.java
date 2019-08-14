/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.util;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.team503.robot.Robot;
import com.team503.robot.subsystems.SuperStructureSystem;

/**
 * Container for motion magic PIDF constants
 */
public class MotionMagicPID {

    private TalonSRX pTalon;
    double kConstantP, kConstantI, kConstantD, kConstantF;
    int velocity, acceleration;

    public MotionMagicPID(SuperStructureSystem syst, double p, double i, double d, double f, int cruiseVel, int accel) {

        this.pTalon = syst.getTalon();
        this.kConstantP = p;
        this.kConstantI = i;
        this.kConstantD = d;
        this.kConstantF = f;
        this.velocity = cruiseVel;
        this.acceleration = accel;
    }

    public void configPIDs() {

        /* Set relevant frame periods to be at least as fast as periodic rate */
        pTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Robot.bot.gTimeoutMs);
        pTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Robot.bot.gTimeoutMs);

        /* Set the peak and nominal outputs */
        pTalon.configNominalOutputForward(0, Robot.bot.gTimeoutMs);
        pTalon.configNominalOutputReverse(0, Robot.bot.gTimeoutMs);
        pTalon.configPeakOutputForward(1, Robot.bot.gTimeoutMs);
        pTalon.configPeakOutputReverse(-1, Robot.bot.gTimeoutMs);

        pTalon.config_kF(Robot.bot.gSlotIdx, kConstantF, Robot.bot.gTimeoutMs);
        pTalon.config_kP(Robot.bot.gSlotIdx, kConstantP, Robot.bot.gTimeoutMs);
        pTalon.config_kI(Robot.bot.gSlotIdx, kConstantI, Robot.bot.gTimeoutMs);
        pTalon.config_kD(Robot.bot.gSlotIdx, kConstantD, Robot.bot.gTimeoutMs);

        /* set acceleration and vcruise velocity - see documentation */
        pTalon.configMotionCruiseVelocity(velocity, Robot.bot.gTimeoutMs); // 1235 //2515
        pTalon.configMotionAcceleration(acceleration, Robot.bot.gTimeoutMs);
    }
}
