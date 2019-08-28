/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.loops;

import com.team503.lib.geometry.Pose;
import com.team503.lib.geometry.Rotation2d;
import com.team503.lib.kinematics.SwerveDriveKinematics;
import com.team503.lib.kinematics.SwerveDriveOdometry;
import com.team503.lib.kinematics.SwerveModuleState;
import com.team503.lib.util.FFDashboard;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.subsystems.Pigeon;
import com.team503.robot.subsystems.SwerveDrive;

/**
 * Add your docs here.
 */
public class FroggyPoseController {
    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Robot.bot.kModulePositions);
    private static final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(kinematics);

    private static SwerveDrive mSwerve = SwerveDrive.getInstance();

    private static final FFDashboard table = new FFDashboard("Localization");

    public static synchronized void updateOdometry() {
        final double robotAngle = Pigeon.getInstance().getYaw();
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = mSwerve.getModules().get(i).getState();
        }
        RobotState.getInstance().setCurrentPose(mOdometry.update(Rotation2d.fromDegrees(robotAngle), moduleStates));
    }

    public static synchronized void resetPose(final Pose pose) {
        mOdometry.resetPosition(pose);
    }

    public static synchronized void outputPoseToDashboard() {
        final Pose robotPose = RobotState.getInstance().getCurrentPose();
        table.putString("Current Pose", robotPose.toString());
        table.putNumber("Current Pose X", robotPose.getX());
        table.putNumber("Current Pose Y", robotPose.getY());
    }

}
