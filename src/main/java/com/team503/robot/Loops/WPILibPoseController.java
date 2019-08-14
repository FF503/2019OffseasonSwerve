/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.Loops;

import com.team503.lib.kinematics.SwerveDriveKinematics;
import com.team503.lib.kinematics.SwerveDriveOdometry;
import com.team503.lib.kinematics.SwerveModuleState;
import com.team503.lib.util.Pose;
import com.team503.robot.Robot;
import com.team503.robot.subsystems.SwerveDrive;

/**
 * Add your docs here.
 */
public class WPILibPoseController {
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Robot.bot.kModulePositions);

    private final SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(kinematics);

    private SwerveDrive mSwerve = SwerveDrive.getInstance();

    public void updateOdometry() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = mSwerve.getModules().get(i).getState();
        }
        mOdometry.update(moduleStates);
    }

    public void resetPose(final Pose pose) {
        mOdometry.resetPosition(pose);
    }
}
