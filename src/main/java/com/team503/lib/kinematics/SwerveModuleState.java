/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.kinematics;

import com.team503.lib.geometry.Rotation2d;

/*----------------------------------------------------------------------------*/

/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * Represents the state of one swerve module.
 */
public class SwerveModuleState implements Comparable<SwerveModuleState> {

    /**
     * Speed of the wheel of the module.
     */
    public double speed;

    /**
     * Angle of the module.
     */
    public Rotation2d angle = Rotation2d.fromDegrees(0);

    /**
     * Constructs a SwerveModuleState with zeros for speed and angle.
     */
    public SwerveModuleState() {
    }

    /**
     * Constructs a SwerveModuleState.
     *
     * @param speed The speed of the wheel of the module.
     * @param angle The angle of the module.
     */
    public SwerveModuleState(double speed, Rotation2d angle) {
        this.speed = speed;
        this.angle = angle;
    }

    /**
     * Compares two swerve module states. One swerve module is "greater" than the
     * other if its speed is higher than the other.
     *
     * @param o The other swerve module.
     * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
     */
    @Override
    public int compareTo(SwerveModuleState o) {
        return Double.compare(this.speed, o.speed);
    }
}
