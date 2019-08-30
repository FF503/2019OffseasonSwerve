/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.lib.kinematics;

import com.team503.lib.geometry.Pose;
import com.team503.lib.geometry.Rotation2d;
import com.team503.lib.geometry.Translation2d;
import com.team503.lib.geometry.Twist2d;
import com.team503.lib.util.FFDashboard;

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * swerve drive encoders and swerve azimuth encoders.
 *
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
public class SwerveDriveOdometry {
    private final SwerveDriveKinematics m_kinematics;
    private Pose m_pose;
    private double m_prevTimeSeconds = -1;
    private FFDashboard table = new FFDashboard("Localization");

    private final CalculationMode calcMode = CalculationMode.Non_Linear;

    private Rotation2d m_previousAngle;

    /**
     * Constructs a SwerveDriveOdometry object.
     *
     * @param kinematics  The swerve drive kinematics for your drivetrain.
     * @param initialPose The starting position of the robot on the field.
     */
    public SwerveDriveOdometry(SwerveDriveKinematics kinematics, Pose initialPose) {
        m_kinematics = kinematics;
        m_pose = initialPose;
        m_previousAngle = initialPose.getRotation();
    }

    /**
     * Constructs a SwerveDriveOdometry object with the default pose at the origin.
     *
     * @param kinematics The swerve drive kinematics for your drivetrain.
     */
    public SwerveDriveOdometry(SwerveDriveKinematics kinematics) {
        this(kinematics, new Pose(new Translation2d(), 0));
    }

    /**
     * Resets the robot's position on the field.
     *
     * @param pose The position on the field that your robot is at.
     */
    public void resetPosition(Pose pose) {
        m_pose = pose;
        m_previousAngle = pose.getRotation();
    }

    /**
     * Updates the robot's position on the field using forward kinematics and
     * integration of the pose over time. This method takes in the current time as a
     * parameter to calculate period (difference between two timestamps). The period
     * is used to calculate the change in distance from a velocity. This also takes
     * in an angle parameter which is used instead of the angular rate that is
     * calculated from forward kinematics.
     *
     * @param currentTimeSeconds The current time in seconds.
     * @param angle              The angle of the robot.
     * @param moduleStates       The current state of all swerve modules. Please
     *                           provide the states in the same order in which you
     *                           instantiated your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose updateWithTime(double currentTimeSeconds, Rotation2d angle, SwerveModuleState... moduleStates) {
        double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
        m_prevTimeSeconds = currentTimeSeconds;

        var chassisState = m_kinematics.toChassisSpeeds(moduleStates);
        Pose newPose;
        if (calcMode == CalculationMode.Non_Linear) {
            newPose = m_pose.plus(m_pose.exp(new Twist2d(chassisState.vx * period, chassisState.vy * period,
                    angle.minus(m_previousAngle).getRadians())));
        } else {
            Translation2d positionVector = m_pose.getTranslation().plus(chassisState.getTranslation().times(period));
            newPose = new Pose(positionVector, angle.getDegrees());
        }

        table.putNumber("Current Time", currentTimeSeconds);
        table.putNumber("Robot Velocity X", chassisState.vx);
        table.putNumber("Robot Velocity Y", chassisState.vy);
        table.putNumber("Robot Velocity (Mag)", Math.hypot(chassisState.vy, chassisState.vx));
        table.putNumber("Robot Acceleration (Mag)", Math.hypot(chassisState.vy / period, chassisState.vx / period));

        m_previousAngle = angle;
        m_pose = new Pose(currentTimeSeconds, newPose.getTranslation(), angle.getDegrees());
        return m_pose;
    }

    /**
     * Updates the robot's position on the field using forward kinematics and
     * integration of the pose over time. This method automatically calculates the
     * current time to calculate period (difference between two timestamps). The
     * period is used to calculate the change in distance from a velocity. This also
     * takes in an angle parameter which is used instead of the angular rate that is
     * calculated from forward kinematics.
     *
     * @param angle        The angle of the robot.
     * @param moduleStates The current state of all swerve modules. Please provide
     *                     the states in the same order in which you instantiated
     *                     your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose update(Rotation2d angle, SwerveModuleState... moduleStates) {
        return updateWithTime(System.currentTimeMillis() / 1000.0, angle, moduleStates);
    }

    private enum CalculationMode {
        Linear, Non_Linear;
    }
}
