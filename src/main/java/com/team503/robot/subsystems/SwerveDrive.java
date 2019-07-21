
package com.team503.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team503.robot.subsystems.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team503.lib.util.Util;
import com.team503.robot.Constants;
import com.team503.robot.RobotState;

public class SwerveDrive extends Subsystem {

    // Instance declaration
    private static SwerveDrive instance = null;

    public static SwerveDrive getInstance() {
        if (instance == null)
            instance = new SwerveDrive();
        return instance;
    }

    // Swerve Dimensions
    public final double L = 21.0;
    public final double W = 21.0;

    // Module declaration
    private SwerveModule backRight, backLeft, frontRight, frontLeft;
    private List<SwerveModule> modules;

    // Constructor
    public SwerveDrive() {
        // before module inversion
        // this.backRight = new
        // SwerveModule(3,7,20.0,0.0,2.200,6.577,748,300,900,true,true,true,false,false);
        // this.backLeft = new
        // SwerveModule(4,8,20.0,0.0,2.200,6.577,980,300,900,true,true,true,false,false);

        // this.backRight = new SwerveModule(3, 7, 20.0, 0.0, 2.200, 6.577, 748, 300,
        // 900, true, false, false, false,
        // false);
        // this.backLeft = new SwerveModule(4, 8, 20.0, 0.0, 2.200, 6.577, 980, 300,
        // 900, true, false, false, false,
        // false);
        // // was 275 FR
        // this.frontRight = new SwerveModule(2, 6, 20.0, 0.0, 2.200, 6.577, 787, 300,
        // 900, true, false, true, false,
        // false);
        // this.frontLeft = new SwerveModule(1, 5, 2.0, 0.0, 2.200, 6.577, 222, 300,
        // 900, true, false, false, false,
        // false); // change drive inverted to false
        try {
            this.backRight = Util.readSwerveJSON(Constants.SwerveFileNames.backRight);
            this.backLeft = Util.readSwerveJSON(Constants.SwerveFileNames.backLeft);
            this.frontRight = Util.readSwerveJSON(Constants.SwerveFileNames.frontRight);
            this.frontLeft = Util.readSwerveJSON(Constants.SwerveFileNames.frontLeft);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        modules = Arrays.asList(backRight, backLeft, frontLeft, frontRight);
    }

    // Teleop driving variables
    private Translation2d translationalVector = new Translation2d();
    private double rotationalInput = 0;
    private Translation2d lastDriveVector = new Translation2d();
    private final Translation2d rotationalVector = Translation2d.identity();
    private double lowPowerScalar = 0.6;

    public void setLowPowerScalar(double scalar) {
        lowPowerScalar = scalar;
    }

    private double maxSpeedFactor = 1.0;

    public void setMaxSpeed(double max) {
        maxSpeedFactor = max;
    }

    private boolean fieldCentric = true;

    /**
     * @return the fieldCentric
     */
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    /**
     * @param fieldCentric
     */
    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    // Takes joystick input an calculates drive wheel speed and turn motor angle
    public void drive(double str, double fwd, double rcw) {
        double r = Math.sqrt((L * L) + (W * W));

        if (fieldCentric) {
            double angle = Math.toRadians(RobotState.getInstance().getCurrentTheta());
            double temp = fwd * Math.cos(angle) + str * Math.sin(angle);
            str = -fwd * Math.sin(angle) + str * Math.cos(angle);
            fwd = temp;
        }

        translationalVector = new Translation2d(str, fwd);
        rotationalInput = rcw;

        double a = str - rcw * (L / r);
        double b = str + rcw * (L / r);
        double c = fwd - rcw * (W / r);
        double d = fwd + rcw * (W / r);

        double backRightSpeed = Math.sqrt((a * a) + (c * c));
        double backLeftSpeed = Math.sqrt((a * a) + (d * d));
        double frontRightSpeed = Math.sqrt((b * b) + (c * c));
        double frontLeftSpeed = Math.sqrt((b * b) + (d * d));

        double backRightAngle = (Math.atan2(a, c) * 180 / Math.PI);
        double backLeftAngle = (Math.atan2(a, d) * 180 / Math.PI);
        double frontRightAngle = (Math.atan2(b, c) * 180 / Math.PI);
        double frontLeftAngle = (Math.atan2(b, d) * 180 / Math.PI);

        // if the speed is zero and the right side = 0, then left side should be zero
        if (frontLeftSpeed == 0.0 && frontRightSpeed == 0.0) {
            if (frontRightAngle == 180.0) {
                frontRightAngle = 0.0;
                backRightAngle = 0.0;
            }
        }

        // normalize wheel speeds
        double max = frontRightSpeed;
        if (frontLeftSpeed > max) {
            max = frontLeftSpeed;
        }
        if (backLeftSpeed > max) {
            max = backLeftSpeed;
        }
        if (backRightSpeed > max) {
            max = backRightSpeed;
        }
        if (max > 1.0) {
            frontRightSpeed /= max;
            frontLeftSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        if (shouldReverse(backRightAngle, backRight.getTurnEncoderPositioninDegrees())) {
            backRightAngle += 180;
            backRightSpeed *= -1;
        }

        if (shouldReverse(backLeftAngle, backLeft.getTurnEncoderPositioninDegrees())) {
            backLeftAngle += 180;
            backLeftSpeed *= -1;
        }

        if (shouldReverse(frontRightAngle, frontRight.getTurnEncoderPositioninDegrees())) {
            frontRightAngle += 180;
            frontRightSpeed *= -1;
        }

        if (shouldReverse(frontLeftAngle, frontLeft.getTurnEncoderPositioninDegrees())) {
            frontLeftAngle += 180;
            frontLeftSpeed *= -1;
        }

        // Send speeds and angles to the drive motors
        backRight.drive(backRightSpeed, backRightAngle);
        backLeft.drive(backLeftSpeed, backLeftAngle);
        frontRight.drive(frontRightSpeed, frontRightAngle);
        frontLeft.drive(frontLeftSpeed, frontLeftAngle);

        SmartDashboard.putNumber("LF Calc Angle (deg)", frontLeftAngle);
        SmartDashboard.putNumber("RF Calc Angle (deg)", frontRightAngle);
        SmartDashboard.putNumber("LR Calc Angle (deg)", backLeftAngle);
        SmartDashboard.putNumber("RR Calc Angle (deg)", backRightAngle);

        // inform drives whats going on
        // SmartDashboard.putBoolean("Drive Motor Inverted", kDriveMotorInverted);
        // SmartDashboard.putBoolean("LF Turn Encoder Inverted", kEncoderInverted);
    }

    /**
     * 
     * @param goalAngle    Target Angle through drive vectors
     * @param currentAngle Current Angle of swerve module
     * @return if the module phase should be inverted
     */
    public static boolean shouldReverse(double goalAngle, double currentAngle) {
        goalAngle = boundAngle0to360Degrees(goalAngle);
        currentAngle = boundAngle0to360Degrees(currentAngle);
        double reversedAngle = boundAngle0to360Degrees(currentAngle + 180);
        double angleDifference = Math.abs(goalAngle - currentAngle);
        double reversedAngleDifference = Math.abs(goalAngle - reversedAngle);
        angleDifference = (angleDifference > 180) ? 360 - angleDifference : angleDifference;
        reversedAngleDifference = (reversedAngleDifference > 180) ? 360 - reversedAngleDifference
                : reversedAngleDifference;
        return reversedAngleDifference < angleDifference;
    }

    public static double boundAngle0to360Degrees(double angle) {
        while (angle >= 360.0) {
            angle -= 360.0;
        }
        while (angle < 0.0) {
            angle += 360.0;
        }
        return angle;
    }

    class SwerveInverseKinematics {

        private final int kNumberOfModules = 4;

        private List<Translation2d> moduleRelativePositions = Constants.kModulePositions;
        private List<Translation2d> moduleRotationDirections = updateRotationDirections();

        private List<Translation2d> updateRotationDirections() {
            List<Translation2d> directions = new ArrayList<>(kNumberOfModules);
            for (int i = 0; i < kNumberOfModules; i++) {
                directions.add(moduleRelativePositions.get(i).rotateBy(Rotation2d.fromDegrees(90)));
            }
            return directions;
        }

        public List<Translation2d> updateDriveVectors(Translation2d translationalVector, double rotationalMagnitude,
                Pose2d robotPose, boolean robotCentric) {
            SmartDashboard.putNumber("Vector Direction", translationalVector.direction().getDegrees());
            // SmartDashboard.putNumber("Vector Magnitude", translationalVector.norm());
            SmartDashboard.putNumber("Robot Velocity", translationalVector.norm());

            if (!robotCentric)
                translationalVector = translationalVector.rotateBy(robotPose.getRotation().inverse());
            List<Translation2d> driveVectors = new ArrayList<>(kNumberOfModules);
            for (int i = 0; i < kNumberOfModules; i++) {
                driveVectors.add(
                        translationalVector.translateBy(moduleRotationDirections.get(i).scale(rotationalMagnitude)));
            }
            double maxMagnitude = 1.0;
            for (Translation2d t : driveVectors) {
                double magnitude = t.norm();
                if (magnitude > maxMagnitude) {
                    maxMagnitude = magnitude;
                }
            }
            for (int i = 0; i < kNumberOfModules; i++) {
                Translation2d driveVector = driveVectors.get(i);
                driveVectors.set(i, driveVector.scale(1.0 / maxMagnitude));
            }
            return driveVectors;
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("LF Drive Position (clicks)", frontLeft.getDriveEncoderPosition());
        SmartDashboard.putNumber("LF Drive Velocity", frontLeft.getDriveEncoderVelocity());
        // SmartDashboard.putNumber("LF Drive Position (inches)",
        // ticksToInches(getDriveEncoderPosition()));
        SmartDashboard.putNumber("LF Turn Position (clicks)", frontLeft.getTurnEncoderPosition());
        SmartDashboard.putNumber("LF Turn Position (degrees)", frontLeft.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber("LF Turn Closed Loop Error (clicks)", frontLeft.getTurnClosedLoopError());

        SmartDashboard.putNumber("RF Drive Position (clicks)", frontRight.getDriveEncoderPosition());
        SmartDashboard.putNumber("RF Drive Velocity", frontRight.getDriveEncoderVelocity());
        // SmartDashboard.putNumber("RF Drive Position (inches)",
        // ticksToInches(getDriveEncoderPosition()));
        SmartDashboard.putNumber("RF Turn Position (clicks)", frontRight.getTurnEncoderPosition());
        SmartDashboard.putNumber("RF Turn Position (degrees)", frontRight.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber("RF Turn Closed Loop Error (clicks)", frontRight.getTurnClosedLoopError());

        SmartDashboard.putNumber("LR Drive Position (clicks)", backLeft.getDriveEncoderPosition());
        SmartDashboard.putNumber("LR Drive Velocity", backLeft.getDriveEncoderVelocity());
        // SmartDashboard.putNumber("LR Drive Position (inches)",
        // ticksToInches(getDriveEncoderPosition()));
        SmartDashboard.putNumber("LR Turn Position (clicks)", backLeft.getTurnEncoderPosition());
        SmartDashboard.putNumber("LR Turn Position (degrees)", backLeft.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber("LR Turn Closed Loop Error (clicks)", backLeft.getTurnClosedLoopError());

        SmartDashboard.putNumber("RR Drive Position (clicks)", backRight.getDriveEncoderPosition());
        SmartDashboard.putNumber("RR Drive Velocity", backRight.getDriveEncoderVelocity());
        // SmartDashboard.putNumber("RR Drive Position (inches)",
        // ticksToInches(getDriveEncoderPosition()));
        SmartDashboard.putNumber("RR Turn Position (clicks)", backRight.getTurnEncoderPosition());
        SmartDashboard.putNumber("RR Turn Position (degrees)", backRight.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber("RR Turn Closed Loop Error (clicks)", backRight.getTurnClosedLoopError());
    }

    @Override
    public void stop() {
        modules.forEach((m) -> m.setDriveMotorSpeed(0));
    }
}