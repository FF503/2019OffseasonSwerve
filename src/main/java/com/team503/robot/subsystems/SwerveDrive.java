
package com.team503.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Translation2d;
import com.team503.lib.util.SnappingPosition;
import com.team503.lib.util.SwerveHeadingController;
import com.team503.lib.util.Util;
import com.team503.robot.Robot;
import com.team503.robot.RobotHardware;
import com.team503.robot.RobotState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends Subsystem {

    // Instance declaration
    private static SwerveDrive instance = null;
    private SwerveHeadingController headingController = new SwerveHeadingController();

    // Teleop driving variables
    private Translation2d translationalVector = new Translation2d();
    private Translation2d centerOfRotation = new Translation2d();

    private double rotationalInput = 0;

    public static SwerveDrive getInstance() {
        if (instance == null)
            instance = new SwerveDrive();
        return instance;
    }

    public enum DriveMode {
        TeleopDrive, Defense, MotionProfling, Vision, PurePursuit;
    }

    private DriveMode mode = DriveMode.TeleopDrive;

    public DriveMode getMode() {
        return mode;
    }

    public void setMode(DriveMode mode) {
        this.mode = mode;
    }

    // Module declaration
    private SwerveModule backRight, backLeft, frontRight, frontLeft;
    private List<SwerveModule> modules;

    public double[][] getWheelComponentVelocities() {
        double[][] returner = { { frontRight.getXComponentVelocity() }, { frontRight.getYComponentVelocity() },
                { frontLeft.getXComponentVelocity() }, { frontLeft.getYComponentVelocity() },
                { backLeft.getXComponentVelocity() }, { backLeft.getYComponentVelocity() },
                { backRight.getXComponentVelocity() }, { backRight.getYComponentVelocity() } };
        return returner;

    }

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
            this.backRight = Util.readSwerveJSON(Robot.bot.getBackRightName());
            this.backLeft = Util.readSwerveJSON(Robot.bot.getBackLeftName());
            this.frontRight = Util.readSwerveJSON(Robot.bot.getFrontRightName());
            this.frontLeft = Util.readSwerveJSON(Robot.bot.getFrontLeftName());
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        modules = Arrays.asList(backRight, backLeft, frontLeft, frontRight);
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

    public void toggleFieldCentric() {
        this.fieldCentric = !this.fieldCentric;
    }

    public void drive(Translation2d translationVector) {
        drive(translationVector, getRotationalOutput());
    }

    public void drive(Translation2d translationVector, double rotatationalInput) {
        translationVector = translationVector.normalize();

        double str = translationVector.x();
        double fwd = translationVector.y();
        drive(str, fwd, rotationalInput);
    }

    // Takes joystick input an calculates drive wheel speed and turn motor angle
    private void drive(double str, double fwd, double rcw) {
        final double length = Robot.bot.kWheelbaseLength, width = Robot.bot.kWheelbaseWidth;
        double r = Math.sqrt((length * length) + (width * width));

        if (fieldCentric) {
            double angle = Math.toRadians(RobotState.getInstance().getCurrentTheta());
            double temp = fwd * Math.cos(angle) + str * Math.sin(angle);
            str = -fwd * Math.sin(angle) + str * Math.cos(angle);
            fwd = temp;
        }

        translationalVector = new Translation2d(str, fwd);
        rotationalInput = rcw;

        double a = str - rcw * (length / r);
        double b = str + rcw * (length / r);
        double c = fwd - rcw * (width / r);
        double d = fwd + rcw * (width / r);

        double backRightSpeed = Math.sqrt((a * a) + (c * c));
        double backLeftSpeed = Math.sqrt((a * a) + (d * d));
        double frontRightSpeed = Math.sqrt((b * b) + (c * c));
        double frontLeftSpeed = Math.sqrt((b * b) + (d * d));

        double backRightAngle = (Math.atan2(a, c) * 180 / Math.PI);
        double backLeftAngle = (Math.atan2(a, d) * 180 / Math.PI);
        double frontRightAngle = (Math.atan2(b, c) * 180 / Math.PI);
        double frontLeftAngle = (Math.atan2(b, d) * 180 / Math.PI);

        // // if the speed is zero and the right side = 0, then left side should be zero
        // if (frontLeftSpeed == 0.0 && frontRightSpeed == 0.0) {
        // if (frontRightAngle == 180.0) {
        // frontRightAngle = 0.0;
        // backRightAngle = 0.0;
        // }
        // }

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

    public void defensePosition() {
        if (mode == DriveMode.Defense) {
            double backRightSpeed = 0, backLeftSpeed = 0, frontRightSpeed = 0, frontLeftSpeed = 0, backRightAngle = -45,
                    backLeftAngle = 45, frontLeftAngle = -45, frontRightAngle = 45;

            backRight.drive(backRightSpeed, backRightAngle);
            backLeft.drive(backLeftSpeed, backLeftAngle);
            frontRight.drive(frontRightSpeed, frontRightAngle);
            frontLeft.drive(frontLeftSpeed, frontLeftAngle);
        }
    }

    public synchronized double getRotationalOutput() {
        return headingController.getRotationalOutput();
    }

    public synchronized void rotate(SnappingPosition snappingPosition) {
        rotate(snappingPosition.getAngle());
    }

    // Various methods to control the heading controller
    public synchronized void rotate(double goalHeading) {
        if (translationalVector.x() == 0 && translationalVector.y() == 0)
            rotateInPlace(goalHeading);
        else {
            stabilize(goalHeading);
        }
    }

    public void setPathHeading(SnappingPosition pos) {
        setPathHeading(pos.getAngle());
    }

    public void setPathHeading(double goalHeading) {
        headingController.setSnapTarget(
                Util.placeInAppropriate0To360Scope(RobotState.getInstance().getCurrentTheta(), goalHeading));
    }

    public synchronized void stabilize(double goalHeading) {
        headingController.setStabilizationTarget(
                Util.placeInAppropriate0To360Scope(RobotState.getInstance().getCurrentTheta(), goalHeading));
    }

    public void rotateInPlace(double goalHeading) {
        headingController.setStationaryTarget(
                Util.placeInAppropriate0To360Scope(RobotState.getInstance().getCurrentTheta(), goalHeading));
    }

    /**
     * 
     * @param goalAngle    Target Angle through drive vectors
     * @param currentAngle Current Angle of swerve module
     * @return if the module phase should be inverted
     */
    private boolean shouldReverse(double goalAngle, double currentAngle) {
        return Util.shouldReverse(goalAngle, currentAngle);
    }

    public Translation2d getCenterOfRotation() {
        return this.centerOfRotation;
    }

    public void setCenterOfRotation(Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
    }

    public void setCenterOfRotation(double x, double y) {
        setCenterOfRotation(new Translation2d(x, y));
    }

    public void setBrakeMode() {
        modules.forEach((mod) -> mod.brakeDrive());
    }

    public void setCoastMode() {
        modules.forEach((mod) -> mod.coastDrive());
    }

    private void setCurrentLimit(int limit) {
        modules.forEach((mod) -> mod.setDriveMotorCurrentLimit(limit));
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
        // setCoastMode();
    }
}
