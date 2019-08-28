
package com.team503.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.team503.lib.controllers.VisionFollowerController;
import com.team503.lib.geometry.Translation2d;
import com.team503.lib.util.SnappingPosition;
import com.team503.lib.util.SwerveHeadingController;
import com.team503.lib.util.Util;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.loops.LimelightProcessor;
import com.team503.robot.loops.LimelightProcessor.Pipeline;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends Subsystem {

    // Instance declaration
    private static SwerveDrive instance = null;
    private SwerveHeadingController headingController = new SwerveHeadingController();

    // Teleop driving variables
    private Translation2d translationalVector = new Translation2d();
    private Translation2d centerOfRotation = new Translation2d();
    private double rotationalInput = 0;

    private final double kLengthComponent;
    private final double kWidthComponent;

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

    public List<SwerveModule> getModules() {
        return modules;
    }

    public double[][] getWheelComponentVelocities() {
        double[][] returner = { { frontRight.getXComponentVelocity() }, { frontRight.getYComponentVelocity() },
                { frontLeft.getXComponentVelocity() }, { frontLeft.getYComponentVelocity() },
                { backLeft.getXComponentVelocity() }, { backLeft.getYComponentVelocity() },
                { backRight.getXComponentVelocity() }, { backRight.getYComponentVelocity() } };
        return returner;

    }

    // Constructor
    public SwerveDrive() {
        try {
            this.backRight = Util.readSwerveJSON(Robot.bot.getBackRightName());
            this.backLeft = Util.readSwerveJSON(Robot.bot.getBackLeftName());
            this.frontRight = Util.readSwerveJSON(Robot.bot.getFrontRightName());
            this.frontLeft = Util.readSwerveJSON(Robot.bot.getFrontLeftName());
        } catch (Exception e) {
            e.printStackTrace();
        }

        modules = Arrays.asList(backRight, backLeft, frontLeft, frontRight);

        double width = Robot.bot.kWheelbaseWidth, length = Robot.bot.kWheelbaseLength;
        double radius = Math.hypot(width, length);
        kLengthComponent = length / radius;
        kWidthComponent = width / radius;

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
        drive(translationVector, rotationalInput, false);
    }

    public void drive(Translation2d translationVector, double rotatationalInput, boolean lowPower) {
        translationVector = translationVector.normalize();
        double str = translationVector.getX();
        double fwd = translationVector.getY();
        drive(str, fwd, rotationalInput, lowPower);
    }

    // Takes joystick input an calculates drive wheel speed and turn motor angle
    public void drive(double str, double fwd, double rcw, boolean lowPower) {
        str *= (lowPower ? 0.3 : 1.0) * Robot.bot.requestDriveReversed;
        fwd *= (lowPower ? 0.5 : 1.0) * Robot.bot.requestDriveReversed;
        rcw *= lowPower ? 0.5 : 1.0;

        if (fieldCentric) {
            double angle = Math.toRadians(RobotState.getInstance().getCurrentTheta());
            double temp = fwd * Math.cos(angle) + str * Math.sin(angle);
            str = -fwd * Math.sin(angle) + str * Math.cos(angle);
            fwd = temp;
        }

        translationalVector = new Translation2d(str, fwd);
        rotationalInput = rcw;

        double a = str - rcw * kLengthComponent;
        double b = str + rcw * kLengthComponent;
        double c = fwd - rcw * kWidthComponent;
        double d = fwd + rcw * kWidthComponent;

        double backRightSpeed = Math.hypot(a, c);
        double backLeftSpeed = Math.hypot(a, d);
        double frontRightSpeed = Math.hypot(b, c);
        double frontLeftSpeed = Math.hypot(b, d);

        double backRightAngle = (Math.atan2(a, c) * 180 / Math.PI);
        double backLeftAngle = (Math.atan2(a, d) * 180 / Math.PI);
        double frontRightAngle = (Math.atan2(b, c) * 180 / Math.PI);
        double frontLeftAngle = (Math.atan2(b, d) * 180 / Math.PI);

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
        if (translationalVector.getX() == 0 && translationalVector.getY() == 0)
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
     * Targets the closest vision target and aproaches it using swerve/strafe
     * control and locking the angle
     * 
     */
    private VisionFollowerController visionFollower = new VisionFollowerController();

    public synchronized void visionFollow() {
        if (Robot.bot.hasLimelight()) {
            LimelightProcessor.getInstance().setPipeline(Pipeline.CLOSEST);
            setFieldCentric(false);
            Translation2d vector = visionFollower.getVectorToTarget(LimelightProcessor.getInstance().getTA(),
                    LimelightProcessor.getInstance().getTX());
            drive(vector);
        }
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

    public void resetDriveEncoder() {
        // modules.forEach((mod) -> mod.resetDriveEncoder());
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("LF Drive Position (clicks)", frontLeft.getDriveEncoderClicks());
        SmartDashboard.putNumber("LF Drive Position (inches)", frontLeft.getDriveMotorPosition());
        SmartDashboard.putNumber("LF Drive Velocity", frontLeft.getDriveMotorVelocity());
        SmartDashboard.putNumber("LF Turn Position (clicks)", frontLeft.getTurnEncoderPosition());
        SmartDashboard.putNumber("LF Turn Position (degrees)", frontLeft.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber("LF Turn Closed Loop Error (clicks)", frontLeft.getTurnClosedLoopError());

        SmartDashboard.putNumber("RF Drive Position (clicks)", frontRight.getDriveEncoderClicks());
        SmartDashboard.putNumber("RF Drive Position (inches)", frontRight.getDriveMotorPosition());
        SmartDashboard.putNumber("RF Drive Velocity", frontRight.getDriveMotorVelocity());
        SmartDashboard.putNumber("RF Turn Position (clicks)", frontRight.getTurnEncoderPosition());
        SmartDashboard.putNumber("RF Turn Position (degrees)", frontRight.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber("RF Turn Closed Loop Error (clicks)", frontRight.getTurnClosedLoopError());

        SmartDashboard.putNumber("LR Drive Position (clicks)", backLeft.getDriveEncoderClicks());
        SmartDashboard.putNumber("LR Drive Position (inches)", backLeft.getDriveMotorPosition());
        SmartDashboard.putNumber("LR Drive Velocity", backLeft.getDriveMotorVelocity());
        SmartDashboard.putNumber("LR Turn Position (clicks)", backLeft.getTurnEncoderPosition());
        SmartDashboard.putNumber("LR Turn Position (degrees)", backLeft.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber("LR Turn Closed Loop Error (clicks)", backLeft.getTurnClosedLoopError());

        SmartDashboard.putNumber("RR Drive Position (clicks)", backRight.getDriveEncoderClicks());
        SmartDashboard.putNumber("RR Drive Position (inches)", backRight.getDriveMotorPosition());
        SmartDashboard.putNumber("RR Drive Velocity", backRight.getDriveMotorVelocity());
        SmartDashboard.putNumber("RR Turn Position (clicks)", backRight.getTurnEncoderPosition());
        SmartDashboard.putNumber("RR Turn Position (degrees)", backRight.getTurnEncoderPositioninDegrees());
        SmartDashboard.putNumber("RR Turn Closed Loop Error (clicks)", backRight.getTurnClosedLoopError());

        SmartDashboard.putNumber("RF Power: ", frontRight.getMotorPower());
        SmartDashboard.putNumber("RR Power: ", backRight.getMotorPower());
        SmartDashboard.putNumber("LR Power: ", backLeft.getMotorPower());
        SmartDashboard.putNumber("LF Power: ", frontLeft.getMotorPower());
        SmartDashboard.putBoolean("Field Centric", isFieldCentric());
    }

    @Override
    public void stop() {
        modules.forEach((m) -> m.setDriveMotorSpeed(0));
    }

    @Override
    public void zeroSensors() {
        resetDriveEncoder();
    }

}
