
package frc.robot;

import frc.robot.SwerveModule;
import frc.subsystem.Pigeon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

    private boolean fieldCentric = true;

    public final double L = 21.0;
    public final double W = 21.0;

    private SwerveModule backRight;
    private SwerveModule backLeft;
    private SwerveModule frontRight;
    private SwerveModule frontLeft;

    // Constructor
    public SwerveDrive() {
        // before module inversion
        // this.backRight = new
        // SwerveModule(3,7,20.0,0.0,2.200,6.577,748,300,900,true,true,true,false,false);
        // this.backLeft = new
        // SwerveModule(4,8,20.0,0.0,2.200,6.577,980,300,900,true,true,true,false,false);
        this.backRight = new SwerveModule(3, 7, 20.0, 0.0, 2.200, 6.577, 748, 300, 900, true, false, false, false,
                false);
        this.backLeft = new SwerveModule(4, 8, 20.0, 0.0, 2.200, 6.577, 980, 300, 900, true, false, false, false,
                false);
        // was 275 FR
        this.frontRight = new SwerveModule(2, 6, 20.0, 0.0, 2.200, 6.577, 787, 300, 900, true, false, true, false,
                false);
        this.frontLeft = new SwerveModule(1, 5, 2.0, 0.0, 2.200, 6.577, 222, 300, 900, true, false, false, false,
                false); // change drive inverted to false
    }

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
        fwd *= -1;

        if (fieldCentric) {
            double angle = Math.toRadians(Pigeon.getInstance().getYaw());
            double temp = fwd * Math.cos(angle) + str * Math.sin(angle);
            str = -fwd * Math.sin(angle) + str * Math.cos(angle);
            fwd = temp;
        }

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
}