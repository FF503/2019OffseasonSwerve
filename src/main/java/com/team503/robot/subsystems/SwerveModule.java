
package com.team503.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team503.lib.geometry.Rotation2d;
import com.team503.lib.kinematics.SwerveModuleState;
import com.team503.lib.util.Util;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.RobotState.Bot;

public class SwerveModule {
    private static final double kTurnEncoderClicksperRevolution = Robot.bot.kTurnEncoderClicksperRevolution;
    private static final double kWheelDiameter = Robot.bot.wheelDiameter;
    private static final double kAzimuthDegreesPerClick = 360.0 / kTurnEncoderClicksperRevolution;
    private final double kAzimuthClicksPerDegree = kTurnEncoderClicksperRevolution / 360.0;

    private final int countsPerRotation = 42; // Counts/Rotation
    private final double driveGearRatio = (12.0 / 40.0) * (20.0 / 40.0); // Unitless
    private final double CIRCUMFERENCE = (Math.PI * kWheelDiameter); // Rotations/inch
    private final double COUNTS_PER_INCH = countsPerRotation / driveGearRatio / CIRCUMFERENCE; // Counts/Inch
    private final double driveVelocityConversionFactor = (COUNTS_PER_INCH * 60.0); // 60 Inches/Rotation

    private static final int kSlotIdx = 0;
    private static final int kTimeoutMs = 30;
    private static double power = 0.0;

    private CANSparkMax driveMotor;
    private TalonSRX turnMotor;
    private CANEncoder motorEncoder;

    // Swerve Module Specific - must be changed for each swerve module !!!!!
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private int kBaseEncoderClicks;
    private int kMagicCruiseVelocity;
    private int kMagicCruiseAcceleration;
    private boolean kTurnCountsDecreasing;
    private boolean kDriveMotorInverted;
    private boolean kDriveEncoderInverted;
    private boolean kTurnMotorInverted;
    private boolean kTurnEncoderInverted;

    private double lastSetAngle = 0.0;

    public SwerveModule(int driveMotorID, int turnMotorID, double P, double I, double D, double F,
            int startingEncoderClick, int cruiseVelocity, int cruiseAccel, boolean turnCountsDecreasing,
            boolean DriveInverted, boolean DriveEncoderInverted, boolean TurnMotorInverted,
            boolean TurnEncoderInverted) {

        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.motorEncoder = new CANEncoder(this.driveMotor);
        this.turnMotor = new TalonSRX(turnMotorID);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        // this is the encoder count when the wheel is aligned forward at the start
        this.kBaseEncoderClicks = startingEncoderClick;
        this.kMagicCruiseVelocity = cruiseVelocity;
        this.kMagicCruiseAcceleration = cruiseAccel;
        this.kTurnCountsDecreasing = turnCountsDecreasing;
        this.kDriveMotorInverted = DriveInverted;
        this.kDriveEncoderInverted = DriveEncoderInverted;
        this.kTurnMotorInverted = TurnMotorInverted;
        this.kTurnEncoderInverted = TurnEncoderInverted;
        this.kP = P;
        this.kI = I;
        this.kD = D;
        this.kF = F;

        // configure drive motor
        driveMotor.setInverted(kDriveMotorInverted);
        // driveMotor.setOpenLoopRampRate(1.0);

        // configure turn motor
        if (RobotState.getInstance().getCurrentRobot().equals(Bot.FFSwerve)) {
            turnMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kSlotIdx, kTimeoutMs);
            turnMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
        } else if (RobotState.getInstance().getCurrentRobot().equals(Bot.ProgrammingBot)) {
            turnMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        }
        turnMotor.configFeedbackNotContinuous(true, kTimeoutMs);

        // set to true to invert sensor
        turnMotor.setInverted(kTurnMotorInverted);
        turnMotor.setSensorPhase(kTurnEncoderInverted);

        turnMotor.configNominalOutputForward(0, kTimeoutMs);
        turnMotor.configNominalOutputReverse(0, kTimeoutMs);
        turnMotor.configPeakOutputForward(1, kTimeoutMs);
        turnMotor.configPeakOutputReverse(-1, kTimeoutMs);
        turnMotor.selectProfileSlot(kSlotIdx, 0); // slot index = 0 , pidloopidx = 0
        turnMotor.config_kF(kSlotIdx, kF, kTimeoutMs);
        turnMotor.config_kP(kSlotIdx, kP, kTimeoutMs);
        turnMotor.config_kI(kSlotIdx, kI, kTimeoutMs);
        turnMotor.config_kD(kSlotIdx, kD, kTimeoutMs);
        turnMotor.configMotionCruiseVelocity(kMagicCruiseVelocity, kTimeoutMs);
        turnMotor.configMotionAcceleration(kMagicCruiseAcceleration, kTimeoutMs);

        driveMotor.setSmartCurrentLimit(50);
        turnMotor.enableCurrentLimit(true);
        turnMotor.configContinuousCurrentLimit(15);
        
        // motorEncoder.setVelocityConversionFactor(driveVelocityConversionFactor);
    }

    public void drive(double speed, double angle) {
        if (Util.alternateShouldReverse(angle, getTurnEncoderPositioninDegrees())) {
            speed *= -1;
            angle = Util.boundAngle0to360Degrees(angle + 180);
        }

        double trueSpeed = speed;
        if (speed == 503.0) {
            trueSpeed = 0.0;
        }
        this.power = trueSpeed;
        setDriveMotorSpeed(trueSpeed);

        // angle is bound to -180 - +180 and degrees are from 0-360
        // convert bounded angle into true compass degrees
        // double trueAngle = angle;
        // if (angle < 0) {
        // trueAngle = 180 + (180 + angle);
        // }
        double trueAngle = Util.boundAngle0to360Degrees(angle);

        // convert angle (0-360) into encoder clicks (0-1024)
        int desiredclicks = (int) Math.round(trueAngle * kAzimuthClicksPerDegree);

        if (kTurnCountsDecreasing) {
            // this means a positive right turn mean decreasing encoder counts
            desiredclicks = kBaseEncoderClicks - desiredclicks;
            if (desiredclicks < 0) {
                desiredclicks += kTurnEncoderClicksperRevolution;
            }

        } else {
            // this means a positive right trn with increasing encoder counts
            // addin the base starting clicks when the wheel is pointing to zero
            desiredclicks += kBaseEncoderClicks;
            // becuase we are using an absolute encoder the value must be between 0 and 1024
            if (desiredclicks >= kTurnEncoderClicksperRevolution) {
                desiredclicks -= kTurnEncoderClicksperRevolution;
            }
        }
        if (Math.abs(speed) > 0.02 || speed == 503.0) {
            turnMotor.set(ControlMode.MotionMagic, desiredclicks);
        }
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(speed);
    }

    public void resetDriveEncoder() {
        motorEncoder.setPosition(0.0);
    }

    /**
     * 
     * @return clicks
     */
    public double getDriveEncoderClicks() {
        double pos = countsPerRotation * motorEncoder.getPosition();
        if(kDriveEncoderInverted) {
        pos *= -1;
        }
        return pos;
    }

    public double getDriveEncoderRPM() {
        double vel = motorEncoder.getVelocity() * countsPerRotation;
        if(kDriveEncoderInverted) {
        vel *= -1;
        }
        return vel;
    }

    /**
     * 
     * @return in inches
     */
    public double getDriveMotorPosition() {
        return getDriveEncoderClicks() / COUNTS_PER_INCH;
    }

    /**
     * 
     * @return in inches/second
     */
    public double getDriveMotorVelocity() {
        return getDriveEncoderRPM() / driveVelocityConversionFactor;
    }

    public SwerveModuleState getState() {

        return new SwerveModuleState(getDriveMotorVelocity(),Rotation2d.fromDegrees(-getTurnEncoderPositioninDegrees()));
    }

    public double getTurnEncoderPosition() {
        return turnMotor.getSelectedSensorPosition(0);
    }

    public double getTurnClosedLoopError() {
        return turnMotor.getClosedLoopError();
    }

    public void setDriveMotorCurrentLimit(int limit) {
        driveMotor.setSmartCurrentLimit(limit);
    }

    public void coastDrive() {
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    public void brakeDrive() {
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getTurnEncoderPositioninDegrees() {
        // get the base clicks = zero degrees for this particular wheel

        double pos = getTurnEncoderPosition();
        // relative position
        double relpos = 0;
        // actual position
        int actpos;

        if (kTurnCountsDecreasing) {
            // this means a positive right turn means decreasing encoder counts
            relpos = (kBaseEncoderClicks - pos);
        } else {
            // this means a positive right turn with increasing encoder counts
            relpos = (pos - kBaseEncoderClicks);
        }

        // adjust for absolute encoder with 0-1024 clicks
        if (relpos < 0) {
            relpos += kTurnEncoderClicksperRevolution;
        }
        actpos = (int) Math.round(relpos * kAzimuthDegreesPerClick);

        if (actpos == 360) {
            actpos = 0;
        }

        return actpos;
    }

    /********************************************************************************
     * Section - Encoder Conversion Routines
     *******************************************************************************/

    private static double ticksToInches(double ticks) {
        return rotationsToInches(ticksToRotations(ticks));
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (kWheelDiameter * Math.PI);
    }

    private static double ticksToRotations(double ticks) {
        int kEncoderUnitsPerRev = 42; // Rev Native Internal Encoder clicks per revolution
        return ticks / kEncoderUnitsPerRev;
    }

    private static double inchesToRotations(double inches) {
        return inches / (kWheelDiameter * Math.PI);
    }

    public double getXComponentVelocity() {
        return Math.cos(Math.toRadians(Util.unitCircleify(getTurnEncoderPositioninDegrees())))
                * driveMotor.getEncoder().getVelocity();
    }

    public double getYComponentVelocity() {
        return Math.sin(Math.toRadians(Util.unitCircleify(getTurnEncoderPositioninDegrees())))
                * driveMotor.getEncoder().getVelocity();
    }

    public double getMotorPower() {
        return power;
    }

}