package com.team503.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team503.lib.kinematics.SwerveModuleState;
import com.team503.lib.util.Util;
import com.team503.robot.Robot;
import com.team254.drivers.LazyTalonSRX;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends Subsystem {
	LazyTalonSRX rotationMotor;
	CANSparkMax driveMotor;
	CANEncoder driveEncoder;
	String moduleID;
	String name = "Module ";
	int rotationSetpoint = 0;
	double driveSetpoint = 0;
	int encoderOffset;
	int encoderReverseFactor = 1;
	boolean useDriveEncoder = true;
	boolean tenVoltRotationMode = false;
	boolean standardCarpetDirection = true;

	// Motion magic variables
	private double kP;
	private double kI;
	private double kD;
	private double kF;
	private int kMagicCruiseVelocity;
	private int kMagicCruiseAcceleration;
	private boolean kTurnCountsDecreasing;
	private boolean kDriveMotorInverted;
	private boolean kDriveEncoderInverted;
	private boolean kTurnMotorInverted;
	private boolean kTurnEncoderInverted;

	public void setCarpetDirection(boolean standardDirection) {
		standardCarpetDirection = standardDirection;
	}

	PeriodicIO periodicIO = new PeriodicIO();

	public SwerveModule(int rotationSlot, int driveSlot, String moduleID, int encoderOffset, double P, double I,
			double D, double F, int cruiseVelocity, int cruiseAccel, boolean turnCountsDecreasing,
			boolean DriveInverted, boolean DriveEncoderInverted, boolean TurnMotorInverted, boolean TurnEncoderInverted,
			Translation2d startingPose) {
		name += (moduleID + " ");
		rotationMotor = new LazyTalonSRX(rotationSlot);
		driveMotor = new CANSparkMax(driveSlot, MotorType.kBrushless);
		driveEncoder = new CANEncoder(driveMotor);

		this.moduleID = moduleID;
		this.encoderOffset = encoderOffset - degreesToEncUnits(90);
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

		configureMotors();

		// configure drive motor
		driveMotor.setInverted(kDriveMotorInverted);
		getRawAngle();
	}

	public synchronized void invertDriveMotor(boolean invert) {
		driveMotor.setInverted(invert);
	}

	public synchronized void invertRotationMotor(boolean invert) {
		rotationMotor.setInverted(invert);
	}

	public synchronized void reverseDriveSensor(boolean reverse) {

	}

	public synchronized void reverseRotationSensor(boolean reverse) {
		encoderReverseFactor = reverse ? -1 : 1;
		rotationMotor.setSensorPhase(reverse);
	}

	public synchronized void setMaxRotationSpeed(double maxSpeed) {
		rotationMotor.configMotionCruiseVelocity((int) maxSpeed, 0);
	}

	public synchronized void disableDriveEncoder() {
		useDriveEncoder = false;
	}

	private void configureMotors() {
		rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
		rotationMotor.configFeedbackNotContinuous(true, 30);
		rotationMotor.setSensorPhase(this.kTurnEncoderInverted);
		rotationMotor.setInverted(this.kTurnMotorInverted);
		// rotationMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
		rotationMotor.setNeutralMode(NeutralMode.Brake);
		rotationMotor.configVoltageCompSaturation(7.0, 10);
		rotationMotor.enableVoltageCompensation(true);
		rotationMotor.configAllowableClosedloopError(0, 0, 10);
		rotationMotor.configMotionAcceleration(this.kMagicCruiseAcceleration, 10);
		rotationMotor.configMotionCruiseVelocity(this.kMagicCruiseVelocity, 10);
		rotationMotor.selectProfileSlot(0, 0);
		// Slot 1 is for normal use
		rotationMotor.config_kP(0, this.kP, 10);
		rotationMotor.config_kI(0, this.kI, 10);
		rotationMotor.config_kD(0, this.kD, 10);
		rotationMotor.config_kF(0, this.kF, 10);
		// // Slot 2 is reserved for the beginning of auto
		// rotationMotor.config_kP(1, 8.0, 10);
		// rotationMotor.config_kI(1, 0.0, 10);
		// rotationMotor.config_kD(1, 200.0, 10);
		// rotationMotor.config_kF(1, 1023.0 / Constants.kSwerveRotation10VoltMaxSpeed,
		// 10);
		rotationMotor.set(ControlMode.MotionMagic, rotationMotor.getSelectedSensorPosition(0));
		if (!isRotationSensorConnected()) {
			DriverStation.reportError(name + "rotation encoder not detected!", false);
			hasEmergency = true;
		}

		driveEncoder.setPosition(0.0);
		driveMotor.enableVoltageCompensation(12.0);
		driveMotor.setOpenLoopRampRate(0.25);
		driveMotor.setInverted(this.kDriveMotorInverted);
		driveMotor.setIdleMode(IdleMode.kBrake);
	}

	private boolean isRotationSensorConnected() {
		int pulseWidthPeriod = rotationMotor.getSensorCollection().getPulseWidthRiseToRiseUs();
		return pulseWidthPeriod != 0;
	}

	private double getRawAngle() {
		return encUnitsToDegrees(periodicIO.rotationPosition);
	}

	public Rotation2d getModuleAngle() {
		if (!this.kTurnCountsDecreasing) {
			return Rotation2d.fromDegrees(encUnitsToDegrees(encoderOffset) - getRawAngle());
		}
		return Rotation2d.fromDegrees(getRawAngle() - encUnitsToDegrees(encoderOffset));
	}

	public Rotation2d getFieldCentricAngle(Rotation2d robotHeading) {
		Rotation2d normalizedAngle = getModuleAngle();
		return normalizedAngle.rotateBy(robotHeading);
	}

	public void setModuleAngle(double goalAngle) {

		double newAngle = Util.placeInAppropriate0To360Scope(getRawAngle(),
				encUnitsToDegrees(encoderOffset) + goalAngle * (this.kTurnCountsDecreasing ? 1 : -1));
		SmartDashboard.putNumber(name + " AZ NEW Target", newAngle);
		SmartDashboard.putNumber(name + " AZ Target", goalAngle);

		// int setpoint = degreesToEncUnits(encUnitsToDegrees(encoderOffset + 90));

		double setpoint = degreesToEncUnits(newAngle);

		SmartDashboard.putNumber(name + " AZ Setpoint", setpoint);

		periodicIO.rotationControlMode = ControlMode.MotionMagic;
		periodicIO.rotationDemand = setpoint;
	}

	public boolean angleOnTarget() {
		double error = encUnitsToDegrees(Math.abs(periodicIO.rotationDemand - periodicIO.rotationPosition));
		return error < 4.5;
	}

	public void set10VoltRotationMode(boolean tenVolts) {
		if (tenVolts && !tenVoltRotationMode) {
			rotationMotor.selectProfileSlot(1, 0);
			rotationMotor.configVoltageCompSaturation(10.0, 10);
			tenVoltRotationMode = true;

		} else if (!tenVolts && tenVoltRotationMode) {
			rotationMotor.selectProfileSlot(0, 0);
			rotationMotor.configVoltageCompSaturation(7.0, 10);
			tenVoltRotationMode = false;
		}
	}

	public void setRotationOpenLoop(double power) {
		periodicIO.rotationControlMode = ControlMode.PercentOutput;
		periodicIO.rotationDemand = power;
	}

	/**
	 * @param velocity Normalized value
	 */
	public void setDriveOpenLoop(double velocity) {
		/*
		 * double volts = 0.0; if(!Util.epsilonEquals(velocity, 0.0,
		 * Constants.kEpsilon)){ velocity *= Constants.kSwerveMaxSpeedInchesPerSecond;
		 * double m = Constants.kVoltageVelocityEquations[moduleID][velocity < 0 ? 1 :
		 * 0][0]; double b = Constants.kVoltageVelocityEquations[moduleID][velocity < 0
		 * ? 1 : 0][1]; volts = (velocity - b) / m; volts = Util.deadBand(volts, 1.0); }
		 */

		periodicIO.driveControlMode = ControlMode.PercentOutput;
		// periodicIO.driveDemand = volts / 12.0;
		periodicIO.driveDemand = velocity;
	}

	// public void setDrivePositionTarget(double deltaDistanceInches) {
	// driveMotor.selectProfileSlot(0, 0);
	// periodicIO.driveControlMode = ControlMode.MotionMagic;
	// periodicIO.driveDemand = periodicIO.drivePosition +
	// inchesToEncUnits(deltaDistanceInches);
	// }

	// public boolean drivePositionOnTarget() {
	// if (driveMotor.getControlMode() == ControlMode.MotionMagic)
	// return encUnitsToInches((int) Math.abs(periodicIO.driveDemand -
	// periodicIO.drivePosition)) < 2.0;
	// return false;
	// }

	// public void setVelocitySetpoint(double inchesPerSecond) {
	// driveMotor.selectProfileSlot(1, 0);
	// periodicIO.driveControlMode = ControlMode.Velocity;
	// periodicIO.driveDemand = inchesPerSecondToEncVelocity(inchesPerSecond);
	// }

	private double getDriveDistanceInches() {
		return encUnitsToInches(periodicIO.drivePosition);
	}

	public double encUnitsToInches(double encUnits) {
		return encUnits / Robot.bot.kSwerveEncUnitsPerInch;
	}

	public int inchesToEncUnits(double inches) {
		return (int) (inches * Robot.bot.kSwerveEncUnitsPerInch);
	}

	public double encVelocityToInchesPerSecond(double rpm) {
		return encUnitsToInches(rpm) / 60.0;
	}

	public int inchesPerSecondToEncVelocity(double inchesPerSecond) {
		return (int) (inchesToEncUnits(inchesPerSecond / 10.0));
	}

	public int degreesToEncUnits(double degrees) {
		return (int) (degrees / 360.0 * Robot.bot.kTurnEncoderClicksperRevolution);
	}

	public double encUnitsToDegrees(double encUnits) {
		return encUnits / Robot.bot.kTurnEncoderClicksperRevolution * 360.0;
	}

	@Override
	public synchronized void readPeriodicInputs() {
		periodicIO.rotationPosition = rotationMotor.getSelectedSensorPosition(0);
		if (useDriveEncoder)
			periodicIO.drivePosition = (int) driveEncoder.getPosition();
		// periodicIO.velocity = driveMotor.getSelectedSensorVelocity();
		if (Robot.bot.kDebuggingOutput) {
			periodicIO.velocity = (int) driveEncoder.getVelocity();
		}
		/*
		 * if(moduleID == 3){ periodicIO.velocity =
		 * driveMotor.getSelectedSensorVelocity(0); periodicIO.driveVoltage =
		 * driveMotor.getMotorOutputVoltage(); if(periodicIO.velocity != 0 &&
		 * periodicIO.driveVoltage != 0) Logger.log("(" + periodicIO.driveVoltage + ", "
		 * + encVelocityToFeetPerSecond(periodicIO.velocity) + "), "); }
		 */
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		rotationMotor.set(periodicIO.rotationControlMode, periodicIO.rotationDemand);
		driveMotor.set(periodicIO.driveDemand);
	}

	@Override
	public synchronized void stop() {
		setDriveOpenLoop(0.0);
	}

	public synchronized void disable() {
		setDriveOpenLoop(0.0);
		setRotationOpenLoop(0.0);
	}

	public synchronized void resetRotationToAbsolute() {
		rotationMotor.setSelectedSensorPosition(
				encoderReverseFactor * (rotationMotor.getSensorCollection().getPulseWidthPosition() - encoderOffset), 0,
				10);
	}

	@Override
	public synchronized void zeroSensors() {
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber(name + this.rotationMotor.getDeviceID() + " Angle", getModuleAngle().getDegrees());
		SmartDashboard.putNumber(name + "Inches Driven", getDriveDistanceInches());
		// SmartDashboard.putNumber(name + "Velocity",
		// encVelocityToInchesPerSecond(periodicIO.velocity));
		if (Robot.bot.kDebuggingOutput) {
			SmartDashboard.putNumber(name + "Pulse Width", rotationMotor.getSelectedSensorPosition(0));
			SmartDashboard.putNumber(name + "Drive Voltage", periodicIO.driveVoltage);
			SmartDashboard.putNumber(name + "Rotation Voltage", rotationMotor.getMotorOutputVoltage());
			SmartDashboard.putNumber(name + "Velocity", encVelocityToInchesPerSecond(periodicIO.velocity));
			if (rotationMotor.getControlMode() == ControlMode.MotionMagic)
				SmartDashboard.putNumber(name + "Error", encUnitsToDegrees(rotationMotor.getClosedLoopError(0)));
			// SmartDashboard.putNumber(name + "X", position.x());
			// SmartDashboard.putNumber(name + "Y", position.y());
			SmartDashboard.putNumber(name + "Drive Current", driveMotor.getOutputCurrent());
			SmartDashboard.putNumber(name + "Rotation Speed", rotationMotor.getSelectedSensorVelocity(0));
			SmartDashboard.putNumber(name + "Rotation Demand", periodicIO.rotationDemand);

		}
	}

	public static class PeriodicIO {
		// Inputs
		public int rotationPosition = 0;
		public int drivePosition = 0;
		public int velocity = 0;
		public double driveVoltage = 0.0;

		// Outputs
		public ControlMode rotationControlMode = ControlMode.PercentOutput;
		public ControlMode driveControlMode = ControlMode.PercentOutput;
		public double rotationDemand;
		public double driveDemand;
	}

	public SwerveModuleState getState() {
		return null;
	}

}
