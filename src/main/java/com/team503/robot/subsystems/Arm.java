package com.team503.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.drivers.LazyTalonSRX;
import com.team503.lib.util.Util;
import com.team503.robot.Robot;
import com.team503.robot.subsystems.requests.Prerequisite;
import com.team503.robot.subsystems.requests.Request;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {
	private static Arm instance = null;

	public static Arm getInstance() {
		if (instance == null)
			instance = new Arm();
		return instance;
	}

	LazyTalonSRX arm;

	private double targetAngle = 0.0;
	private double maxAllowableAngle = Robot.bot.kArmMaxControlAngle;

	public void setMaxAllowableAngle(double angle) {
		maxAllowableAngle = angle;
		lockAngle();
	}

	public enum ArmControlState {
		OPEN_LOOP, POSITION
	}

	private ArmControlState currentState = ArmControlState.OPEN_LOOP;

	PeriodicIO periodicIO = new PeriodicIO();

	private Arm() {
		arm = new LazyTalonSRX(Robot.bot.ARM);

		arm.configVoltageCompSaturation(12.0, 10);
		arm.enableVoltageCompensation(true);
		arm.configNominalOutputForward(0.0 / 12.0, 10);
		arm.configContinuousCurrentLimit(25, 10);
		arm.configPeakCurrentLimit(30, 10);
		arm.configPeakCurrentDuration(100, 10);
		arm.enableCurrentLimit(true);

		arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		arm.setInverted(true);
		arm.setSensorPhase(true);
		arm.setNeutralMode(NeutralMode.Brake);

		configurationOne();
		arm.configForwardSoftLimitThreshold(armAngleToEncUnits(Robot.bot.kArmMaxControlAngle), 10);
		arm.configReverseSoftLimitThreshold(armAngleToEncUnits(Robot.bot.kArmMinControlAngle), 10);
		// arm.configForwardSoftLimitEnable(true, 10); //TODO
		// arm.configReverseSoftLimitEnable(true, 10);
		arm.getSensorCollection().setPulseWidthPosition(0, 10);
		arm.setSelectedSensorPosition(0);

		setOpenLoop(0.0);
	}

	private void configurationOne() {
		arm.selectProfileSlot(0, 0);
		arm.config_kP(0, 2.0, 10); 
		arm.config_kI(0, 0.0, 10);
		arm.config_kD(0, 0.0, 10);
		arm.config_kF(0, 1023.0 / 350.0, 10);
		arm.config_kP(1, 2.0, 10);
		arm.config_kI(1, 0.0, 10);
		arm.config_kD(1, 0.0, 10);
		arm.config_kF(1, 1023.0 / 350.0, 10);
		arm.configMotionCruiseVelocity((int) (Robot.bot.kArmMaxSpeed * 1.0), 10);
		arm.configMotionAcceleration((int) (Robot.bot.kArmMaxSpeed * 3.0), 10);
		arm.configMotionSCurveStrength(6);
	}

	public void setOpenLoop(double output) {
		periodicIO.demand = output ;
		currentState = ArmControlState.OPEN_LOOP;
	}

	public boolean isOpenLoop() {
		return currentState == ArmControlState.OPEN_LOOP;
	}

	public void setAngle(double angle) {
		if (isSensorConnected()) {
			configurationOne();
			if (angle <= maxAllowableAngle) {
				targetAngle = angle;
			} else {
				targetAngle = maxAllowableAngle;
			}
			if (angle > getAngle())
				arm.selectProfileSlot(1, 0);
			else
				arm.selectProfileSlot(0, 0);
			periodicIO.demand = armAngleToEncUnits(targetAngle);
			currentState = ArmControlState.POSITION;
		} else {
			DriverStation.reportError("Arm encoder not detected!", false);
			stop();
		}
	}

	public void lockAngle() {
		setAngle(getAngle());
	}

	public Request angleRequest(double angle) {
		return new Request() {

			@Override
			public void act() {
				arm.configMotionCruiseVelocity((int) (Robot.bot.kArmMaxSpeed));
				setAngle(angle);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetAngle();
			}

		};
	}

	public Request angleRequest(double angle, double speedScalar) {
		return new Request() {

			@Override
			public void act() {
				// arm.configMotionCruiseVelocity(
				// (int) ((isHighGear ? Robot.bot.kArmMaxSpeedHighGear :
				// Robot.bot.kArmMaxSpeedLowGear)
				// * speedScalar));

				arm.configMotionCruiseVelocity((int) (Robot.bot.kArmMaxSpeed * speedScalar));
				setAngle(angle);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetAngle();
			}

		};
	}

	public Request angleRequest(double angle, double speedScalar, boolean wait) {
		return new Request() {

			@Override
			public void act() {
				arm.configMotionCruiseVelocity((int) (Robot.bot.kArmMaxSpeed * speedScalar));
				setAngle(angle);
			}

			@Override
			public boolean isFinished() {
				if (wait)
					return hasReachedTargetAngle();
				else
					return true;
			}

		};
	}

	public Request lockAngleRequest() {
		return new Request() {

			@Override
			public void act() {
				lockAngle();
			}

		};
	}

	public Request openLoopRequest(double output) {
		return new Request() {

			@Override
			public void act() {
				setOpenLoop(output);
			}

		};
	}

	public Prerequisite angleRequisite(double angle, boolean above) {
		return new Prerequisite() {

			@Override
			public boolean met() {
				return Util.epsilonEquals(Math.signum(angle - getAngle()), above ? -1.0 : 1.0);
			}

		};
	}

	public double getAngle() {
		return encUnitsToArmAngle(periodicIO.position);
	}

	public boolean hasReachedTargetAngle() {
		return Math.abs(targetAngle - getAngle()) <= Robot.bot.kArmAngleTolerance;
	}

	public double encUnitsToDegrees(double encUnits) {
		return encUnits / 4096.0 / Robot.bot.kArmEncoderToOutputRatio * 360.0;
	}

	public int degreesToEncUnits(double degrees) {
		return (int) (degrees / 360.0 * Robot.bot.kArmEncoderToOutputRatio * 4096.0);
	}

	public double encUnitsToArmAngle(int encUnits) {
		return Robot.bot.kArmStartingAngle + encUnitsToDegrees(encUnits - Robot.bot.kArmStartingEncoderPosition);
	}

	public int armAngleToEncUnits(double armAngle) {
		return Robot.bot.kArmStartingEncoderPosition + degreesToEncUnits(armAngle - Robot.bot.kArmStartingAngle);
	}

	public boolean isSensorConnected() {
		int pulseWidthPeriod = arm.getSensorCollection().getPulseWidthRiseToRiseUs();
		boolean connected = pulseWidthPeriod != 0;
		if (!connected)
			hasEmergency = true;
		return connected; 
	}

	public void onLoop(double timestamp) {
		if (arm.getOutputCurrent() > Robot.bot.kArmMaxCurrent) {
			DriverStation.reportError("Arm current high", false);
		}
	}

	@Override
	public synchronized void readPeriodicInputs() {
		periodicIO.position = arm.getSelectedSensorPosition(0);
		if (Robot.bot.kDebuggingOutput) {
			periodicIO.velocity = arm.getSelectedSensorVelocity(0);
			periodicIO.voltage = arm.getMotorOutputVoltage();
			periodicIO.current = arm.getOutputCurrent();
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (currentState == ArmControlState.POSITION)
			arm.set(ControlMode.MotionMagic, periodicIO.demand);
		else
			arm.set(ControlMode.PercentOutput, periodicIO.demand);
	}

	@Override
	public void stop() {
		setOpenLoop(0.0);
	}

	@Override
	public void zeroSensors() {

	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber("Arm Angle", getAngle());
		if (Robot.bot.kDebuggingOutput) {
			SmartDashboard.putNumber("Arm Current", periodicIO.current);
			SmartDashboard.putNumber("Arm Voltage", periodicIO.voltage);
			SmartDashboard.putNumber("Arm Encoder", periodicIO.position);
			SmartDashboard.putNumber("Arm Pulse Width Position", arm.getSensorCollection().getPulseWidthPosition());
			SmartDashboard.putNumber("Arm Velocity", periodicIO.velocity);
			SmartDashboard.putNumber("Arm Error", encUnitsToDegrees(arm.getClosedLoopError(0)));
			if (arm.getControlMode() == ControlMode.MotionMagic)
				SmartDashboard.putNumber("Arm Setpoint", arm.getClosedLoopTarget(0));
		}
	}

	public boolean checkSystem() {
		double currentMinimum = 0.5;
		double currentMaximum = 20.0;

		boolean passed = true;

		if (!isSensorConnected()) {
			System.out.println("Arm sensor is not connected, connect and retest");
			return false;
		}

		double startingEncPosition = arm.getSelectedSensorPosition(0);
		arm.set(ControlMode.PercentOutput, 3.0 / 12.0);
		Timer.delay(1.0);
		double current = arm.getOutputCurrent();
		arm.set(ControlMode.PercentOutput, 0.0);
		if (Math.signum(arm.getSelectedSensorPosition(0) - startingEncPosition) != 1.0) {
			System.out.println("Arm needs to be reversed");
			passed = false;
		}
		if (current < currentMinimum) {
			System.out.println("Arm current too low: " + current);
			passed = false;
		} else if (current > currentMaximum) {
			System.out.println("Arm current too high: " + current);
			passed = false;
		}

		return passed;
	}

	public static class PeriodicIO {
		// Inputs
		public int position;
		public int velocity;
		public double voltage;
		public double current;

		// Outputs
		public double demand;
	}
}
