package com.team503.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.team254.drivers.LazyCANSparkMax;
import com.team254.drivers.LazyTalonSRX;
// import com.team1323.frc2019.Robot.bot;
// import com.team1323.frc2019.Robot.bot;
// import com.team1323.frc2019.loops.ILooper;
// import com.team1323.frc2019.loops.Loop;
// import com.team1323.frc2019.subsystems.requests.Prerequisite;
// import com.team1323.frc2019.subsystems.requests.Request;
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

	// Solenoid shifter;

	// boolean isHighGear = false;

	// public boolean isHighGear() {
	// return isHighGear;
	// }

	// boolean highGearConfig = false;

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
		arm.configContinuousCurrentLimit(25, 10); // TODO
		arm.configPeakCurrentLimit(30, 10);
		arm.configPeakCurrentDuration(100, 10);
		arm.enableCurrentLimit(true);

		arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		arm.setInverted(false);
		arm.setSensorPhase(false);
		arm.getSensorCollection().setPulseWidthPosition(0, 100);
		resetToAbsolutePosition();
		configurationOne();
		arm.configForwardSoftLimitThreshold(armAngleToEncUnits(Robot.bot.kArmMaxControlAngle), 10);
		arm.configReverseSoftLimitThreshold(armAngleToEncUnits(Robot.bot.kArmMinControlAngle), 10);
		arm.configForwardSoftLimitEnable(true, 10);
		arm.configReverseSoftLimitEnable(true, 10);

		setOpenLoop(0.0);

		// shifter = new Solenoid(Robot.bot.DRIVEBASE_PCM, Robot.bot.WRIST_SHIFTER);
	}

	private void configurationOne() { // TODO TUNE
		arm.selectProfileSlot(0, 0);
		arm.config_kP(0, 1.25, 10); // going down 2.5
		arm.config_kI(0, 0.0, 10);
		arm.config_kD(0, 60.0, 10);// 80.0
		arm.config_kF(0, 1023.0 / Robot.bot.kArmMaxSpeed, 10);
		arm.config_kP(1, 1.25, 10);// going up 2.0
		arm.config_kI(1, 0.0, 10);
		arm.config_kD(1, 60.0, 10);// 80.0
		arm.config_kF(1, 1023.0 / Robot.bot.kArmMaxSpeed, 10);
		arm.configMotionCruiseVelocity((int) (Robot.bot.kArmMaxSpeed * 1.0), 10);
		arm.configMotionAcceleration((int) (Robot.bot.kArmMaxSpeed * 3.0), 10);
		arm.configMotionSCurveStrength(6);

		// highGearConfig = true;
	}

	// public void configForLowGear() {
	// arm.selectProfileSlot(2, 0);
	// arm.config_kP(2, 3.0, 10);
	// arm.config_kI(2, 0.0, 10);
	// arm.config_kD(2, 30.0, 10);
	// arm.config_kF(2, 1023.0 / Robot.bot.kArmMaxSpeedLowGear, 10);
	// arm.config_kP(3, 3.0, 10);
	// arm.config_kI(3, 0.0, 10);
	// arm.config_kD(3, 60.0, 10);
	// arm.config_kF(3, 1023.0 / Robot.bot.kArmMaxSpeedLowGear, 10);
	// arm.configMotionCruiseVelocity((int) (Robot.bot.kArmMaxSpeedLowGear * 1.0),
	// 10);
	// arm.configMotionAcceleration((int) (Robot.bot.kArmMaxSpeedLowGear * 3.0),
	// 10);
	// arm.configMotionSCurveStrength(4);

	// highGearConfig = false;
	// System.out.println("Low gear set");
	// }

	// public void setHighGear(boolean high) {
	// if (high && !isHighGear) {
	// shifter.set(true);
	// configForHighGear();
	// isHighGear = true;
	// } else if (!high && isHighGear) {
	// shifter.set(false);
	// configForLowGear();
	// isHighGear = false;
	// }
	// DriverStation.reportError("Arm shifted to: " + (high ? "high" : "low"),
	// true);
	// }

	public void setOpenLoop(double output) {
		periodicIO.demand = output * 0.5;// TODO update coefficient
		currentState = ArmControlState.OPEN_LOOP;
	}

	public boolean isOpenLoop() {
		return currentState == ArmControlState.OPEN_LOOP;
	}

	public void setAngle(double angle) {
		if (isSensorConnected()) {
			// if (isHighGear && !highGearConfig) {
			configurationOne();
			// } else if (!isHighGear && highGearConfig) {
			// configForLowGear();
			// }
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
				// arm.configMotionCruiseVelocity(
				// (int) ((isHighGear ? Robot.bot.kArmMaxSpeedHighGear :
				// Robot.bot.kArmMaxSpeedLowGear)
				// * speedScalar));
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

	// public Request gearShiftRequest(boolean high) {
	// return new Request() {

	// @Override
	// public void act() {
	// setHighGear(high);
	// }

	// };
	// }

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
		return connected; // TODO Does this work
	}

	public void resetToAbsolutePosition() {
		int absolutePosition = (int) Util.boundToScope(0, 4096, arm.getSensorCollection().getPulseWidthPosition());
		if (encUnitsToArmAngle(absolutePosition) > Robot.bot.kArmMaxPhysicalAngle) {
			absolutePosition -= 4096;
		} else if (encUnitsToArmAngle(absolutePosition) < Robot.bot.kArmMinPhysicalAngle) {
			absolutePosition += 4096;
		}
		double armAngle = encUnitsToArmAngle(absolutePosition);
		if (armAngle > Robot.bot.kArmMaxPhysicalAngle || armAngle < Robot.bot.kArmMinPhysicalAngle) {
			DriverStation.reportError("Arm angle is out of bounds", false);
			hasEmergency = true;
		}
		arm.setSelectedSensorPosition(absolutePosition, 0, 10);
	}

	// private final Loop loop = new Loop() {

	// @Override
	// public void onStart(double timestamp) {

	// }

	// @Override
	public void onLoop(double timestamp) {
		if (arm.getOutputCurrent() > Robot.bot.kArmMaxCurrent) {
			// stop();
			DriverStation.reportError("Arm current high", false);
		}
	}

	// @Override
	// public void onStop(double timestamp) {

	// }

	// };

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

	// @Override
	// public void registerEnabledLoops(ILooper enabledLooper) {
	// enabledLooper.register(loop);
	// }

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
