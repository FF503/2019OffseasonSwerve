package com.team503.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;
import com.team254.drivers.LazyCANSparkMax;
import com.team254.drivers.LazyCANSparkMax.ControlMode;
import com.team503.lib.util.Util;
import com.team503.robot.Robot;
// import com.team1323.frc2019.Robot.bot;
// import com.team1323.frc2019.Robot.bot;
// import com.team1323.frc2019.loops.ILooper;
// import com.team1323.frc2019.loops.LimelightProcessor;
// import com.team1323.frc2019.loops.Loop;
import com.team503.robot.subsystems.requests.Prerequisite;
import com.team503.robot.subsystems.requests.Request;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {
	private static Elevator instance = null;

	public static Elevator getInstance() {
		if (instance == null)
			instance = new Elevator();
		return instance;
	}

	LazyCANSparkMax elevator;// , motor2;
	// List<LazyCANSparkMax> motors, slaves;
	private double targetHeight = 0.0;

	public double getTargetHeight() {
		return targetHeight;
	}

	private boolean configuredForAscent = true;
	private boolean limitsEnabled = false;

	public boolean limitsEnabled() {
		return limitsEnabled;
	}

	// public TalonSRX getPigeonTalon(){
	// return motor2;
	// }

	public enum ControlState {
		Neutral, Position, OpenLoop, Locked
	}

	private ControlState state = ControlState.Neutral;

	public ControlState getState() {
		return state;
	}

	public void setState(ControlState newState) {
		state = newState;
	}

	double manualSpeed = Robot.bot.kElevatorTeleopManualSpeed;

	public void setManualSpeed(double speed) {
		manualSpeed = speed;
	}

	PeriodicIO periodicIO = new PeriodicIO();

	private Elevator() {
		elevator = new LazyCANSparkMax(Robot.bot.ELEVATOR);
		// motor2 = new LazyCANSparkMax(Robot.bot.ELEVATOR_2);

		// motors = Arrays.asList(elevator, motor2);
		// slaves = Arrays.asList(motor2);

		// slaves.forEach((s) -> s.set(ControlMode.Follower, Robot.bot.ELEVATOR_1));

		// for(LazyCANSparkMax motor : motors){
		// elevator.configVoltageCompSaturation(12.0, 10);
		elevator.enableVoltageCompensation(12.0);
		elevator.setIdleMode(IdleMode.kBrake);
		// }

		// if (Robot.bot.kIsUsingCompBot) {
		elevator.setInverted(true); // TODO
		// motor2.setInverted(true);
		// } else {
		// elevator.setInverted(false);
		// motor2.setInverted(false);
		// }

		// elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
		// 0, 10);
		elevator.setSensorPhase(true);
		elevator.setEncoderPosition(0.0);
		// zeroSensors();
		// elevator.configReverseSoftLimitThreshold(Robot.bot.kElevatorEncoderStartingPosition,
		// 10);
		// elevator.configForwardSoftLimitThreshold(
		// Robot.bot.kElevatorEncoderStartingPosition +
		// inchesToEncUnits(Robot.bot.kElevatorMaxHeight), 10);
		// elevator.configForwardSoftLimitEnable(true, 10);
		// elevator.configReverseSoftLimitEnable(true, 10);
		enableLimits(true);

		setCurrentLimit(Robot.bot.kElevatorCurrentLimit);

		resetToAbsolutePosition();

		configForAscent();

	}

	private void configForAscent() {
		manualSpeed = Robot.bot.kElevatorTeleopManualSpeed;

		elevator.setP(0, 0.0);// 0.75 going up
		elevator.setI(0, 0.0);
		elevator.setD(0, 0.000);// 20.0
		elevator.setFF(0, 1.0 / Robot.bot.kElevatorMaxSpeed);

		elevator.setP(1, 0.0);// 2.5 going down
		elevator.setI(1, 0.0);
		elevator.setD(1, 0.0);// 20.0
		elevator.setFF(1, 1.0 / Robot.bot.kElevatorMaxSpeed);

		elevator.setCruiseVelocity(0, Robot.bot.kElevatorMaxSpeed * 1.0);
		elevator.setAcceleration(0, Robot.bot.kElevatorMaxSpeed * 1.0);

		elevator.setCruiseVelocity(1, Robot.bot.kElevatorMaxSpeed * 1.0);

		elevator.setAcceleration(1, Robot.bot.kElevatorMaxSpeed * 0.5);
		// elevator.configMotionSCurveStrength(0);

		configuredForAscent = true;
	}

	private void configforDescent() {
		// elevator.configMotionSCurveStrength(4);

		configuredForAscent = false;
	}

	public void configForTeleopSpeed() {
		configForAscent();
	}

	public void configForAutoSpeed() {
		configForAscent();
	}

	public void enableLimits(boolean enable) {
		// elevator.overrideSoftLimitsEnable(enable);
		limitsEnabled = enable;
	}

	public void setCurrentLimit(int amps) {
		// for (LazyCANSparkMax motor : motors) {
		elevator.setSmartCurrentLimit(amps, amps);
		// elevator.enableCurrentLimit(true);
		// }
	}

	public void setOpenLoop(double output) {
		setState(ControlState.OpenLoop);
		periodicIO.demand = output * manualSpeed;
	}

	public boolean isOpenLoop() {
		return getState() == ControlState.OpenLoop;
	}

	public synchronized void setTargetHeight(double heightFeet) {
		setState(ControlState.Position);
		if (heightFeet > Robot.bot.kElevatorMaxHeight)
			heightFeet = Robot.bot.kElevatorMaxHeight;
		else if (heightFeet < Robot.bot.kElevatorMinHeight)
			heightFeet = Robot.bot.kElevatorMinHeight;
		if (isSensorConnected()) {
			if (heightFeet > getHeight()) {
				elevator.selectProfileSlot(0);
				configForAscent();
			} else {
				elevator.selectProfileSlot(1);
				configforDescent();
			}
			targetHeight = heightFeet;
			periodicIO.demand = elevatorHeightToEncUnits(heightFeet);
			System.out.println("Set elevator height to: " + heightFeet);
			onTarget = false;
			startTime = Timer.getFPGATimestamp();
		} else {
			DriverStation.reportError("Elevator encoder not detected!", false);
			stop();
		}
	}

	public synchronized void lockHeight() {
		setState(ControlState.Locked);
		if (isSensorConnected()) {
			targetHeight = getHeight();
			periodicIO.demand = periodicIO.position;
		} else {
			DriverStation.reportError("Elevator encoder not detected!", false);
			stop();
		}
	}

	public Request openLoopRequest(double output) {
		return new Request() {

			@Override
			public void act() {
				setOpenLoop(output);
			}

		};
	}

	public Request heightRequest(double height) {
		return new Request() {

			@Override
			public void act() {
				setTargetHeight(height);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetHeight() || isOpenLoop();
			}

		};
	}

	public Request lockHeightRequest() {
		return new Request() {

			@Override
			public void act() {
				lockHeight();
			}

		};
	}

	public Prerequisite heightRequisite(double height, boolean above) {
		return new Prerequisite() {

			@Override
			public boolean met() {
				return Util.epsilonEquals(Math.signum(height - getHeight()), above ? -1.0 : 1.0);
			}

		};
	}

	public double getHeight() {
		return encUnitsToElevatorHeight(periodicIO.position);
	}

	public double getVelocityFeetPerSecond() {
		return encUnitsToInches(periodicIO.velocity) * 10.0;
	}

	boolean onTarget = false;
	double startTime = 0.0;

	public boolean hasReachedTargetHeight() {
		if (elevator.getControlMode() == ControlMode.SmartMotion) {
			if ((Math.abs(targetHeight - getHeight()) <= Robot.bot.kElevatorHeightTolerance)) {
				if (!onTarget) {
					// System.out.println("Elevator done in: " + (Timer.getFPGATimestamp() -
					// startTime)); //TODO uncomment this if desired
					onTarget = true;
				}
				return true;
			}
		}
		return false;
	}

	private int inchesToEncUnits(double inches) {
		return (int) (inches * Robot.bot.kElevatorTicksPerInch);
	}

	private double encUnitsToInches(double encUnits) {
		return encUnits / Robot.bot.kElevatorTicksPerInch;
	}

	private double elevatorHeightToEncUnits(double elevatorHeight) {
		return inchesToEncUnits(elevatorHeight) - Robot.bot.kElevatorEncoderStartingPosition;
	}

	private double encUnitsToElevatorHeight(double encUnits) {
		return encUnitsToInches(encUnits + Robot.bot.kElevatorEncoderStartingPosition);
	}

	public boolean inVisionRange(List<double[]> ranges) {
		return inVisionRange(getHeight(), ranges);
	}

	public boolean inVisionRange(double height, List<double[]> ranges) {
		boolean inRange = false;
		for (double[] range : ranges) {
			inRange |= (height >= range[0]) && (height <= range[1]);
		}
		return inRange;
	}

	public double nearestVisionHeight(List<double[]> ranges) {
		return nearestVisionHeight(getHeight(), ranges);
	}

	public double nearestVisionHeight(double height, List<double[]> ranges) {
		if (inVisionRange(height, ranges))
			return height;
		double nearestHeight = Robot.bot.kElevatorMaxHeight;
		double smallestDistance = Math.abs(height - nearestHeight);
		for (double[] range : ranges) {
			for (int i = 0; i < 2; i++) {
				if (Math.abs(height - range[i]) < smallestDistance) {
					smallestDistance = Math.abs(height - range[i]);
					nearestHeight = (i == 0) ? range[i] + 0.5 : range[i] - 0.5;
				}
			}
		}
		if (Util.epsilonEquals(nearestHeight, Robot.bot.kElevatorMaxHeight))
			return height;
		return nearestHeight;
	}

	private boolean getMotorsWithHighCurrent() {
		return periodicIO.current >= Robot.bot.kElevatorMaxCurrent;
	}

	// private final Loop loop = new Loop() {

	// @Override
	// public void onStart(double timestamp) {

	// }

	// @Override
	public void onLoop(double timestamp) {
		if (getMotorsWithHighCurrent()) {
			DriverStation.reportError("Elevator current too high", false);
			// stop();
		}
	}

	// @Override
	// public void onStop(double timestamp) {

	// }

	// };

	public boolean isSensorConnected() {
		// int pulseWidthPeriod =
		// elevator.getSensorCollection().getPulseWidthRiseToRiseUs();
		// boolean connected = pulseWidthPeriod != 0;
		// if (!connected)
		// hasEmergency = true;
		// return connected;
		return true;
	}

	public void resetToAbsolutePosition() {
		// int absolutePosition = (int) Util.boundToScope(0, 4096,
		// elevator.getEncoderPosition());
		// if (encUnitsToElevatorHeight(absolutePosition) >
		// Robot.bot.kElevatorMaxInitialHeight) {
		// absolutePosition -= 4096;
		// } else if (encUnitsToElevatorHeight(absolutePosition) <
		// Robot.bot.kElevatorMinInitialHeight) {
		// absolutePosition += 4096;
		// }
		// double height = encUnitsToElevatorHeight(absolutePosition);
		// if (height > Robot.bot.kElevatorMaxInitialHeight || height <
		// Robot.bot.kElevatorMinInitialHeight) {
		// DriverStation.reportError("Elevator height is out of bounds", false);
		// hasEmergency = true;
		// }
		// elevator.setEncoderPosition(inchesToEncUnits(inches));
	}

	@Override
	public synchronized void readPeriodicInputs() {
		periodicIO.position = elevator.getEncoderPosition();

		if (Robot.bot.kDebuggingOutput) {
			periodicIO.velocity = elevator.getEncoderVelocity();
			periodicIO.voltage = elevator.getBusVoltage();
			periodicIO.current = elevator.getOutputCurrent();
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (getState() == ControlState.Position || getState() == ControlState.Locked) {

			if (!hasReachedTargetHeight()) {
				elevator.set(ControlMode.SmartMotion, periodicIO.demand);
			}

		} else {
			elevator.set(ControlMode.PercentOutput, periodicIO.demand);
		}
	}

	@Override
	public void stop() {
		setOpenLoop(0.0);
	}

	@Override
	public void zeroSensors() {
		// master.setSelectedSensorPosition(0, 0, 10);
		resetToAbsolutePosition();
	}

	// @Override
	// public void registerEnabledLoops(ILooper enabledLooper) {
	// enabledLooper.register(loop);
	// }

	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber("Elevator Height", getHeight());
		if (Robot.bot.kDebuggingOutput) {
			SmartDashboard.putNumber("Elevator Current", periodicIO.current);
			SmartDashboard.putNumber("Elevator Voltage", periodicIO.voltage);
			SmartDashboard.putNumber("Elevator Height Graph", getHeight());
			// SmartDashboard.putNumber("Elevator Pulse Width Position",
			// elevator.getSensorCollection().getPulseWidthPosition());
			SmartDashboard.putNumber("Elevator Encoder", periodicIO.position);
			SmartDashboard.putNumber("Elevator Velocity", periodicIO.velocity);
			SmartDashboard.putNumber("Elevator Error", elevator.getClosedLoopError());
			if (elevator.getControlMode() == ControlMode.SmartMotion)
				SmartDashboard.putNumber("Elevator Setpoint", elevator.getLastSet());
		}
	}

	public static class PeriodicIO {
		// Inputs
		public double position = 0;
		public double velocity = 0.0;
		public double voltage = 0.0;
		public double current = 0.0;

		// outputs
		public double demand;
	}
}
