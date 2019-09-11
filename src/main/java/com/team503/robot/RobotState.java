package com.team503.robot;

import com.team254.lib.geometry.Translation2d;
import com.team503.lib.geometry.Pose;
import com.team503.lib.util.FFDashboard;
// import com.team503.robot.auton.FroggyAuton.StartingDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
	private Bot currentRobot;
	private Pose currentPose;
	private double currentTheta;
	private static RobotState instance = new RobotState();

	private GameElement gameElement;
	private ArmDirection armDirection;
	private TargetHeight targetHeight;
	private boolean hasElement;
	private SuperStructurePreset sPreset;
	private boolean armPositionChanged;
	private boolean isArmFlip;
	private boolean grabberDeployed;
	private boolean isManualControl = true;
	private boolean autoDone = false;
	private int pipeline = 2;
	private Translation2d centerOfRotation = new Translation2d();
	// private volatile StartingDirection startingDirection =
	// StartingDirection.FORWARD;

	public static RobotState getInstance() {
		return instance;
	}

	public synchronized Pose getCurrentPose() {
		return currentPose;
	}

	public synchronized void setCurrentPose(Pose currentPose) {
		this.currentPose = currentPose;
	}

	public synchronized double getCurrentTheta() {
		return currentTheta;
	}

	public synchronized void setCurrentTheta(double currentTheta) {
		this.currentTheta = currentTheta;
	}

	// public synchronized void setStartingDirection(StartingDirection
	// startingDirection) {
	// this.startingDirection = startingDirection;
	// }

	// public synchronized double getGyroOffset() {
	// return this.startingDirection.getGyroOffset();
	// }

	public boolean getHasElement() {
		return hasElement;
	}

	public void setHasElement(boolean state) {
		this.hasElement = state;
	}

	public boolean getHatchDependence() {
		return hasElement;
	}

	public void setHatchDependence(boolean state) {
		this.hasElement = state;
	}

	public boolean getIsArmFlip() {
		return isArmFlip;
	}

	public void setIsArmFlip(boolean state) {
		this.isArmFlip = state;
	}

	public boolean getIsManual() {
		return isManualControl;
	}

	public void setIsManual(boolean state) {
		this.isManualControl = state;
	}

	public boolean getGrabberDeployed() {
		return grabberDeployed;
	}

	public void setGrabberDeployed(boolean state) {
		this.grabberDeployed = state;
	}

	public static enum GameElement {
		CARGO, HATCH;
	}

	public static enum ArmDirection {
		FRONT, BACK;
	}

	public static enum TargetHeight {
		HOME, BUS, INTAKE, LOW, MIDDLE, HIGH;
	}

	public static enum SuperStructurePreset {// FRONT_CARGO_INTAKE(-32, -58, 0.)
		HATCH_HOME(-56., 90, 0.), CARGO_HOME(-45, 45, 0), VIEW_AUTO_REAR(187., 90., 0.),
		FRONT_CARGO_BUS(44.5, -27.8, 0.), BACK_CARGO_BUS(111, 53. + 180., 0.), FRONT_CARGO_INTAKE(-48, -15.5, 0.),
		FRONT_CARGO_LOW(-12.0, 2., 0.), FRONT_CARGO_MID(69., -5., 0.), // 0., -3., 0.
		FRONT_CARGO_HIGH(84.0, 48.0, 12.7), BACK_CARGO_INTAKE(222, 183, 0.), BACK_CARGO_LOW(170., 180., 0.),
		BACK_CARGO_MID(94., 173., 0.), BACK_CARGO_HIGH(86., 124., 12.), FRONT_HATCH_INTAKE(-32, 85.0, 6.),
		FRONT_HATCH_LOW(-44, 85, 6.), FRONT_HATCH_MID(8.0 /* 13 */, 83. /* 90 */, 0.),
		FRONT_HATCH_HIGH(59., 88.0, 12.3), BACK_HATCH_INTAKE(187., 90. + 180., 0.), BACK_HATCH_LOW(187., 255., 0.),
		BACK_HATCH_MID(119.5, 90. + 180., 4.), // 180., 87., 0.
		BACK_HATCH_HIGH(95, 90. + 180., 0.);// 175., 77., 0

		double aPos, wPos, ePos;

		private SuperStructurePreset(double arm, double wrist, double extension) {
			this.aPos = arm;
			this.wPos = wrist;
			this.ePos = extension;
		}

		public double getArmPosition() {
			return aPos;
		}

		public double getWristPosition() {
			return wPos;
		}

		public double getExtPosition() {
			return ePos;
		}
	}

	public GameElement getGameElement() {
		return gameElement;
	}

	public void setGameElement(GameElement element) {
		this.gameElement = element;
		if (this.gameElement != GameElement.CARGO) {
			FFDashboard.getInstance().putString("Game Element", "HATCH");
		} else {
			FFDashboard.getInstance().putString("Game Element", gameElement.toString());
		}
	}

	public TargetHeight getTargetHeight() {
		return targetHeight;
	}

	public void setTargetHeight(TargetHeight height) {
		targetHeight = height;
		// System.out.println("SETTING TARGET");
		FFDashboard.getInstance().putString("Arm Level", height.toString());
	}

	public void setPositionChanged(boolean changed) {
		armPositionChanged = changed;
	}

	public boolean getPositionChanged() {
		return armPositionChanged;
	}

	public void setCenterOfRotation(Translation2d center) {
		this.centerOfRotation = center;
	}

	public Translation2d getCenterOfRotation() {
		return centerOfRotation;
	}

	public boolean getAutonDone() {
		return autoDone;
	}

	public void setAutonDone(boolean b) {
		this.autoDone = b;
	}

	public SuperStructurePreset getSuperStructurePreset() {
		return sPreset;
	}

	public void setSuperStructurePreset(SuperStructurePreset sPos) {
		this.sPreset = sPos;
		FFDashboard.getInstance().putString("Selected Preset", sPreset.toString());
	}

	public ArmDirection getArmDirection() {
		return armDirection;
	}

	public void setArmDirection(ArmDirection armDirection) {
		this.armDirection = armDirection;
		FFDashboard.getInstance().putString("Direction", armDirection.toString());
	}

	public void setCurrentRobot(final Bot currentRobot) {
		this.currentRobot = currentRobot;
	}

	public Bot getCurrentRobot() {
		return this.currentRobot;
	}

	public enum Bot {
		Automatic, ProgrammingBot, FFSwerve;
	}

}