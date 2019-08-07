package com.team503.robot;

import com.team503.lib.util.FFDashboard;
import com.team503.lib.util.Pose;

public class RobotState {
	private Bot currentRobot;
	private Pose currentPose;
	private double currentTheta;
	private static RobotState instance = new RobotState();

	private GameElement gameElement;
	private ArmDirection armDirection;
	private TargetHeight targetHeight;
	private boolean hasElement;
	private boolean hatchDependence;
	private SuperStructurePreset sPreset;
	private boolean armPositionChanged;
	private boolean isArmFlip;
	private boolean grabberDeployed;
	private boolean isManualControl = true;

	public static RobotState getInstance() {
		return instance;
	}

	public Pose getCurrentPose() {
		return currentPose;
	}

	public void setCurrentPose(Pose currentPose) {
		this.currentPose = currentPose;
	}

	public synchronized double getCurrentTheta() {
		return currentTheta;
	}

	public synchronized void setCurrentTheta(double currentTheta) {
		this.currentTheta = currentTheta;
	}

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
		FRONT_CARGO_BUS(48.2, -21.8, 0.), BACK_CARGO_BUS(111, 53. + 180., 0.), FRONT_CARGO_INTAKE(-38, -14. + 3.5, 0.),
		FRONT_CARGO_LOW(0., -3., 0.), FRONT_CARGO_MID(69., -5., 0.), // 0., -3., 0.
		FRONT_CARGO_HIGH(84.0, 48.0, 12.7), BACK_CARGO_INTAKE(222, 183, 0.), BACK_CARGO_LOW(170., 180., 0.),
		BACK_CARGO_MID(92., 180., 0.), BACK_CARGO_HIGH(90.6, 126.5, 12.7), FRONT_HATCH_INTAKE(-32, 85.0, 6.),
		FRONT_HATCH_LOW(-32, 73, 6.), FRONT_HATCH_MID(17.0 /* 13 */, 77. /* 90 */, 0.),
		FRONT_HATCH_HIGH(59., 87.0, 12.3), BACK_HATCH_INTAKE(187., 90. + 180., 0.), BACK_HATCH_LOW(187., 255., 0.),
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
		System.out.println("SETTING TARGET");
		FFDashboard.getInstance().putString("Arm Level", height.toString());
	}

	public void setPositionChanged(boolean changed) {
		armPositionChanged = changed;
	}

	public boolean getPositionChanged() {
		return armPositionChanged;
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
    
    public static enum Bot {
        Automatic, ProgrammingBot;
    }


}