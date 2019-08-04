package com.team503.robot.Loops;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team503.robot.RobotState;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightProcessor {
	static LimelightProcessor instance = new LimelightProcessor();
	edu.wpi.first.networktables.NetworkTable table;
	RobotState robotState = RobotState.getInstance();
	NetworkTableEntry ledMode;
	NetworkTableEntry pipeline;
	NetworkTableEntry camMode;
	NetworkTableEntry stream;
	public List<NetworkTableEntry> target1, target2, combinedTarget;
	public NetworkTableEntry cornerX, cornerY;

	boolean updatesAllowed = true;

	public void enableUpdates(boolean enable) {
		updatesAllowed = enable;
	}

	public static LimelightProcessor getInstance() {
		return instance;
	}

	public LimelightProcessor() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		ledMode = table.getEntry("ledMode");
		pipeline = table.getEntry("pipeline");
		camMode = table.getEntry("camMode");
		stream = table.getEntry("stream");
		// setStreamMode(2);// 0
		target1 = Arrays.asList(table.getEntry("tx0"), table.getEntry("ty0"), table.getEntry("ta0"));
		target2 = Arrays.asList(table.getEntry("tx1"), table.getEntry("ty1"), table.getEntry("ta1"));
		combinedTarget = Arrays.asList(table.getEntry("tx"), table.getEntry("ty"), table.getEntry("ta"),
				table.getEntry("tv"));
		cornerX = table.getEntry("tcornx");
		cornerY = table.getEntry("tcorny");
		setPipeline(Pipeline.CLOSEST);
	}

	public void blink() {
		if (ledMode.getDouble(0) != 2)
			ledMode.setNumber(2);
	}

	public void ledOn(boolean on) {
		if (ledMode.getDouble(1) != 0 && on)
			ledMode.setNumber(0);
		else if (ledMode.getDouble(0) != 1 && !on)
			ledMode.setNumber(1);
	}

	public void setDriverMode() {
		camMode.setNumber(1);
	}

	public void setVisionMode() {
		camMode.setNumber(0);
	}

	public void setStreamMode(int id) {
		stream.setNumber(id);
	}

	public void setPipeline(int id) {
		pipeline.setNumber(id);
	}

	public void setPipeline(Pipeline p) {
		setPipeline(p.id);
		System.out.println("Pipeline set to " + p.id);
	}

	public double getPipelineNumber() {
		return pipeline.getDouble(0.0);
	}

	public double getTX() {
		return combinedTarget.get(0).getDouble(0.0);
	}

	public double getTY() {
		return combinedTarget.get(1).getDouble(0.0);
	}

	public double getTA() {
		return combinedTarget.get(2).getDouble(0.0);
	}

	public double getTV() {
		return (combinedTarget.get(3).getDouble(0.0));
	}

	public enum Pipeline {
		LEFTMOST(0), RIGHTMOST(1), CLOSEST(2), LOWEST(3), HIGHEST(4);

		int id;

		private Pipeline(int id) {
			this.id = id;
		}
	}

	public boolean seesTarget() {
		boolean targetInSight = (getTV() == 1.0) ? true : false;
		return targetInSight;
	}

}
