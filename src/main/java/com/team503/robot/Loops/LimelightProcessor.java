package com.team503.robot.loops;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightProcessor {
	private static LimelightProcessor instance = new LimelightProcessor();
	private edu.wpi.first.networktables.NetworkTable table;
	private NetworkTableEntry ledMode;
	private NetworkTableEntry pipeline;
	private NetworkTableEntry camMode;
	private NetworkTableEntry stream;
	private NetworkTableEntry ct;
	private List<NetworkTableEntry> target1, target2, combinedTarget;
	private NetworkTableEntry cornerX, cornerY;

	public static LimelightProcessor getInstance() {
		return instance == null ? instance = new LimelightProcessor() : instance;
	}

	public LimelightProcessor() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		ledMode = table.getEntry("ledMode");
		pipeline = table.getEntry("pipeline");
		camMode = table.getEntry("camMode");
		ct = table.getEntry("camtran");
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

	//TODO: do this properly
	public boolean hasReachedAreaThreshold() {
		return getTA() > 13;
	}

	public boolean seesTarget() {
		return getTV() == 1.0;
	}

	public enum Pipeline {
		DRIVER(0), LEFTMOST(0), RIGHTMOST(0), CLOSEST(2), LOWEST(0), HIGHEST(0);

		int id;

		private Pipeline(int id) {
			this.id = id;
		}
	}

}
