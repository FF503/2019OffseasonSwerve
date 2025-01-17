/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.auton.FroggyAuton.StartingDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Pigeon extends Subsystem {
	private static Pigeon instance = null;

	public static Pigeon getInstance() {
		if (instance == null) {
			instance = new Pigeon();
		}
		return instance;
	}

	private PigeonIMU pigeon;

	private Pigeon() {
		try {
			// pigeon can id = 22
			// pigeon = new PigeonIMU(BallIntake.getInstance().getPigeonTalon());
			pigeon = new PigeonIMU(22/* Ports.PIGEON_ID */);
		} catch (Exception e) {
			// System.out.println(e);
		}
	}

	public boolean isGood() {
		return (pigeon.getState() == PigeonState.Ready);
	}

	public double getYaw() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		double heading = boundTo360(Robot.bot.requestPigeonFlipped * pigeon.getFusedHeading(fusionStatus));
		SmartDashboard.putNumber("Pigeon Heading", heading);
		return heading/*-ypr[0]*/;
	}

	public double getUnitCircleHeading() {
		return 90 - getYaw();
	}

	public double getPitch() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[1];
	}

	public double getRoll() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr[2];
	}

	public double[] getYPR() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return ypr;
	}

	private void zero() {
		setAngle(0);
	}

	public void reset() {
		setAngle(RobotState.getInstance().getGyroOffset());
	}

	public void setAngle(double angle) {
		pigeon.setFusedHeading(-angle * 64.0, 10);
		pigeon.setYaw(-angle, 10);
		// .println("Pigeon angle set to: " + angle);
	}

	private double boundTo360(double angle_degrees) {
		while (angle_degrees >= 360.0)
			angle_degrees -= 360.0;
		while (angle_degrees < 0.0)
			angle_degrees += 360.0;
		return angle_degrees;
	}

	private void outputToSmartDashboard() {
		SmartDashboard.putString("Pigeon Good", pigeon.getState().toString());
		getYaw();
	}

	@Override
	public void outputTelemetry() {
		outputToSmartDashboard();
	}

	@Override
	public void zeroSensors() {
		zero();
	}

	@Override
	public void stop() {
	}
}
