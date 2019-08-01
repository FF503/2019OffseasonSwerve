package com.team503.lib.util;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Pose {
	private double timestamp = 0;
	private double theta = 503.503503503, lastTheta = 503.503503503;
	public Translation2d translation = new Translation2d(), lastTranslation = new Translation2d();

	public Pose(double x, double y, double theta) {
		this.translation = new Translation2d(x, y);
		this.theta = theta;
	}

	public Pose(double timestamp, double x, double y, double theta) {
		this.translation = new Translation2d(x, y);
		this.theta = theta;
		setTimestamp(timestamp);
	}

	public Pose(Translation2d translation2d, double theta) {
		this.translation = translation2d;
		this.theta = theta;
	}

	public Pose(double[] coord) {
		this(coord[0], coord[1], 0);
	}

	public double getX() {
		return translation.x();
	}

	public double getY() {
		return translation.y();
	}

	public double getTheta() {
		return theta;
	}

	public void update(double x, double y, double theta) {
		this.translation.setX(x);
		this.translation.setY(y);
		this.theta = theta;
	}

	public void increment(double x, double y, double theta) {
		this.translation.translateBy(new Translation2d(x, y));
		// this.x += x;
		// this.y += y;
		this.theta += theta;
	}

	public double[] get() {
		return new double[] { getX(), getY(), theta };
	}

	public double getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

	public Pose2d toPose2D() {
		return new Pose2d(translation, Rotation2d.fromDegrees(theta));
	}

	public void translatePose(Translation2d translation) {
		// this.x += Math.cos(Math.toRadians(this.theta))*centerOfRotDist;
		// this.y += Math.sin(Math.toRadians(this.theta))*centerOfRotDist;
		double deltaX = translation.x();
		double deltaY = translation.y();
		double x = this.getX();
		double y = this.getY();
		double alpha = this.getTheta();
		double cos = (Math.cos(Math.toRadians(alpha)));
		double sin = (Math.sin(Math.toRadians(alpha)));
		double x1 = x + (cos * deltaX) - (sin * deltaX);
		// Math.sin(Math.toRadians(alpha))*deltaX +
		// Math.cos(Math.toRadians(alpha))*deltaY;
		double sign = Math.signum(deltaX * deltaX + deltaY * deltaY - (x1 * x1));
		double y1 = y + (sin * deltaX) + (cos * deltaX);// sign*Math.sqrt(Math.abs(deltaX*deltaX + deltaY*deltaY -
														// (x1*x1))) + y;// x + Math.sin(Math.toRadians(alpha))*deltaY +
		// Math.cos(Math.toRadians(alpha))*deltaX;//Math.sqrt(deltaX*deltaX
		// + deltaY*deltaY - (x1*x1)) + y;
		this.translation.setX(x1);
		this.translation.setY(y1);
		this.theta = alpha;
	}

	public void updateTheta(double theta) {
		lastTheta = this.theta;
		this.theta = theta;
	}

	public double getLastTheta() {
		return lastTheta;
	}

	public Translation2d toVector() {
		return (new Translation2d(translation.x(), translation.y()));
	}

	public Pose copy() {
		return new Pose(this.translation.x(), this.translation.y(), theta);
	}

	public String toString() {
		return "x:" + translation.x() + " y: " + translation.y() + " theta: " + theta;
	}

}
