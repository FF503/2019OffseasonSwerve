package com.team503.lib.geometry;

public class Pose {
	private double timestamp = 0;
	private double theta = 0;
	public Translation2d translation = new Translation2d();

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
		this(0, translation2d, theta);
	}

	public Pose(double timestamp, Translation2d translation2d, double theta) {
		this.timestamp = timestamp;
		this.translation = translation2d;
		this.theta = theta;
	}

	public Pose(double[] coord) {
		this(coord[0], coord[1], 90);
	}

	public void update(double timestamp, Translation2d translation2d, double theta) {
		this.timestamp = timestamp;
		this.translation = translation2d;
		this.theta = theta;
	}

	public void update(Translation2d translation2d, double theta) {
		this.translation = translation2d;
		this.theta = theta;
	}

	public void setTimestamp(double timestamp) {
		this.timestamp = timestamp;
	}

	public void updateTheta(double theta) {
		this.theta = theta;
	}

	public double getX() {
		return translation.getX();
	}

	public double getY() {
		return translation.getY();
	}

	public double getTheta() {
		return theta;
	}

	public double getTimestamp() {
		return timestamp;
	}

	public Translation2d toNewVector() {
		return (new Translation2d(getX(), getY()));
	}

	public Pose copy() {
		return new Pose(this.translation.getX(), this.translation.getY(), theta);
	}

	public String toString() {
		return "time: " + timestamp + "x:" + translation.getX() + " y: " + translation.getY() + " theta: " + theta;
	}

	/**
	 * Returns the other pose relative to the current pose.
	 *
	 * <p>
	 * This function can often be used for trajectory tracking or pose stabilization
	 * algorithms to get the error between the reference and the current pose.
	 *
	 * @param other The pose that is the origin of the new coordinate frame that the
	 *              current pose will be converted into.
	 * @return The current pose relative to the new origin pose.
	 */
	public Pose relativeTo(Pose other) {
		var transform = new Transform2d(other, this);
		return new Pose(transform.getTranslation(), transform.getRotation().getDegrees());
	}

	/**
	 * Obtain a new Pose2d from a (constant curvature) velocity.
	 *
	 * <p>
	 * See <a href="https://file.tavsys.net/control/state-space-guide.pdf"> Controls
	 * Engineering in the FIRST Robotics Competition</a> section on nonlinear pose
	 * estimation for derivation.
	 *
	 * <p>
	 * The twist is a change in pose in the robot's coordinate frame since the
	 * previous pose update. When the user runs exp() on the previous known
	 * field-relative pose with the argument being the twist, the user will receive
	 * the new field-relative pose.
	 *
	 * <p>
	 * "Exp" represents the pose exponential, which is solving a differential
	 * equation moving the pose forward in time.
	 *
	 * @param twist The change in pose in the robot's coordinate frame since the
	 *              previous pose update. For example, if a non-holonomic robot
	 *              moves forward 0.01 meters and changes angle by .5 degrees since
	 *              the previous pose update, the twist would be Twist2d{0.01, 0.0,
	 *              toRadians(0.5)}
	 * @return The new pose of the robot.
	 */
	public Transform2d exp(Twist2d twist) {
		double dx = twist.dx;
		double dy = twist.dy;
		double dtheta = twist.dtheta;

		double sinTheta = Math.sin(dtheta);
		double cosTheta = Math.cos(dtheta);

		double s;
		double c;
		if (Math.abs(dtheta) < 1E-9) {
			s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
			c = 0.5 * dtheta;
		} else {
			s = sinTheta / dtheta;
			c = (1 - cosTheta) / dtheta;
		}
		var transform = new Transform2d(new Translation2d(dx * s - dy * c, dx * c + dy * s),
				new Rotation2d(cosTheta, sinTheta));

		return (transform); // plus(transform);
	}

	/**
	 * Transforms the pose by the given transformation and returns the new
	 * transformed pose.
	 *
	 * <p>
	 * The matrix multiplication is as follows [x_new] [cos, -sin, 0][transform.x]
	 * [y_new] += [sin, cos, 0][transform.y] [t_new] [0, 0, 1][transform.t]
	 *
	 * @param other The transform to transform the pose by.
	 * @return The transformed pose.
	 */
	public Pose plus(Transform2d other) {
		return transformBy(other);
	}

	/**
	 * Transforms the pose by the given transformation and returns the new pose. See
	 * + operator for the matrix multiplication performed.
	 *
	 * @param other The transform to transform the pose by.
	 * @return The transformed pose.
	 */
	public Pose transformBy(Transform2d other) {
		return new Pose(getTranslation().plus(other.getTranslation().rotateBy(getRotation())),
				getRotation().plus(other.getRotation()).getDegrees());
	}

	/**
	 * Returns the translation component of the transformation.
	 *
	 * @return The translational component of the pose.
	 */
	public Translation2d getTranslation() {
		return translation;
	}

	/**
	 * Returns the rotational component of the transformation.
	 *
	 * @return The rotational component of the pose.
	 */
	public Rotation2d getRotation() {
		return Rotation2d.fromDegrees(theta);
	}

	/**
	 * @return Translated Pose into different coordinated system used in other
	 *         calculations
	 */
	public Pose getTranslatedPose() {
		Translation2d modifiedTranslation = new Translation2d(getY(), -getX());
		return new Pose(this.timestamp, modifiedTranslation, 90.0 + this.theta);
	}

}
