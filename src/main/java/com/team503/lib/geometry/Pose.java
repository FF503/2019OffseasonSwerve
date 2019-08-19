package com.team503.lib.geometry;

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
		return translation.getX();
	}

	public double getY() {
		return translation.getY();
	}

	public double getTheta() {
		return theta;
	}

	public void update(double x, double y, double theta) {
		this.translation = new Translation2d(x, y);
		this.theta = theta;
	}

	public void increment(double x, double y, double theta) {
		this.translation.plus(new Translation2d(x, y));
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

	public void translatePose(Translation2d translation) {
		// this.x += Math.cos(Math.toRadians(this.theta))*centerOfRotDist;
		// this.y += Math.sin(Math.toRadians(this.theta))*centerOfRotDist;
		double deltaX = translation.getX();
		double deltaY = translation.getY();
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
		this.translation = new Translation2d(x1, y1);
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
		return (new Translation2d(translation.getX(), translation.getY()));
	}

	public Pose copy() {
		return new Pose(this.translation.getX(), this.translation.getY(), theta);
	}

	public String toString() {
		return "time: " + timestamp + "x:" + translation.getX() + " y: " + translation.getY() + " theta: " + theta;
	}

	public Pose exp(Twist2d twist) {
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

		return this.plus(transform);
	}

	public Pose plus(Transform2d other) {
		return transformBy(other);
	}

	public Pose transformBy(Transform2d other) {
		Rotation2d m_rotation = Rotation2d.fromDegrees(theta);
		return new Pose(translation.plus(other.getTranslation().rotateBy(m_rotation)),
				m_rotation.plus(other.getRotation()).getDegrees());
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

}
