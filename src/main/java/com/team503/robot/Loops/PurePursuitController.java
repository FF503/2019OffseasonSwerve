package com.team503.robot.Loops;

import com.team254.lib.geometry.Translation2d;
import com.team503.lib.util.Pose;
import com.team503.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import motionProfiling.Trajectory;
import motionProfiling.Trajectory.Segment;

public class PurePursuitController {
    private Trajectory traj;
    private double lookAheadDist;
    private double minLookaheadDistance;
    private int theoreticalSegmentIndex = 0, lookAheadIndex = 0, closestSegmentIndex = 0;
    private boolean isReversed = false;
    private Translation2d lookaheadPoint;
    private Pose pose;
    private Pose lastPose = new Pose(Timer.getFPGATimestamp(), 0, 0, 0.0);

    public PurePursuitController(Trajectory traj, double lookAheadDist) {
        this.traj = traj;
        this.minLookaheadDistance = lookAheadDist;
        this.lookAheadDist = lookAheadDist;
    }

    public Translation2d calculateDriveVector(Pose robotPose) {
        this.lookaheadPoint = getLookAhead(robotPose);
        Translation2d robotToLookahead = new Translation2d(lookaheadPoint).translateBy(robotPose.toVector().inverse());

        Translation2d velocityVector = scaleVectorToDesiredVelocity(robotToLookahead, getClosestSegment().vel);
        lastPose = robotPose.copy();
        return applyFeedForward(velocityVector);
    }

    private Translation2d getLookAhead(Pose robotPose) {
        double tValue = 0;
        Translation2d startSeg = new Translation2d(traj.getSegment(lookAheadIndex).x,
                traj.getSegment(lookAheadIndex).y);
        Translation2d dSegmentVector = new Translation2d(0, 0);
        for (int i = closestSegmentIndex; i < traj.getNumSegments() - 1; i++) {
            Translation2d curPathSeg = new Translation2d(traj.getSegment(i).x, traj.getSegment(i).y);
            Translation2d nextPathSeg = new Translation2d(traj.getSegment(i + 1).x, traj.getSegment(i + 1).y);
            Translation2d curToNextSegment = new Translation2d(nextPathSeg.x() - curPathSeg.x(),
                    nextPathSeg.y() - curPathSeg.y());
            Translation2d f = new Translation2d(curPathSeg.x() - robotPose.getX(), curPathSeg.y() - robotPose.getY());

            double a = Translation2d.dot(curToNextSegment, curToNextSegment);
            double b = 2 * Translation2d.dot(f, curToNextSegment);
            double c = Translation2d.dot(f, f) - lookAheadDist * lookAheadDist;
            double discriminant = b * b - 4 * a * c;

            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);

                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

                if (t1 >= 0 && t1 <= 1) {
                    tValue = t1;
                    startSeg = curPathSeg.getTranslation();
                    dSegmentVector = new Translation2d(curToNextSegment);
                    lookAheadIndex = i;
                    break;
                } else if (t2 >= 0 && t2 <= 1) {
                    tValue = t2;
                    startSeg = new Translation2d(curPathSeg);
                    dSegmentVector = new Translation2d(curToNextSegment);
                    lookAheadIndex = i;
                    break;
                }

            }
        }
        Translation2d lookAhead = new Translation2d(startSeg);
        dSegmentVector.scale(tValue);
        lookAhead.translateBy(dSegmentVector);

        Segment finalSegment = traj.getSegment(traj.getNumSegments() - 1);
        Translation2d finalSegmentAsTranslation = new Translation2d(finalSegment.x, finalSegment.y);
        return lookAhead;
    }

    public Translation2d getCalculatedLookahead() {
        return lookaheadPoint;
    }

    private Segment getClosestSegment() {
        Segment closestSeg = traj.getSegment(closestSegmentIndex);
        for (int i = closestSegmentIndex; i < this.traj.getNumSegments(); i++) {
            Segment curSegment = traj.getSegment(i);
            double poseToPrevHypot = Math.hypot(this.pose.getX() - closestSeg.x, this.pose.getY() - closestSeg.y);
            double poseToCurrHypot = Math.hypot(this.pose.getX() - curSegment.x, this.pose.getY() - curSegment.y);
            if (poseToCurrHypot < poseToPrevHypot) {
                closestSegmentIndex = i;
                closestSeg = curSegment;
            }
        }
        return closestSeg;
    }

    private Translation2d scaleVectorToDesiredVelocity(Translation2d vector, double desiredVelocity) {
        double mag = vector.norm();
        return vector.scale(desiredVelocity / mag);
    }

    private Translation2d applyFeedForward(Translation2d velocityVector) {
        return velocityVector.scale(Robot.bot.kV_PurePursuit);
    }

    private void applyFeedback(Pose robotPose, Translation2d targetVector) {
        double currentVelocity = (robotPose.toVector().translateBy(lastPose.toVector().inverse()).norm())
                / (robotPose.getTimestamp() - lastPose.getTimestamp());

        double targetVelocity = targetVector.norm();
        double error = targetVelocity - currentVelocity;
        double output = error * Robot.bot.kP_PurePursuit;
    }

    public boolean isDone() {
        return getSegmentIndex() == traj.getNumSegments() - 1;
    }

    private double lastError = 0.0;

    public void setIsReversed(boolean rev) {
        isReversed = rev;
    }

    public Segment getSegment() {
        return traj.getSegment(getSegmentIndex());
    }

    public int getSegmentIndex() {
        return closestSegmentIndex;
    }

    public int getTheoreticalSegmentIndex() {
        return theoreticalSegmentIndex;
    }

    public double getSpeedBasedLookahead(Segment seg) {
        return getSpeedBasedLookahead(seg.vel);
    }

    public double getSpeedBasedLookahead(double speed) {
        // Change Parameters below as nessesary
        double desiredMaxLookahead = isReversed ? 70 : 40;
        final double minLookaheadDistance = this.minLookaheadDistance, maxLookaheadDistance = desiredMaxLookahead;
        final double min_speed = 0, max_speed = Robot.bot.kMaxVelocityInchesPerSec;
        /**********************************************************/

        final double delta_distance = maxLookaheadDistance - minLookaheadDistance;
        final double delta_speed = max_speed - min_speed;
        double lookahead = delta_distance * (speed - min_speed) / delta_speed + minLookaheadDistance;
        return Double.isNaN(lookahead) ? minLookaheadDistance
                : Math.max(minLookaheadDistance, Math.min(maxLookaheadDistance, lookahead));
    }

}