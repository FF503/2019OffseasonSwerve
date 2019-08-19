package com.team503.lib.controllers;

import com.team503.lib.geometry.Pose;
import com.team503.lib.geometry.Translation2d;
import com.team503.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import motionProfiling.Trajectory;
import motionProfiling.Trajectory.Segment;

public class PurePursuitController {
    private final Trajectory traj;
    private final Lookahead mLookahead;
    private int theoreticalSegmentIndex = 0, lookAheadIndex = 0, closestSegmentIndex = 0;

    private boolean isReversed = false;
    private Translation2d lookaheadPoint;
    private double lookaheadDistance;

    private Pose pose;
    private Pose lastPose = new Pose(Timer.getFPGATimestamp(), 0, 0, 0.0);

    public PurePursuitController(Trajectory traj, Lookahead lookahead) {
        this.traj = traj;
        this.mLookahead = lookahead;
    }

    public PurePursuitController(Trajectory traj) {
        this.traj = traj;
        this.mLookahead = Robot.bot.getLookahead();
    }

    public Translation2d calculateDriveVector(Pose robotPose) {
        this.pose = robotPose;
        Segment closest = getClosestSegment(robotPose);
        this.lookaheadDistance = getLookaheadDistance(robotPose.toVector(), closest, mLookahead);
        this.lookaheadPoint = getLookAhead(robotPose, lookaheadDistance);
        Translation2d robotToLookahead = new Translation2d(lookaheadPoint).minus(robotPose.toVector());

        Translation2d velocityVector = scaleVectorToDesiredVelocity(robotToLookahead, closest.vel);
        lastPose = robotPose.copy();
        return applyFeedForward(velocityVector);
    }

    private double getLookaheadDistance(Translation2d robot, Segment closest, Lookahead lookahead) {
        double closestPointDistance = new Translation2d(closest.x, closest.y).minus(robot).getNorm();
        double lookahead_distance = lookahead.getSpeedBasedLookahead(closest.vel) + closestPointDistance;
        return lookahead_distance;
    }

    private Translation2d getLookAhead(Pose robotPose, double lookaheadDistance) {
        double tValue = 0;
        Translation2d startSeg = new Translation2d(traj.getSegment(lookAheadIndex).x,
                traj.getSegment(lookAheadIndex).y);
        Translation2d dSegmentVector = new Translation2d(0, 0);
        for (int i = closestSegmentIndex; i < traj.getNumSegments() - 1; i++) {
            Translation2d curPathSeg = new Translation2d(traj.getSegment(i).x, traj.getSegment(i).y);
            Translation2d nextPathSeg = new Translation2d(traj.getSegment(i + 1).x, traj.getSegment(i + 1).y);
            Translation2d curToNextSegment = new Translation2d(nextPathSeg.getX() - curPathSeg.getX(),
                    nextPathSeg.getY() - curPathSeg.getY());
            Translation2d f = new Translation2d(curPathSeg.getX() - robotPose.getX(),
                    curPathSeg.getY() - robotPose.getY());

            double a = Translation2d.dot(curToNextSegment, curToNextSegment);
            double b = 2 * Translation2d.dot(f, curToNextSegment);
            double c = Translation2d.dot(f, f) - lookaheadDistance * lookaheadDistance;
            double discriminant = b * b - 4 * a * c;

            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);

                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

                if (t1 >= 0 && t1 <= 1) {
                    tValue = t1;
                    startSeg = curPathSeg;
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
        dSegmentVector.times(tValue);
        lookAhead.plus(dSegmentVector);

        Segment finalSegment = traj.getSegment(traj.getNumSegments() - 1);
        Translation2d finalSegmentAsTranslation = new Translation2d(finalSegment.x, finalSegment.y);
        return lookAhead;
    }

    public Translation2d getCalculatedLookahead() {
        return lookaheadPoint;
    }

    private Segment getClosestSegment(Pose pose) {
        Segment closestSeg = traj.getSegment(closestSegmentIndex);
        for (int i = closestSegmentIndex; i < this.traj.getNumSegments(); i++) {
            Segment curSegment = traj.getSegment(i);
            double poseToPrevHypot = Math.hypot(pose.getX() - closestSeg.x, pose.getY() - closestSeg.y);
            double poseToCurrHypot = Math.hypot(pose.getX() - curSegment.x, pose.getY() - curSegment.y);
            if (poseToCurrHypot < poseToPrevHypot) {
                closestSegmentIndex = i;
                closestSeg = curSegment;
            }
        }
        return closestSeg;
    }

    private Translation2d scaleVectorToDesiredVelocity(Translation2d vector, double desiredVelocity) {
        double mag = vector.getNorm();
        return vector.times(desiredVelocity / mag);
    }

    private Translation2d applyFeedForward(Translation2d velocityVector) {
        return velocityVector.times(Robot.bot.kPurePursuitV);
    }

    private Translation2d applyFeedback(Pose robotPose, Translation2d targetVector) {
        // double currentVelocity =
        // (robotPose.toVector().minus(lastPose.toVector()).getNorm())
        // / (robotPose.getTimestamp() - lastPose.getTimestamp());

        // double targetVelocity = targetVector.getNorm();
        // double error = targetVelocity - currentVelocity;
        // double output = error * Robot.bot.kP_PurePursuit;

        Translation2d currentVelocity = robotPose.toVector().minus(lastPose.toVector())
                .div((robotPose.getTimestamp() - lastPose.getTimestamp()));
        Translation2d error = targetVector.minus(currentVelocity);
        Translation2d output = error.times(Robot.bot.kPurePursuitP);
        return output;
    }

    public boolean isDone() {
        return getSegmentIndex() == traj.getNumSegments() - 1;
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
        return mLookahead.getSpeedBasedLookahead(seg.vel);
    }

    public static class Lookahead {
        public final double min_distance;
        public final double max_distance;
        public final double min_speed;
        public final double max_speed;

        protected final double delta_distance;
        protected final double delta_speed;

        public Lookahead(double min_distance, double max_distance, double min_speed, double max_speed) {
            this.min_distance = min_distance;
            this.max_distance = max_distance;
            this.min_speed = min_speed;
            this.max_speed = max_speed;
            delta_distance = max_distance - min_distance;
            delta_speed = max_speed - min_speed;
        }

        public double getSpeedBasedLookahead(double speed) {
            double lookahead = delta_distance * (speed - min_speed) / delta_speed + min_distance;
            return Double.isNaN(lookahead) ? min_distance : Math.max(min_distance, Math.min(max_distance, lookahead));
        }
    }
}