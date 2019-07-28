package com.team503.robot.Loops;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team503.lib.util.Pose;
import com.team503.lib.util.Util;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.subsystems.SwerveDrive;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

public class PoseController {
    private Pose currentPose;
    private double lastPoseUpdate = -1;
    private Notifier threadRunner;
    private boolean run = false;
    private PoseThread poseThread = new PoseThread();

    public PoseController() {
        threadRunner.setHandler(poseThread);
        threadRunner.startPeriodic(Robot.bot.POSE_LOOP_DT);
    }

    public synchronized void start() {
        run = true;
    }

    public synchronized void stop() {
        // threadRunner.stop();
        run = false;
    }

    public synchronized void updatePoseWithVelocity(Translation2d velocity) {
        double dt = Timer.getFPGATimestamp() - lastPoseUpdate;
        Translation2d positionChange = lastPoseUpdate == -1 ? velocity.scale(Robot.bot.POSE_LOOP_DT)
                : velocity.scale(dt);
        currentPose = currentPose.toPose2D().transformBy(Pose2d.fromTranslation(positionChange)).toPose();
        RobotState.getInstance().setCurrentPose(currentPose);
        lastPoseUpdate = Timer.getFPGATimestamp();
    }

    private class PoseThread implements Runnable {
        @Override
        public void run() {
            if (run) {
                SimpleMatrix wheelVelocities = new SimpleMatrix(
                        SwerveDrive.getInstance().getWheelComponentVelocities());
                Translation2d velocity = Util.getVelocity(wheelVelocities);
                updatePoseWithVelocity(velocity);
            }
        }

    }

}