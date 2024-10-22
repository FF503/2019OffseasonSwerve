package com.team503.robot.loops;

import com.team503.lib.geometry.Pose;
import com.team503.lib.geometry.Rotation2d;
import com.team503.lib.geometry.Transform2d;
import com.team503.lib.geometry.Translation2d;
import com.team503.lib.util.Util;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.subsystems.Pigeon;
import com.team503.robot.subsystems.SwerveDrive;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseController {
    private Pose currentPose;
    private double lastPoseUpdate = -1;
    private Notifier threadRunner;
    private boolean run = false;
    private PoseThread poseThread = new PoseThread();

    private static PoseController instance;

    public static PoseController getInstance() {
        return instance == null ? instance = new PoseController() : instance;
    }

    public PoseController() {
        threadRunner.setHandler(poseThread);
    }

    public synchronized void start() {
        threadRunner.startPeriodic(Robot.bot.POSE_LOOP_DT);
        run = true;
    }

    public synchronized void stop() {
        // threadRunner.stop();
        run = false;
    }

    public synchronized void resetPose(Pose reset) {
        this.currentPose = reset;
    }

    public synchronized void updatePoseWithVelocity(Translation2d velocity) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastPoseUpdate;
        Translation2d positionChange = lastPoseUpdate == -1 ? velocity.times(Robot.bot.POSE_LOOP_DT)
                : velocity.times(dt);
        currentPose = currentPose.transformBy(new Transform2d(positionChange, new Rotation2d()));
        currentPose.setTimestamp(currentTime);
        currentPose.updateTheta(Pigeon.getInstance().getUnitCircleHeading());
        RobotState.getInstance().setCurrentPose(currentPose);
        lastPoseUpdate = currentTime;
    }

    private class PoseThread implements Runnable {
        @Override
        public void run() {
            if (run) {
                SimpleMatrix wheelVelocities = new SimpleMatrix(
                        SwerveDrive.getInstance().getWheelComponentVelocities());
                Translation2d velocity = Util.getVelocity(wheelVelocities);
                SmartDashboard.putNumber("Trans Velocity", velocity.getNorm());
                updatePoseWithVelocity(velocity);
            }
        }

    }

}