package com.team503.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;


import com.frcteam2910.common.control.Trajectory;
import com.frcteam2910.common.math.RigidTransform2;
import com.frcteam2910.common.math.Rotation2;
import com.frcteam2910.common.math.Vector2;
import com.frcteam2910.common.util.HolonomicDriveSignal;
import com.team503.lib.geometry.Pose;
import com.team503.lib.geometry.Translation2d;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;
import com.team503.robot.subsystems.SwerveDrive;

import java.util.Optional;
import java.util.function.Supplier;

public class JackInBotFollowTrajectoryCommand extends Command {
    private final Supplier<Trajectory> trajectorySupplier;

    private Trajectory trajectory;
    private double targetTheta;
    public JackInBotFollowTrajectoryCommand(Trajectory trajectory, double targetTheta) {
        this(() -> trajectory);
        this.targetTheta = targetTheta;
    }

    public JackInBotFollowTrajectoryCommand(Supplier<Trajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;
        this.setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        trajectory = trajectorySupplier.get();
        SwerveDrive.getInstance().getFollower().follow(trajectory);
    }

    @Override
    protected void end() {
        SwerveDrive.getInstance().stop();
    }

    @Override
    protected void execute() {
        // terribly ineffiecent way to bridge between two different math libs. Should
        // update this later.
        SwerveDrive.getInstance().rotate(targetTheta);
        Pose frogPose = RobotState.getInstance().getCurrentPose();
        Vector2 robotTranslation = new Vector2(frogPose.getX(), -frogPose.getY());
        Rotation2 robotRotation = Rotation2.fromDegrees(frogPose.getTheta());
        RigidTransform2 pose = new RigidTransform2(robotTranslation, robotRotation);
        // velocity and rotational velocity are irrelevant for pure pursuit calcs I
        // think (from reading the code)
        Optional<HolonomicDriveSignal> signal = SwerveDrive.getInstance().getFollower().update(pose,
                new Vector2(0.0, 0.0), 0.0, Timer.getFPGATimestamp(), 0.05);
        Vector2 translation = signal.get().getTranslation();
        Translation2d frogTranslation = new Translation2d(translation.y, translation.x);
        System.out.println(signal.get().getTranslation().x);
        SwerveDrive.getInstance().drive(frogTranslation, Math.min(Math.max(SwerveDrive.getInstance().getRotationalOutput(), -0.5),0.5));
    }

    @Override
    protected void interrupted() {
        end();
        SwerveDrive.getInstance().getFollower().cancel();
    }

    @Override
    protected boolean isFinished() {
        // Only finish when the trajectory is completed
        return SwerveDrive.getInstance().getFollower().getCurrentTrajectory().isEmpty();
    }
}