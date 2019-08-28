/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team503.robot.auton;

import com.team503.lib.geometry.Pose;
import com.team503.lib.util.FFDashboard;
import com.team503.lib.util.ProfileLoader;
import com.team503.robot.RobotState;
import com.team503.robot.loops.FroggyPoseController;

import edu.wpi.first.wpilibj.command.CommandGroup;

public abstract class FroggyAuton extends CommandGroup {

    protected abstract void initAuton();

    protected abstract AutonStartingLocation getStartingLocation();

    protected StartingDirection getStartingDirection() {
        return StartingDirection.FORWARD;
    }

    protected void init() {
        initAuton();
        initStartingDirection();
    }

    public void startAuton() {
        start();
    }

    @Override
    public synchronized void start() {
        initStartingDirection();
        initStartingLocation(getStartingLocation());
        new FFDashboard("Graph").putBoolean("start", true);
        super.start();
    }

    public void initAndStartAuton() {
        init();
        startAuton();
    }

    private void initStartingLocation(AutonStartingLocation startingLocation) {
        FroggyPoseController.resetPose(startingLocation.getStartingPose());
    }

    private void initStartingDirection() {
        RobotState.getInstance().setStartingDirection(getStartingDirection());
    }

    private ProfileLoader getProfileInfo(String file) {
        ProfileLoader loader = new ProfileLoader();
        loader.storeTrajectory(file);
        return loader;
    }

    public void froggySequentialDrive(String file) {
        ProfileLoader loader = getProfileInfo(file);
        FFDashboard.getInstance().putNumber("Segments", loader.getTrajectory().getNumSegments());
        addSequential(new FollowTrajectoryCommand(loader.getTrajectory()));
    }

    public void froggyParallelDrive(String file) {
        ProfileLoader loader = getProfileInfo(file);
        addParallel(new FollowTrajectoryCommand(loader.getTrajectory()));
    }

    @Override
    public synchronized void cancel() {
        super.cancel();
        new FFDashboard("Graph").putBoolean("start", false);
    }

    protected enum AutonStartingLocation {

        Right(212, 63, 90), RightLevel2(212, 32, 90), Origin(0, 0, 90), tenFeetForward(0, 120, 90),
        FirstCargoBay(208, 255, 180), Left(112, 63, 90), LeftLevel2(112, 32, 90), TEST(301, 15, -90),
        FirstHatch(223.0, 263.0, 0.0), FirstHatchTurned(234.0, 266.0, 90.0);

        private Pose pose;

        AutonStartingLocation(Pose pose) {
            this.pose = pose;
        }

        AutonStartingLocation(double x, double y, double theta) {
            this.pose = new Pose(x, y, theta);
        }

        public Pose getStartingPose() {
            return pose;
        }

    }

    public static enum FieldLocations {
        LeftFirstBay(116, 263, 0), RightFirstBay(208, 263, 180), RightBackRocket(298, 265, -60),
        LeftBackRocket(26, 265, 240), RightThirdBay(207.0, 307.0, 0.0), LeftHatchReload(23, 15, 90),
        RightHatchReload(301, 15, 90), RightNearRocket(298, 198, 60);

        private Pose pose;

        FieldLocations(Pose pose) {
            this.pose = pose;
        }

        FieldLocations(double x, double y, double theta) {
            this.pose = new Pose(x, y, theta);
        }

        public Pose getLocation() {
            return pose;
        }
    }

    public enum StartingDirection {
        FORWARD(0), BACKWARD(180), LEFT(-90), RIGHT(90);

        private double gyroOffset;

        StartingDirection(double gyroOffset) {
            this.gyroOffset = gyroOffset;
        }

        public synchronized double getGyroOffset() {
            return gyroOffset;
        }
    }
}
