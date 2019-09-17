package com.team503.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import com.team503.lib.util.SwerveHeadingController;
import com.team503.lib.util.SwerveInverseKinematics;
import com.team503.robot.Robot;
import com.team503.robot.RobotState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends Subsystem {
    // Instance declaration
    private static SwerveDrive instance = null;


    public static SwerveDrive getInstance() {
        if (instance == null)
            instance = new SwerveDrive();
        return instance;
    }

    // Module declaration
    public SwerveModule frontRight, frontLeft, rearLeft, rearRight;
    List<SwerveModule> modules;
    List<SwerveModule> positionModules;

    // Evade maneuver variables
    Translation2d clockwiseCenter = new Translation2d();
    Translation2d counterClockwiseCenter = new Translation2d();
    boolean evading = false;
    boolean evadingToggled = false;

    /**
     * Toggle the swerve drive into an algorithmically
     * determined center of rotation to get around defense.
     */
    public void toggleEvade() {
        evading = !evading;
        evadingToggled = true;
    }

    // Heading controller methods
    Pigeon pigeon;
    SwerveHeadingController headingController = new SwerveHeadingController();

    public void temporarilyDisableHeadingController() {
        headingController.temporarilyDisable();
    }

    public double getTargetHeading() {
        return headingController.getTargetHeading();
    }

    // Vision dependencies
    RobotState robotState;

    boolean needsToNotifyDrivers = false;

    public boolean needsToNotifyDrivers() {
        if (needsToNotifyDrivers) {
            needsToNotifyDrivers = false;
            return true;
        }
        return false;
    }

    // Module configuration variables (for beginnning of auto)
    boolean modulesReady = false;
    boolean alwaysConfigureModules = false;
    boolean moduleConfigRequested = false;

    public void requireModuleConfiguration() {
        modulesReady = false;
    }

    public void alwaysConfigureModules() {
        alwaysConfigureModules = true;
    }

    double rotationScalar;
    double trajectoryStartTime = 0;
    Translation2d lastTrajectoryVector = new Translation2d();

    public Translation2d getLastTrajectoryVector() {
        return lastTrajectoryVector;
    }

    boolean hasStartedFollowing = false;
    boolean hasFinishedPath = false;

    public boolean hasFinishedPath() {
        return hasFinishedPath;
    }

    private SwerveDrive() {
        try {
            this.rearRight = com.team503.lib.util.Util.readSwerveJSON(Robot.bot.getBackRightName());
            this.rearLeft = com.team503.lib.util.Util.readSwerveJSON(Robot.bot.getBackLeftName());
            this.frontRight = com.team503.lib.util.Util.readSwerveJSON(Robot.bot.getFrontRightName());
            this.frontLeft = com.team503.lib.util.Util.readSwerveJSON(Robot.bot.getFrontLeftName());
        } catch (Exception e) {
            e.printStackTrace();
        }

        modules = Arrays.asList(frontRight, rearRight, rearLeft, frontLeft);
        positionModules = Arrays.asList(frontRight, rearRight, rearLeft, frontLeft);

        pigeon = Pigeon.getInstance();

        robotState = RobotState.getInstance();
    }

    // Assigns appropriate directions for scrub factors
    public void setCarpetDirection(boolean standardDirection) {
        modules.forEach((m) -> m.setCarpetDirection(standardDirection));
    }

    // Teleop driving variables
    private Translation2d translationalVector = new Translation2d();
    private double rotationalInput = 0;
    private Translation2d lastDriveVector = new Translation2d();
    private final Translation2d rotationalVector = Translation2d.identity();
    private double lowPowerScalar = 0.6;

    public void setLowPowerScalar(double scalar) {
        lowPowerScalar = scalar;
    }

    private double maxSpeedFactor = 1.0;

    public void setMaxSpeed(double max) {
        maxSpeedFactor = max;
    }

    private boolean robotCentric = false;

    // Swerve kinematics (exists in a separate class)
    private SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();

    public void setCenterOfRotation(Translation2d center) {
        inverseKinematics.setCenterOfRotation(center);
    }

    // The swerve's various control states
    public enum ControlState {
        NEUTRAL, MANUAL, POSITION, ROTATION, DISABLED, VECTORIZED, TRAJECTORY, VELOCITY, VISION
    }

    private ControlState currentState = ControlState.NEUTRAL;

    public ControlState getState() {
        return currentState;
    }

    public void setState(ControlState newState) {
        currentState = newState;
    }

    /**
     * Main function used to send manual input during teleop.
     * 
     * @param x            forward/backward input
     * @param y            left/right input
     * @param rotate       rotational input
     * @param robotCentric gyro use
     * @param lowPower     scaled down output
     */
    public void sendInput(double x, double y, double rotate, boolean robotCentric, boolean lowPower) {
        Translation2d translationalInput = new Translation2d(x, y);
        double inputMagnitude = translationalInput.norm();

        /*
         * Snap the translational input to its nearest pole, if it is within a certain
         * threshold of it.
         */
        double threshold = Math.toRadians(10.0);
        if (Math.abs(
                translationalInput.direction().distance(translationalInput.direction().nearestPole())) < threshold) {
            translationalInput = translationalInput.direction().nearestPole().toTranslation().scale(inputMagnitude);
        }

        double deadband = 0.25;
        if (inputMagnitude < deadband) {
            translationalInput = new Translation2d();
            inputMagnitude = 0;
        }

        /*
         * Scale x and y by applying a power to the magnitude of the vector they create,
         * in order to make the controls less sensitive at the lower end.
         */
        final double power = (lowPower) ? 1.75 : 1.5;
        Rotation2d direction = translationalInput.direction();
        double scaledMagnitude = Math.pow(inputMagnitude, power);
        translationalInput = new Translation2d(direction.cos() * scaledMagnitude, direction.sin() * scaledMagnitude);

        rotate = (Math.abs(rotate) < deadband) ? 0 : rotate;
        rotate = Math.pow(Math.abs(rotate), 1.75) * Math.signum(rotate);

        translationalInput = translationalInput.scale(maxSpeedFactor);
        rotate *= maxSpeedFactor;

        translationalVector = translationalInput;

        if (lowPower) {
            translationalVector = translationalVector.scale(lowPowerScalar);
            rotate *= lowPowerScalar;
        } else {
            rotate *= 0.8;
        }

        if (rotate != 0 && rotationalInput == 0) {
            headingController.disable();
        } else if (rotate == 0 && rotationalInput != 0) {
            headingController.temporarilyDisable();
        }

        rotationalInput = rotate;

        setState(ControlState.MANUAL);

        if (inputMagnitude > 0.1)
            lastDriveVector = new Translation2d(x, y);
        else if (translationalVector.x() == 0.0 && translationalVector.y() == 0.0 && rotate != 0.0) {
            lastDriveVector = rotationalVector;
        }

        this.robotCentric = robotCentric;
    }

    // Possible new control method for rotation
    public Rotation2d averagedDirection = Rotation2d.identity();

    // public void resetAveragedDirection() {
    // averagedDirection = pose.getRotation();
    // }

    public void setAveragedDirection(double degrees) {
        averagedDirection = Rotation2d.fromDegrees(degrees);
    }

    public final double rotationDirectionThreshold = Math.toRadians(5.0);
    public final double rotationDivision = 1.0;

    public synchronized void updateControllerDirection(Translation2d input) {
        if (Util.epsilonEquals(input.norm(), 1.0, 0.1)) {
            Rotation2d direction = input.direction();
            double roundedDirection = Math.round(direction.getDegrees() / rotationDivision) * rotationDivision;
            averagedDirection = Rotation2d.fromDegrees(roundedDirection);
        }
    }

    // Various methods to control the heading controller
    public synchronized void rotate(double goalHeading) {
        if (translationalVector.x() == 0 && translationalVector.y() == 0)
            rotateInPlace(goalHeading);
        else
            headingController.setStabilizationTarget(com.team503.lib.util.Util
                    .placeInAppropriate0To360Scope(RobotState.getInstance().getCurrentTheta(), goalHeading));
    }

    public void rotateInPlace(double goalHeading) {
        setState(ControlState.ROTATION);
        headingController.setStationaryTarget(com.team503.lib.util.Util
                .placeInAppropriate0To360Scope(RobotState.getInstance().getCurrentTheta(), goalHeading));
    }

    public void rotateInPlaceAbsolutely(double absoluteHeading) {
        setState(ControlState.ROTATION);
        headingController.setStationaryTarget(absoluteHeading);
    }

    public void setPathHeading(double goalHeading) {
        headingController.setSnapTarget(com.team503.lib.util.Util
                .placeInAppropriate0To360Scope(RobotState.getInstance().getCurrentTheta(), goalHeading));
    }

    public void setAbsolutePathHeading(double absoluteHeading) {
        headingController.setSnapTarget(absoluteHeading);
    }

    /** Sets MotionMagic targets for the drive motors */
    // public void setPositionTarget(double directionDegrees, double
    // magnitudeInches) {
    // setState(ControlState.POSITION);
    // modules.forEach((m) -> m.setModuleAngle(directionDegrees));
    // modules.forEach((m) -> m.setDrivePositionTarget(magnitudeInches));
    // }

    /** Locks drive motors in place with MotionMagic */
    // public void lockDrivePosition() {
    // modules.forEach((m) -> m.setDrivePositionTarget(0.0));
    // }

    /** Puts drive motors into closed-loop velocity mode */
    // public void setVelocity(Rotation2d direction, double velocityInchesPerSecond)
    // {
    // setState(ControlState.VELOCITY);
    // modules.forEach((m) -> m.setModuleAngle(direction.getDegrees()));
    // modules.forEach((m) -> m.setVelocitySetpoint(velocityInchesPerSecond));
    // }

    /** Configures each module to match its assigned vector */
    public void setDriveOutput(List<Translation2d> driveVectors) {
        for (int i = 0; i < modules.size(); i++) {
            if (com.team503.lib.util.Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
                    modules.get(i).getModuleAngle().getDegrees())) {
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
                modules.get(i).setDriveOpenLoop(-driveVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(driveVectors.get(i).norm());
            }
        }
    }

    public void setDriveOutput(List<Translation2d> driveVectors, double percentOutputOverride) {
        for (int i = 0; i < modules.size(); i++) {
            if (com.team503.lib.util.Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
                    modules.get(i).getModuleAngle().getDegrees())) {
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
                modules.get(i).setDriveOpenLoop(-percentOutputOverride);
            } else {
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(percentOutputOverride);
            }
        }
    }

    /**
     * Configures each module to match its assigned vector, but puts the drive
     * motors into closed-loop velocity mode
     */
    // public void setVelocityDriveOutput(List<Translation2d> driveVectors) {
    // for (int i = 0; i < modules.size(); i++) {
    // if
    // (com.team503.lib.util.Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
    // modules.get(i).getModuleAngle().getDegrees())) {
    // modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() +
    // 180.0);
    // modules.get(i)
    // .setVelocitySetpoint(-driveVectors.get(i).norm() *
    // Constants.kSwerveMaxSpeedInchesPerSecond);
    // } else {
    // modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    // modules.get(i)
    // .setVelocitySetpoint(driveVectors.get(i).norm() *
    // Constants.kSwerveMaxSpeedInchesPerSecond);
    // }
    // }
    // }

    // public void setVelocityDriveOutput(List<Translation2d> driveVectors, double
    // velocityOverride) {
    // for (int i = 0; i < modules.size(); i++) {
    // if
    // (com.team503.lib.util.Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
    // modules.get(i).getModuleAngle().getDegrees())) {
    // modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() +
    // 180.0);
    // modules.get(i).setVelocitySetpoint(-velocityOverride);
    // } else {
    // modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
    // modules.get(i).setVelocitySetpoint(velocityOverride);
    // }
    // }
    // }

    /** Sets only module angles to match their assigned vectors */
    public void setModuleAngles(List<Translation2d> driveVectors) {
        for (int i = 0; i < modules.size(); i++) {
            if (com.team503.lib.util.Util.shouldReverse(driveVectors.get(i).direction().getDegrees(),
                    modules.get(i).getModuleAngle().getDegrees())) {
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees() + 180.0);
            } else {
                modules.get(i).setModuleAngle(driveVectors.get(i).direction().getDegrees());
            }
        }
    }

    /** Increases each module's rotational power cap for the beginning of auto */
    public void set10VoltRotationMode(boolean tenVolts) {
        modules.forEach((m) -> m.set10VoltRotationMode(tenVolts));
    }

    /**
     * @return Whether or not at least one module has reached its MotionMagic
     *         setpoint
     */
    // public boolean positionOnTarget() {
    // boolean onTarget = false;
    // for (SwerveModule m : modules) {
    // onTarget |= m.drivePositionOnTarget();
    // }
    // return onTarget;
    // }

    /**
     * @return Whether or not all modules have reached their angle setpoints
     */
    public boolean moduleAnglesOnTarget() {
        boolean onTarget = true;
        for (SwerveModule m : modules) {
            onTarget &= m.angleOnTarget();
        }
        return onTarget;
    }

    /**
     * Determines which wheels the robot should rotate about in order to perform an
     * evasive maneuver
     */
    public synchronized void determineEvasionWheels() {
        Translation2d here = lastDriveVector
                .rotateBy(new Rotation2d(RobotState.getInstance().getCurrentTheta()).inverse());
        List<Translation2d> wheels = Robot.bot.kModuleTranslations;
        clockwiseCenter = wheels.get(0);
        counterClockwiseCenter = wheels.get(wheels.size() - 1);
        for (int i = 0; i < wheels.size() - 1; i++) {
            Translation2d cw = wheels.get(i);
            Translation2d ccw = wheels.get(i + 1);
            if (here.isWithinAngle(cw, ccw)) {
                clockwiseCenter = ccw;
                counterClockwiseCenter = cw;
            }
        }
    }


    /** Called every cycle to update the swerve based on its control state */
    public synchronized void updateControlCycle(double timestamp) {
        double rotationCorrection = 0; // headingController.updateRotationCorrection(RobotState.getInstance().getCurrentTheta(),
                                       // timestamp);


        switch (currentState) {
        case MANUAL:
            if (evading && evadingToggled) {
                determineEvasionWheels();
                double sign = Math.signum(rotationalInput);
                if (sign == 1.0) {
                    inverseKinematics.setCenterOfRotation(clockwiseCenter);
                } else if (sign == -1.0) {
                    inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
                }
                evadingToggled = false;
            } else if (evading) {
                double sign = Math.signum(rotationalInput);
                if (sign == 1.0) {
                    inverseKinematics.setCenterOfRotation(clockwiseCenter);
                } else if (sign == -1.0) {
                    inverseKinematics.setCenterOfRotation(counterClockwiseCenter);
                }
            } else if (evadingToggled) {
                inverseKinematics.setCenterOfRotation(Translation2d.identity());
                evadingToggled = false;
            }
            if (translationalVector.equals(Translation2d.identity()) && rotationalInput == 0.0) {
                if (lastDriveVector.equals(rotationalVector)) {
                    stop();
                } else {
                    setDriveOutput(inverseKinematics.updateDriveVectors(lastDriveVector, rotationCorrection,
                            RobotState.getInstance().getCurrentTheta(), robotCentric), 0.0);
                }
            } else {
                setDriveOutput(
                        inverseKinematics.updateDriveVectors(translationalVector, rotationalInput + rotationCorrection,
                                RobotState.getInstance().getCurrentTheta(), robotCentric));
            }
            break;
        case POSITION:
            break;
        case ROTATION:
            break;
        case VECTORIZED:
            break;
        case TRAJECTORY:
            break;
        case VISION:
            break;
        case VELOCITY:

            break;
        case NEUTRAL:
            stop();
            break;
        case DISABLED:

            break;
        default:
            break;
        }
    }

    // private final Loop loop = new Loop() {

    // @Override
    public void onStart(double timestamp) {
        // synchronized (SwerveDrive.this) {
        translationalVector = new Translation2d();
        lastDriveVector = rotationalVector;
        rotationalInput = 0;
        // resetAveragedDirection();
        headingController.temporarilyDisable();
        stop();
        // lastUpdateTimestamp = timestamp;
        // }
    }

    // @Override
    public void onLoop(double timestamp) {
        // synchronized (SwerveDrive.this) {
        // if (modulesReady || (getState() != ControlState.TRAJECTORY)) {
        // updatePose(timestamp);
        // alternatePoseUpdate();
        // }
        updateControlCycle(timestamp);
        writePeriodicOutputs();
        readPeriodicInputs();
        // lastUpdateTimestamp = timestamp;
        // }
    }

    // @Override
    public void onStop(double timestamp) {
        // synchronized (SwerveDrive.this) {
        translationalVector = new Translation2d();
        rotationalInput = 0;
        stop();
        // }
    }

    // };

    // public Request trackRequest(double visionTargetHeight, Translation2d
    // endTranslation, boolean hasDisk) {
    // return new Request() {

    // @Override
    // public void act() {
    // robotHasDisk = hasDisk;
    // useFixedVisionOrientation = false;
    // visionCutoffDistance = Constants.kClosestVisionDistance;
    // visionTrackingSpeed = Constants.kDefaultVisionTrackingSpeed;
    // resetVisionUpdates();
    // setVisionTrajectory(visionTargetHeight, endTranslation, false,
    // VisionState.CURVED);
    // }

    // @Override
    // public boolean isFinished() {
    // return getState() == ControlState.VISION && (robotState.distanceToTarget() <
    // visionCutoffDistance);
    // }

    // };
    // }

    // public Request trackRequest(double visionTargetHeight, Translation2d
    // endTranslation, boolean hasDisk,
    // Rotation2d fixedOrientation, double cutoffDistance, double trackingSpeed) {
    // return new Request() {

    // @Override
    // public void act() {
    // robotHasDisk = hasDisk;
    // fixedVisionOrientation = fixedOrientation;
    // useFixedVisionOrientation = true;
    // visionCutoffDistance = cutoffDistance;
    // visionTrackingSpeed = trackingSpeed;
    // resetVisionUpdates();
    // setVisionTrajectory(visionTargetHeight, endTranslation, false,
    // VisionState.CURVED);
    // }

    // @Override
    // public boolean isFinished() {
    // return getState() == ControlState.VISION && (robotState.distanceToTarget() <
    // visionCutoffDistance);
    // }

    // };
    // }

    // public Request startTrackRequest(double visionTargetHeight, Translation2d
    // endTranslation, boolean hasDisk,
    // VisionState vState) {
    // return new Request() {

    // @Override
    // public void act() {
    // robotHasDisk = hasDisk;
    // useFixedVisionOrientation = false;
    // visionCutoffDistance = Constants.kClosestVisionDistance;
    // visionTrackingSpeed = /* Constants.kDefaultVisionTrackingSpeed */ 48.0;
    // Optional<ShooterAimingParameters> aim = robotState.getAimingParameters();
    // if (aim.isPresent()) {
    // if (vState == VisionState.LINEAR) {
    // visionTrackingSpeed = Constants.kVisionSpeedTreemap
    // .getInterpolated(new InterpolatingDouble(aim.get().getRange())).value;
    // System.out.println("Vision tracking speed set to: " + visionTrackingSpeed);
    // }
    // if (aim.get().getRange() < 54.0) {
    // // visionTrackingSpeed = 30.0;
    // // System.out.println("Vision tracking speed set low");
    // }
    // }
    // resetVisionUpdates();
    // setVisionTrajectory(visionTargetHeight, endTranslation, false, vState);
    // }

    // };
    // }

    // public void startTracking(double visionTargetHeight, Translation2d
    // endTranslation, boolean hasDisk,
    // Rotation2d fixedOrientation) {
    // robotHasDisk = hasDisk;
    // fixedVisionOrientation = fixedOrientation;
    // useFixedVisionOrientation = true;
    // visionCutoffDistance = Constants.kClosestVisionDistance;
    // visionTrackingSpeed = Constants.kDefaultVisionTrackingSpeed;
    // resetVisionUpdates();
    // setVisionTrajectory(visionTargetHeight, endTranslation, true,
    // VisionState.CURVED);
    // }

    // public Request startTrackRequest(double visionTargetHeight, Translation2d
    // endTranslation, boolean hasDisk,
    // Rotation2d fixedOrientation) {
    // return new Request() {

    // @Override
    // public void act() {
    // robotHasDisk = hasDisk;
    // fixedVisionOrientation = fixedOrientation;
    // useFixedVisionOrientation = true;
    // visionCutoffDistance = Constants.kClosestVisionDistance;
    // visionTrackingSpeed = Constants.kDefaultVisionTrackingSpeed;
    // resetVisionUpdates();
    // setVisionTrajectory(visionTargetHeight, endTranslation, false,
    // VisionState.CURVED);
    // }

    // };
    // }

    // public Request startTrackRequest(double visionTargetHeight, Translation2d
    // endTranslation, boolean hasDisk,
    // Rotation2d fixedOrientation, double cutoffDistance, double trackingSpeed) {
    // return new Request() {

    // @Override
    // public void act() {
    // robotHasDisk = hasDisk;
    // fixedVisionOrientation = fixedOrientation;
    // useFixedVisionOrientation = true;
    // visionCutoffDistance = cutoffDistance;
    // visionTrackingSpeed = trackingSpeed;
    // resetVisionUpdates();
    // setVisionTrajectory(visionTargetHeight, endTranslation, false,
    // VisionState.CURVED);
    // }

    // };
    // }

    // public Request waitForTrackRequest() {
    // return new Request() {

    // @Override
    // public void act() {

    // }

    // @Override
    // public boolean isFinished() {
    // return getState() == ControlState.VISION
    // && /* motionPlanner.isDone() */ (robotState.distanceToTarget() <
    // visionCutoffDistance);
    // }

    // };
    // }

    // public Request strictWaitForTrackRequest() {
    // return new Request() {

    // @Override
    // public void act() {

    // }

    // @Override
    // public boolean isFinished() {
    // return getState() == ControlState.VISION && motionPlanner.isDone();
    // }

    // };
    // }

    // public Request trajectoryRequest(Translation2d relativeEndPos, double
    // targetHeading, double defaultVel) {
    // return new Request() {

    // @Override
    // public void act() {
    // setRobotCentricTrajectory(relativeEndPos, targetHeading, defaultVel);
    // }

    // @Override
    // public boolean isFinished() {
    // return (getState() == ControlState.TRAJECTORY && motionPlanner.isDone())
    // || getState() == ControlState.MANUAL;
    // }

    // };
    // }

    // public Request startTrajectoryRequest(Translation2d relativeEndPos, double
    // targetHeading, double defaultVel) {
    // return new Request() {

    // @Override
    // public void act() {
    // setRobotCentricTrajectory(relativeEndPos, targetHeading, defaultVel);
    // }

    // };
    // }

    // public Request openLoopRequest(Translation2d input, double rotation) {
    // return new Request() {

    // @Override
    // public void act() {
    // setState(ControlState.MANUAL);
    // sendInput(input.x(), input.y(), rotation, false, false);
    // }

    // };
    // }

    // public Request velocityRequest(Rotation2d direction, double magnitude) {
    // return new Request() {

    // @Override
    // public void act() {
    // setVelocity(direction, magnitude);
    // }

    // };
    // }

    // public void setNominalDriveOutput(double voltage) {
    // modules.forEach((m) -> m.setNominalDriveOutput(voltage));
    // }

    /**
     * Sets the maximum rotation speed opf the modules, based on the robot's
     * velocity
     */
    // public void setMaxRotationSpeed() {
    // double currentDriveSpeed = translationalVector.norm() *
    // Constants.kSwerveMaxSpeedInchesPerSecond;
    // double newMaxRotationSpeed = Constants.kSwerveRotationMaxSpeed
    // / ((Constants.kSwerveRotationSpeedScalar * currentDriveSpeed) + 1.0);
    // modules.forEach((m) -> m.setMaxRotationSpeed(newMaxRotationSpeed));
    // }

    @Override
    public synchronized void readPeriodicInputs() {
        modules.forEach((m) -> m.readPeriodicInputs());
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        modules.forEach((m) -> m.writePeriodicOutputs());
    }


    

    @Override
    public synchronized void stop() {
        setState(ControlState.NEUTRAL);
        modules.forEach((m) -> m.stop());
    }

    @Override
    public synchronized void zeroSensors() {
        // zeroSensors(Constants.kRobotLeftStartingPose);
    }

    /**
     * Zeroes the drive motors, and sets the robot's internal position and heading
     * to match that of the fed pose
     */
    public synchronized void zeroSensors(Pose2d startingPose) {
        // pigeon.setAngle(startingPose.getRotation().getUnboundedDegrees());
        // modules.forEach((m) -> m.zeroSensors(startingPose));
        // pose = startingPose;
        // distanceTraveled = 0;
    }

    public synchronized void resetPosition(Pose2d newPose) {
        // pose = new Pose2d(newPose.getTranslation(), pose.getRotation());
        // modules.forEach((m) -> m.zeroSensors(pose));
        // distanceTraveled = 0;
    }

    public synchronized void setXCoordinate(double x) {
        // pose.getTranslation().setX(x);
        // modules.forEach((m) -> m.zeroSensors(pose));
        // System.out.println("X coordinate reset to: " + pose.getTranslation().x());
    }

    public synchronized void setYCoordinate(double y) {
        // pose.getTranslation().setY(y);
        // modules.forEach((m) -> m.zeroSensors(pose));
        // System.out.println("Y coordinate reset to: " + pose.getTranslation().y());
    }

    @Override
    public void outputTelemetry() {
        modules.forEach((m) -> m.outputTelemetry());
        // SmartDashboard.putNumberArray("Robot Pose", new double[] {
        // pose.getTranslation().x(), pose.getTranslation().y(),
        // pose.getRotation().getUnboundedDegrees() });
        SmartDashboard.putString("Swerve State", currentState.toString());
        if (Robot.bot.kDebuggingOutput) {
            // SmartDashboard.putNumber("Robot X", pose.getTranslation().x());
            // SmartDashboard.putNumber("Robot Y", pose.getTranslation().y());
            SmartDashboard.putNumber("Robot Heading", RobotState.getInstance().getCurrentTheta());
            SmartDashboard.putString("Heading Controller", headingController.getState().toString());
            SmartDashboard.putNumber("Target Heading", headingController.getTargetHeading());
            SmartDashboard.putString("Last Drive Vector", lastDriveVector.toString());
            SmartDashboard.putString("Center Of Rotation", RobotState.getInstance().getCenterOfRotation().toString());

            // SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
            // SmartDashboard.putNumber("Robot Velocity", currentVelocity);
            SmartDashboard.putString("Swerve State", currentState.toString());
            // SmartDashboard.putBoolean("Vision Updates Allowed", visionUpdatesAllowed);
            SmartDashboard.putNumberArray("Pigeon YPR", pigeon.getYPR());
        }
    }
}
