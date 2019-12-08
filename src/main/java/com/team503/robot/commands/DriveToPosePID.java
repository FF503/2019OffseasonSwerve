package com.team503.robot.commands;

import com.team503.lib.geometry.Pose;
import com.team503.lib.geometry.Translation2d;
import com.team503.lib.util.FrogPIDF;
import com.team503.lib.util.FrogPIDF.ControlMode;
import com.team503.lib.util.Util;
import com.team503.robot.RobotState;
import com.team503.robot.subsystems.SwerveDrive;
import com.team503.robot.subsystems.SwerveDrive.DriveMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToPosePID extends Command {
    // Called just before this Command runs the first time
    FrogPIDF xPID;
    FrogPIDF yPID;
    Translation2d zeroVector = new Translation2d(0, 0);
    Pose currentPose;
    Pose target;
    boolean turnTargetSet = false;
    double TOTAL_DISTANCE;
    double distance;

    public DriveToPosePID(Pose target) {
        this.target = target;
    }

    @Override
    protected void initialize() {
        xPID =  new FrogPIDF(0.008, 0, 0, ControlMode.Position_Control);
        yPID =  new FrogPIDF(0.008, 0, 0, ControlMode.Position_Control);
        System.out.println("startin");
        SwerveDrive.getInstance().setMode(DriveMode.PIDControl);
        xPID.setSetpoint(target.getX());
        yPID.setSetpoint(target.getY());
        xPID.setTolerance(3.0);
        yPID.setTolerance(3.0);
        // if (Math.abs(Util.boundAngle0to360Degrees(target.getTheta()) - Util.boundAngle0to360Degrees(
        //         Util.boundAngle0to360Degrees(RobotState.getInstance().getCurrentTheta()))) < 3.0) {
        //     SwerveDrive.getInstance().rotate(target.getTheta());
        // }
        TOTAL_DISTANCE = target.toNewVector().minus(RobotState.getInstance().getCurrentPose().toNewVector()).getNorm();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        currentPose = RobotState.getInstance().getCurrentPose().getTranslatedPose();
        Translation2d translationVector = new Translation2d(xPID.calculateOutput(currentPose.getX(), false),yPID.calculateOutput(currentPose.getY(), false));

        SmartDashboard.putNumber("y error:", yPID.getError());
        SmartDashboard.putNumber("x output:", translationVector.getX());
        SmartDashboard.putNumber("y output:", translationVector.getY());

        Translation2d outputtedTranslation2d = SwerveDrive.getInstance().getCurrentTranslationVector();
        boolean goodOutput = (outputtedTranslation2d.getX() == translationVector.getX())
                && (outputtedTranslation2d.getY() == translationVector.getY());
        SmartDashboard.putBoolean("good auton output: ", goodOutput);

        distance = target.toNewVector().minus(currentPose.toNewVector()).getNorm();
        if (currentPose.getY() < 80){
            System.out.println("alert");
            translationVector.times(0.3);
        }
        SmartDashboard.putNumber("distance remaining", distance);
        if (distance / TOTAL_DISTANCE > 0.5 && !turnTargetSet) {
            System.out.println("ROTATINT NOW");
            SwerveDrive.getInstance().rotate(target.getTheta());
            turnTargetSet = true;
        }
        // SwerveDrive.getInstance().rotate(target.getTheta());
        System.out.println("set:" + translationVector.toString());
        SwerveDrive.getInstance().drive(translationVector, Math.max(SwerveDrive.getInstance().getRotationalOutput(), -0.2));
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return xPID.onTarget() && yPID.onTarget() && angleTolerance();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        SwerveDrive.getInstance().setMode(DriveMode.TeleopDrive);
        SwerveDrive.getInstance().drive(zeroVector);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

    private boolean angleTolerance() {
        return Math.abs(Util.boundAngle0to360Degrees(target.getTheta())- Util.boundAngle0to360Degrees(currentPose.getTheta())) < 3.0;
    }
}