package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;


import frc.robot.subsystems.Swerve;

import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotState {
	private static RobotState instance = new RobotState();
	public static RobotState getInstance(){
		return instance;
	}
	
	private static final int kObservationBufferSize = 100;
	
	private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
	//private GoalTracker goal_tracker_;
	private Rotation2d camera_pitch_correction_;
    private Rotation2d camera_yaw_correction_;
    private double differential_height_;
    private double targetHeight = Constants.kDiskTargetHeight;
	private double distance_driven_;
   // private ShooterAimingParameters cached_shooter_aiming_params_ = null;
    
    public final int minimumTargetQuantity = 3;
    private final int primaryTargetIndex = 2;
    private Translation2d lastKnownTargetPosition = new Translation2d();
    public Translation2d lastKnownTargetPosition(){ return lastKnownTargetPosition; }

    public double distanceToTarget(){
        return getLatestFieldToVehicle().getValue().getTranslation().distance(lastKnownTargetPosition);
    }


    private boolean seesTarget = false;
    public boolean seesTarget(){
        return seesTarget;
    }
	
	private double angleToCube = 0;
	public double getAngleToCube(){
		return angleToCube;
	}
	public void setAngleToCube(double angle){
		angleToCube = angle;
	}
	
	public Translation2d getCubePosition(){
	//	List<TrackReport> reports = goal_tracker_.getTracks();
	//	if(!reports.isEmpty())
	//		return goal_tracker_.getTracks().get(0).field_to_goal;
//		else
			return new Translation2d();
	}
	
	private static final Pose2d kVehicleToCamera = new Pose2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), Rotation2d.fromDegrees(0.0));
	
	private RobotState() {
        reset(0, new Pose2d());
    }
	
	/**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        //goal_tracker_ = new GoalTracker();
        //camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
        //camera_yaw_correction_ = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees);
        //differential_height_ = targetHeight - Constants.kCameraZOffset;
        distance_driven_ = 0.0;
    }
    
    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized void setVisionTargetHeight(double height){
        targetHeight = height;
        differential_height_ = height - Constants.kCameraZOffset;
    }

    public double getVisionTargetHeight(){
        return targetHeight;
    }
    
    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    
    public synchronized void resetRobotPosition(Translation2d targetPosition){
    	//List<TrackReport> reports = goal_tracker_.getTracks();
        //if (reports.size() >= minimumTargetQuantity) {
        //    TrackReport report = reports.get(primaryTargetIndex);
        //    Translation2d robotFrameToFieldFrame = report.field_to_goal.inverse().translateBy(targetPosition);
        //    if(robotFrameToFieldFrame.norm() <= 5.0){
        //    	Swerve.getInstance().resetPosition(new Pose2d(Swerve.getInstance().getPose().getTranslation().translateBy(robotFrameToFieldFrame), Swerve.getInstance().getPose().getRotation()));
        //    	System.out.println("Coordinates corrected by " + robotFrameToFieldFrame.norm() + " inches");
        //    }else{
        //    	System.out.println("Coordinate correction too large: " + robotFrameToFieldFrame.norm());
        //    }
        //}else{
      //  	System.out.println("Vision did not detect target");
       // }
    }
    
    public synchronized List<Pose2d> getCaptureTimeFieldToGoal() {
        List<Pose2d> rv = new ArrayList<>();
       // for (TrackReport report : goal_tracker_.getTracks()) {
       //     rv.add(Pose2d.fromTranslation(report.field_to_goal));
       // }
        return rv;
    }

  
    
    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }
    
    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }
    
    public void outputToSmartDashboard(){
        SmartDashboard.putBoolean("Sees Target", seesTarget);

    }
}
