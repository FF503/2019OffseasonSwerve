/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Superstructure;
import com.team1323.io.Xbox;
import com.team1323.lib.util.Logger;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team1323.lib.util.Util;
import com.team1323.lib.util.CrashTracker;
import com.team1323.loops.Looper;
import com.team254.lib.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import com.team1323.loops.QuinticPathTransmitter;
import com.team1323.loops.RobotStateEstimator;
import frc.robot.subsystems.Pigeon;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI m_oi;
	private Swerve swerve;
	private boolean robotCentric = false;
	private SubsystemManager subsystems;
	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();
	private Pigeon pigeon; 
	private Superstructure s;
	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private RobotState robotState = RobotState.getInstance();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
	m_oi = new OI();

    swerve = Swerve.getInstance();
    pigeon = Pigeon.getInstance();
    s = Superstructure.getInstance();
   
    subsystems = new SubsystemManager(
        Arrays.asList(swerve,s));
        
    enabledLooper.register(RobotStateEstimator.getInstance());
    enabledLooper.register(QuinticPathTransmitter.getInstance());
    disabledLooper.register(RobotStateEstimator.getInstance());
    disabledLooper.register(QuinticPathTransmitter.getInstance());
    subsystems.registerEnabledLoops(enabledLooper);
    subsystems.registerDisabledLoops(disabledLooper);
        
    swerve.zeroSensors();
    swerve.setNominalDriveOutput(0.0);
	swerve.startTracking(Constants.kDiskTargetHeight, new Translation2d(-6.0, 0.0), true, new Rotation2d());
    swerve.stop();
  }



  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
   
  }

  @Override
  public void teleopInit() {
    try {
		disabledLooper.stop();
  		enabledLooper.start();
  		swerve.requireModuleConfiguration();
		SmartDashboard.putBoolean("Auto", false);
	} catch (Throwable t) {
		CrashTracker.logThrowableCrash(t);
		throw t;
	}
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    try {
		double swerveYInput = OI.getDriverLeftYVal();
		double swerveXInput = OI.getDriverLeftXVal();
		double swerveRotationInput = OI.getDriverRightXVal();

		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, robotCentric, false);

		if (OI.getDriverXButton())
		swerve.rotate(90);
		else if (OI.getDriverAButton())
		swerve.rotate(180);
		else if (OI.getDriverXButton())
		swerve.rotate(270);
		else if(OI.getDriverYButton())
		swerve.rotate(0.0);

		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
		Pigeon.getInstance().outputToSmartDashboard();
		//SmartDashboard.putBoolean("Enabled", ds.isEnabled());
		//SmartDashboard.putNumber("Match time", ds.getMatchTime());
	} catch (Throwable t) {
	//	CrashTracker.logThrowableCrash(t);
		throw t;
	}
  
  }

  @Override
	public void disabledInit() {
		try {
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
	
	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {
	
	}


}
