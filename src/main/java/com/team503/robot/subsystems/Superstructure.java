package com.team503.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import com.team503.robot.Robot;
// import com.team503.robot.Robot.bot;
import com.team503.robot.RobotState;
// import com.team503.robot.loops.ILooper;
import com.team503.robot.loops.LimelightProcessor;
// import com.team503.robot.loops.Loop;
// import com.team503.robot.subsystems.Swerve.VisionState;
import com.team503.robot.subsystems.requests.Request;
import com.team503.robot.subsystems.requests.RequestList;
// import com.team1323.lib.util.InterpolatingDouble;
// import com.team254.lib.geometry.Rotation2d;
// import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {

	public Elevator elevator;
	public Arm arm;
	public BallIntake ballIntake;
	public DiskIntake diskIntake;
	// public Superstructure s;

	private Compressor compressor;

	private SwerveDrive swerve;

	private LimelightProcessor limelight;

	private RobotState robotState;

	private Element element = Element.DISK;


	public Superstructure() {
		elevator = Elevator.getInstance();
		arm = Arm.getInstance();
		ballIntake = BallIntake.getInstance();
		diskIntake = DiskIntake.getInstance();
		// s = Superstructure.getInstance();

		compressor = new Compressor(0);

		swerve = SwerveDrive.getInstance();

		robotState = RobotState.getInstance();

		queuedRequests = new ArrayList<>(0);
	}

	private static Superstructure instance = null;

	public static Superstructure getInstance() {
		if (instance == null)
			instance = new Superstructure();
		return instance;
	}

	public enum Element {
		BALL, DISK;
	}

	private RequestList activeRequests;
	private ArrayList<RequestList> queuedRequests;
	private Request currentRequest;

	private boolean newRequests = false;
	private boolean activeRequestsCompleted = false;
	private boolean allRequestsCompleted = false;

	public Element getCurrentElement() {
		return element;
	}

	public boolean requestsCompleted() {
		return allRequestsCompleted;
	}

	private void setActiveRequests(RequestList requests) {
		activeRequests = requests;
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}

	private void setQueuedRequests(RequestList requests) {
		queuedRequests.clear();
		queuedRequests.add(requests);
	}

	private void setQueuedRequests(List<RequestList> requests) {
		queuedRequests.clear();
		queuedRequests = new ArrayList<>(requests.size());
		for (RequestList list : requests) {
			queuedRequests.add(list);
		}
	}

	public void request(Request r) {
		setActiveRequests(new RequestList(Arrays.asList(r), false));
		setQueuedRequests(new RequestList());
	}

	public void request(Request active, Request queue) {
		setActiveRequests(new RequestList(Arrays.asList(active), false));
		setQueuedRequests(new RequestList(Arrays.asList(queue), false));
	}

	public void request(RequestList list) {
		setActiveRequests(list);
		setQueuedRequests(new RequestList());
	}

	public void request(RequestList activeList, RequestList queuedList) {
		setActiveRequests(activeList);
		setQueuedRequests(queuedList);
	}

	public void addActiveRequest(Request request) {
		activeRequests.add(request);
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}

	/** Ill-advised */
	public void addForemostActiveRequest(Request request) {
		activeRequests.addToForefront(request);
		newRequests = true;
		activeRequestsCompleted = false;
		allRequestsCompleted = false;
	}

	public void queue(Request request) {
		queuedRequests.add(new RequestList(Arrays.asList(request), false));
	}

	public void queue(RequestList list) {
		queuedRequests.add(list);
	}

	public void replaceQueue(Request request) {
		setQueuedRequests(new RequestList(Arrays.asList(request), false));
	}

	public void replaceQueue(RequestList list) {
		setQueuedRequests(list);
	}

	public void replaceQueue(List<RequestList> lists) {
		setQueuedRequests(lists);
	}

	// private final Loop loop = new Loop(){

	// @Override
	public void onStart(double timestamp) {
		stop();
		enableCompressor(true);
	}

	// @Override
	public void onLoop(double timestamp) {
		synchronized (Superstructure.this) {

			// double elevatorHeight = elevator.getHeight();
			// swerve.setMaxSpeed(Robot.bot.kSwerveSpeedTreeMap.getInterpolated(new
			// InterpolatingDouble(elevatorHeight)).value);

			if (!activeRequestsCompleted) {
				if (newRequests) {
					if (activeRequests.isParallel()) {
						boolean allActivated = true;
						for (Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator
								.hasNext();) {
							Request request = iterator.next();
							boolean allowed = request.allowed();
							allActivated &= allowed;
							if (allowed)
								request.act();
						}
						newRequests = !allActivated;
					} else {
						if (activeRequests.isEmpty()) {
							activeRequestsCompleted = true;
							return;
						}
						currentRequest = activeRequests.remove();
						currentRequest.act();
						newRequests = false;
					}
				}
				if (activeRequests.isParallel()) {
					boolean done = true;
					for (Request request : activeRequests.getRequests()) {
						done &= request.isFinished();
					}
					activeRequestsCompleted = done;
				} else if (currentRequest.isFinished()) {
					if (activeRequests.isEmpty()) {
						activeRequestsCompleted = true;
					} else if (activeRequests.getRequests().get(0).allowed()) {
						newRequests = true;
						activeRequestsCompleted = false;
					}
				}
			} else {
				if (!queuedRequests.isEmpty()) {
					setActiveRequests(queuedRequests.remove(0));
				} else {
					allRequestsCompleted = true;
				}
			}

		}
	}

	public void onStop(double timestamp) {
		disabledState();
	}

	public synchronized void sendManualInput(double armOutput, double elevatorOutput) {
		RequestList list = RequestList.emptyList();
		if (armOutput != 0) {
			list.add(arm.openLoopRequest(armOutput));
		} else if (arm.isOpenLoop()) {
			list.add(arm.lockAngleRequest());
		}
		if (elevatorOutput != 0) {
			list.add(elevator.openLoopRequest(elevatorOutput));
		} else if (elevator.isOpenLoop()) {
			list.add(elevator.lockHeightRequest());
		}

		if (!list.isEmpty()) {
			request(list);
		}
	}

	public void enableCompressor(boolean enable) {
		compressor.setClosedLoopControl(enable);
	}

	public RequestList elevatorArmConfig(double elevatorHeight, double armAngle) {
		return new RequestList(Arrays.asList(elevator.heightRequest(elevatorHeight), arm.angleRequest(armAngle)), true);
	}

	public RequestList idleRequest() {
		return new RequestList(Arrays.asList(arm.openLoopRequest(0.0), elevator.openLoopRequest(0.0)), true);
	}

	@Override
	public void stop() {
		setActiveRequests(idleRequest());
	}

	@Override
	public void zeroSensors() {

	}

	// @Override
	// public void registerEnabledLoops(ILooper enabledLooper) {
	// enabledLooper.register(loop);
	// }

	@Override
	public void outputTelemetry() {
	}

	public Request waitRequest(double seconds) {
		return new Request() {
			double startTime = 0.0;
			double waitTime = 1.0;

			@Override
			public void act() {
				startTime = Timer.getFPGATimestamp();
				waitTime = seconds;
			}

			@Override
			public boolean isFinished() {
				return (Timer.getFPGATimestamp() - startTime) > waitTime;
			}
		};
	}

	///// States/////

	public void disabledState() {
		RequestList state = new RequestList(Arrays.asList(ballIntake.stateRequest(BallIntake.State.OFF),
				arm.angleRequest(Robot.bot.kArmPrimaryStowAngle), diskIntake.stateRequest(DiskIntake.State.OFF)), true);
		request(state);
	}

	public void neutralState() {
		RequestList state = new RequestList(Arrays.asList(
				// ballCarriage.stateRequest(BallCarriage.State.OFF),
				ballIntake.stateRequest(BallIntake.State.OFF), diskIntake.stateRequest(DiskIntake.State.OFF)),

				true);
		request(state);
	}

	public void ballIntakingState() {

		element = Element.BALL;

		RequestList state = new RequestList(Arrays.asList(elevator.heightRequest(Robot.bot.kElevatorBallIntakeHeight),
				arm.angleRequest(Robot.bot.kArmIntakingAngle), ballIntake.stateRequest(BallIntake.State.INTAKING),
				diskIntake.stateRequest(DiskIntake.State.OFF), ballIntake.waitForBallRequest()), true);
		/*
		 * RequestList queue = new RequestList(Arrays.asList(
		 * elevator.heightRequest(Robot.bot.kElevatorBallIntakeHeight),
		 * wrist.angleRequest(Robot.bot.kArmBallHoldingAngle),
		 * ballCarriage.stateRequest(BallCarriage.State.OFF),
		 * ballIntake.stateRequest(BallIntake.State.HOLDING)), true);
		 */
		request(state);
	}

	public void ballScoringState(double elevatorHeight, double armAngle) {
		RequestList state = new RequestList(
				Arrays.asList(elevator.heightRequest(elevatorHeight), arm.angleRequest(armAngle),
						ballIntake.stateRequest(BallIntake.State.HOLDING), diskIntake.stateRequest(DiskIntake.State.OFF)),
				true);
		request(state);
	}

	public void ball(double elevatorHeight, double armAngle) {
		RequestList state = new RequestList(
				Arrays.asList(elevator.heightRequest(elevatorHeight), arm.angleRequest(armAngle),
						ballIntake.stateRequest(BallIntake.State.OFF), diskIntake.stateRequest(DiskIntake.State.OFF)),
				true);
		request(state);
	}
	public void diskReceivingState() {

		element = Element.DISK;

		RequestList state = new RequestList(Arrays.asList(elevator.heightRequest(Robot.bot.kElevatorHumanLoaderHeight),
				diskIntake.stateRequest(DiskIntake.State.INTAKING), arm.angleRequest(Robot.bot.kArmHumanLoaderAngle),
				ballIntake.stateRequest(BallIntake.State.OFF), diskIntake.waitForDiskRequest()), true);
		RequestList queue = new RequestList(Arrays.asList(diskIntake.stateRequest(DiskIntake.State.HOLDING)), true);
		request(state, queue);
	}

	public void diskScoringState(double elevatorHeight, double armAngle){
		RequestList state = new RequestList(Arrays.asList(
			elevator.heightRequest(elevatorHeight),
			arm.angleRequest(armAngle),
			diskIntake.stateRequest(DiskIntake.State.HOLDING),
			ballIntake.stateRequest(BallIntake.State.OFF)), true);
		request(state); 
	}

}
