package org.usfirst.frc.team217.robot;

import org.usfirst.frc.team3630.robot.Consts;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author walsl methods to implement for motion magic plus angle pid control
 *         loop on on the talon srx
 */
public class AutoMovement {

	// hardware implementation all pins are theortical
	// could flexable add more tallons
	// I am aasumeing that the driveTrain is in a tank format
	TalonSRX leftFollower = new TalonSRX(2);
	TalonSRX rightMaster = new TalonSRX(1);
	PigeonIMU pidgey;

	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double robotTargetAngle, robotPostion;

	/**
	 * @param leftInvert
	 * @param leftSensorPhase
	 * @param rightInverted
	 * @param rightSensorPhase
	 */
	public AutoMovement(boolean leftInvert, boolean leftSensorPhase, boolean rightInverted, boolean rightSensorPhase,
			int imuID) {

		robotTargetAngle = 0;
		 robotPostion = 0;
		pidgey = new PigeonIMU(imuID);
		/* Configure output and sensor direction */
		leftFollower.setInverted(leftInvert);
		leftFollower.setSensorPhase(leftSensorPhase);
		rightMaster.setInverted(rightInverted);
		rightMaster.setSensorPhase(rightSensorPhase);

		rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		leftFollower.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		/*
		 * Max out the peak output (for all modes). However you can limit the output of
		 * a given PID object with configClosedLoopPeakOutput().
		 */
		leftFollower.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		leftFollower.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

	}

	/**
	 * @param _kP
	 * @param _kI
	 * @param _kD
	 * @param _kF
	 * @param _kIzone
	 * @param _kPeakOutput
	 */
	public void configPosPIDF(double _kP, double _kI, double _kD, double _kF, double _kIzone, double _kPeakOutput) {
		// to do change kp gains to motuion porofile
		/* FPID Gains for distance servo */
		rightMaster.config_kP(Constants.kSlot_Distanc, _kP, Constants.kTimeoutMs);
		rightMaster.config_kI(Constants.kSlot_Distanc, _kI, Constants.kTimeoutMs);
		rightMaster.config_kD(Constants.kSlot_Distanc, _kD, Constants.kTimeoutMs);

		rightMaster.config_kF(Constants.kSlot_Distanc, _kF, Constants.kTimeoutMs);
		rightMaster.config_IntegralZone(Constants.kSlot_Distanc, (int) _kIzone, Constants.kTimeoutMs);
		rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Distanc, _kPeakOutput, Constants.kTimeoutMs);

	}

	/**
	 * @param _kP
	 * @param _kI
	 * @param _kD
	 * @param _kF
	 * @param _kIzone
	 * @param _kPeakOutput
	 * 
	 *            angle pid controller at aux pid sensor
	 */
	public void configAnglePIDF(double _kP, double _kI, double _kD, double _kF, double _kIzone, double _kPeakOutput) {
		/* FPID Gains for turn servo */
		rightMaster.config_kP(Constants.kSlot_Turning, _kP, Constants.kTimeoutMs);
		rightMaster.config_kI(Constants.kSlot_Turning, _kI, Constants.kTimeoutMs);
		rightMaster.config_kD(Constants.kSlot_Turning, _kD, Constants.kTimeoutMs);
		rightMaster.config_kF(Constants.kSlot_Turning, _kF, Constants.kTimeoutMs);
		rightMaster.config_IntegralZone(Constants.kSlot_Turning, (int) _kIzone, Constants.kTimeoutMs);
		rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, _kPeakOutput, Constants.kTimeoutMs);
	}

	/**
	 * @param auxPidPolarity
	 *            configAuxPIDPolarity(boolean invert, int timeoutMs) false means
	 *            talon's local output is PID0 + PID1, and other side Talon is PID0
	 *            - PID1 true means talon's local output is PID0 - PID1, and other
	 *            side Talon is PID0 + PID1
	 * @param _closeLoppTimeMs
	 * 
	 *            1ms per loop. PID loop can be slowed down if need be. For example,
	 *            - if sensor updates are too slow - sensor deltas are very small
	 *            per update, so derivative error never gets large enough to be
	 *            useful. - sensor movement is very slow causing the derivative
	 *            error to be near zero.
	 */
	public void configLoopPeramiters(boolean auxPidPolarity, int _closeLoppTimeMs) {

		int closedLoopTimeMs = _closeLoppTimeMs;
		rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);
		rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);
		rightMaster.configAuxPIDPolarity(auxPidPolarity, Constants.kTimeoutMs);

	}

	/**
	 * @param CruiseVelocity
	 * @param acceloration
	 * @param kSlotIdx
	 * @param kPIDLoopIdx
	 */
	public void motionMagicInit(int CruiseVelocity, int acceloration, int kSlotIdx, int kPIDLoopIdx) {

		rightMaster.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		/* set acceleration and vcruise velocity - see documentation */
		rightMaster.configMotionCruiseVelocity(CruiseVelocity, Constants.kTimeoutMs);
		rightMaster.configMotionAcceleration(acceloration, Constants.kTimeoutMs);

	}

	/**
	 * inig loop that sets up all modes
	 * 
	 * right useses the left encoder for postion feedback data
	 * 
	 */
	public void autoMovementInit() {
		/* Disable all motor controllers */
		rightMaster.set(ControlMode.PercentOutput, 0);
		leftFollower.set(ControlMode.PercentOutput, 0);

		/* Set Neutral Mode */
		leftFollower.setNeutralMode(NeutralMode.Brake);
		rightMaster.setNeutralMode(NeutralMode.Brake);

		/** Feedback Sensor Configuration */

		/* Configure the left Talon's selected sensor as local QuadEncoder */
		leftFollower.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
				Constants.PID_PRIMARY, // PID Slot for Source [0, 1]
				Constants.kTimeoutMs); // Configuration Timeout

		/*
		 * Configure the Remote Talon's selected sensor as a remote sensor for the right
		 * Talon
		 */
		rightMaster.configRemoteFeedbackFilter(leftFollower.getDeviceID(), // Device ID of Source
				RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
				Constants.REMOTE_0, // Source number [0, 1]
				Constants.kTimeoutMs); // Configuration Timeout

		/* Configure the Pigeon IMU to the other Remote Slot on the Right Talon */
		rightMaster.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Yaw,
			1, Constants.kTimeoutMs);

		/* Setup Sum signal to be used for Distance */
		rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback
																											// Device of
																											// Remote
		// Talon
		rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs); // Quadrature
																											// Encoder
																											// of
																											// current
																											// Talon

		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY,
				Constants.kTimeoutMs);

		/* Scale Feedback by 0.5 to half the sum of Distance */
		rightMaster.configSelectedFeedbackCoefficient(0.5, // Coefficient
				Constants.PID_PRIMARY, // PID Slot of Source
				Constants.kTimeoutMs); // Configuration Timeout

		/*
		 * Configure Remote Slot 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index
		 */
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, Constants.PID_TURN,
				Constants.kTimeoutMs);

		/*
		 * Scale the Feedback Sensor using a coefficient (Configured for 3600 units of
		 * resolution)
		 */
		rightMaster.configSelectedFeedbackCoefficient(
				Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation, Constants.PID_TURN,
				Constants.kTimeoutMs);

		_firstCall = true;
		_state = false;
		zeroSensors();

	}
	
	public void getDiaognostics() {
		SmartDashboard.putNumber("leftsideTicks", getRotations(leftFollower));
		SmartDashboard.putNumber("leftsideVelocity", getVelocity(leftFollower));
	}
	

	/**
	 * @param _talon
	 * @return actual rrotation of talon in a rotation of the wheel
	 */
	public double getRotations(TalonSRX _talon) {
			int ticksPerRotation = 0;
		double distance_ticks = _talon.getSelectedSensorPosition(0);
		double distance_rotations = distance_ticks / ticksPerRotation;
		return distance_rotations;
	}

	public double getTicks(TalonSRX _talon) {
		double distance_ticks = _talon.getSelectedSensorPosition(0);
		return distance_ticks;

	}

	/**
	 * @param _talon
	 * @return velocity in in/ second. from native taon units
	 */
	public double getVelocity(TalonSRX _talon) {
		int ticksPerRotation = 0;
		double velocity_milliseconds = (double) _talon.getSelectedSensorVelocity(0) / ticksPerRotation;
		double velocity_seconds = velocity_milliseconds * 10 * 6 * Math.PI * .0254;
		return velocity_seconds;
	}

	
	public void setSetpoints(double _postion, double turnYaw) {
		robotTargetAngle = turnYaw;
		robotPostion = _postion;
	}

	
	/**
	 * manGages motion magic during auto periood
	 */
	public void autoPeriodicSetPoint() {

		// TODO need to make converstion from inches to encoder ticks the setpoint unit for motion magic
		double targetSesnorUnits = robotPostion * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
		double target_turn = robotTargetAngle;
		/*
		 * Configured for motion magic on Quad Encoders' Sum and Auxiliary PID on
		 * Pigeon's Yaw
		 */
		rightMaster.set(ControlMode.MotionMagic, targetSesnorUnits, DemandType.AuxPID, target_turn);
		leftFollower.follow(rightMaster, FollowerType.AuxOutput1);
		
	}

	/** Zeroes all sensors, Both Pigeon and Talons */
	public void zeroSensors() {
		leftFollower.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);

		pidgey.setYaw(0, Constants.kTimeoutMs);
		pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Sensors] All sensors are zeroed.\n");
	}

	public void stopMotors() {
		rightMaster.set(ControlMode.PercentOutput, 0);
		leftFollower.set(ControlMode.PercentOutput, 0);

	}

	/**
	 * Zeroes QuadEncoders, used to reset Position when performing Position Closed
	 * Loop
	 */
	public void zeroDistance() {
		leftFollower.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("[QuadEncoders] All encoders are zeroed.\n");
	}

	public void zeroIMU() {
		pidgey.setYaw(0, Constants.kTimeoutMs);
		pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Sensors] IMU is zeroed.\n");

	}

}
