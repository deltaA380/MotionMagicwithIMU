package org.usfirst.frc.team217.robot;

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


/**
 * @author walsl
 * methods to implement for motion magic plus angle pid control loop on 
 * on the talon srx 
 */
public class AutoMovement {
	
	// hardware implementation all pins are theortical
	// could flexable add more tallons 
	// I am aasumeing that the driveTrain is in a tank format 
	TalonSRX _leftFollower = new TalonSRX(2);
	TalonSRX _rightMaster = new TalonSRX(1);
	PigeonIMU _pidgey ;

	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _robotTargetAngle, robotPostion;

	/**
	 * @param leftInvert
	 * @param leftSensorPhase
	 * @param rightInverted
	 * @param rightSensorPhase
	 */
	public AutoMovement(boolean leftInvert, boolean leftSensorPhase, boolean rightInverted, boolean rightSensorPhase, int imuID) {
		
		_robotTargetAngle = 0;
		double postion =0;
		_pidgey = new PigeonIMU(imuID);
		/* Configure output and sensor direction */
		_leftFollower.setInverted(leftInvert);
		_leftFollower.setSensorPhase(leftSensorPhase);
		_rightMaster.setInverted(rightInverted);
		_rightMaster.setSensorPhase(rightSensorPhase);

		
		_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_leftFollower.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		/*
		 * Max out the peak output (for all modes). However you can limit the output of
		 * a given PID object with configClosedLoopPeakOutput().
		 */
		_leftFollower.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_leftFollower.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		
		_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

	}
	
	public void configPosPIDF() {
		// to do change kp gains to motuion porofile 
		/* FPID Gains for distance servo */
		_rightMaster.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		_rightMaster.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		_rightMaster.config_IntegralZone(Constants.kSlot_Distanc, (int) Constants.kGains_Distanc.kIzone,
				Constants.kTimeoutMs);
		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Distanc, Constants.kGains_Distanc.kPeakOutput,
				Constants.kTimeoutMs);
		
	}

	/**
	 * Angle pidf is aux pid on the auxilarry pid slot
	 *  */
	public void configAnglePIDF() {
		/* FPID Gains for turn servo */
		_rightMaster.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_rightMaster.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_rightMaster.config_IntegralZone(Constants.kSlot_Turning, (int) Constants.kGains_Turning.kIzone,
				Constants.kTimeoutMs);
		
		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput,Constants.kTimeoutMs);

		
	}
	
	/**
	 * @param auxPidPolarity
	 *  * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
		 * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
		 * local output is PID0 - PID1, and other side Talon is PID0 + PID1
	 * @param _closeLoppTimeMs
	 * 
	 * 	 * 1ms per loop. PID loop can be slowed down if need be. For example, - if
		 * sensor updates are too slow - sensor deltas are very small per update, so
		 * derivative error never gets large enough to be useful. - sensor movement is
		 * very slow causing the derivative error to be near zero.
		 * 
		 * 		
		 * 
	 */
	public void configLoopPeramiters(boolean auxPidPolarity, int _closeLoppTimeMs) {

		int closedLoopTimeMs = _closeLoppTimeMs;
		
		_rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);
		_rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);

		_rightMaster.configAuxPIDPolarity(auxPidPolarity, Constants.kTimeoutMs);


	}
	/**
	 * @param CruiseVelocity
	 * @param acceloration
	 * @param kSlotIdx
	 * @param kPIDLoopIdx
	 */
	public void motionMagicInit(int CruiseVelocity, int acceloration, int kSlotIdx, int kPIDLoopIdx) {

		_rightMaster.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		/* set acceleration and vcruise velocity - see documentation */
		_rightMaster.configMotionCruiseVelocity(CruiseVelocity, Constants.kTimeoutMs);
		_rightMaster.configMotionAcceleration(acceloration, Constants.kTimeoutMs);

	}

	/**
	 * inig loop that sets up all modes 
	 * 
	 * right useses the left encoder for postion feedback data 
	 * 
	 */
	public void autoMovementInit() {
		/* Disable all motor controllers */
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftFollower.set(ControlMode.PercentOutput, 0);

		/* Set Neutral Mode */
		_leftFollower.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);

		/** Feedback Sensor Configuration */

		/* Configure the left Talon's selected sensor as local QuadEncoder */
		_leftFollower.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
				Constants.PID_PRIMARY, // PID Slot for Source [0, 1]
				Constants.kTimeoutMs); // Configuration Timeout

		/*
		 * Configure the Remote Talon's selected sensor as a remote sensor for the right
		 * Talon
		 */
		_rightMaster.configRemoteFeedbackFilter(_leftFollower.getDeviceID(), // Device ID of Source
				RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
				Constants.REMOTE_0, // Source number [0, 1]
				Constants.kTimeoutMs); // Configuration Timeout

		
		
		/* Configure the Pigeon IMU to the other Remote Slot on the Right Talon */
		_rightMaster.configRemoteFeedbackFilter(_pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Yaw,
				Constants.REMOTE_1, Constants.kTimeoutMs);

		/* Setup Sum signal to be used for Distance */
		_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback
																											// Device of
																											// Remote
																										// Talon
		_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs); // Quadrature
																											// Encoder
																											// of
																											// current
																											// Talon

		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY,
				Constants.kTimeoutMs);

		/* Scale Feedback by 0.5 to half the sum of Distance */
		_rightMaster.configSelectedFeedbackCoefficient(0.5, // Coefficient
				Constants.PID_PRIMARY, // PID Slot of Source
				Constants.kTimeoutMs); // Configuration Timeout

		/*
		 * Configure Remote Slot 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index
		 */
		_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, Constants.PID_TURN,
				Constants.kTimeoutMs);

		/*
		 * Scale the Feedback Sensor using a coefficient (Configured for 3600 units of
		 * resolution)
		 */
		_rightMaster.configSelectedFeedbackCoefficient(
				Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation, Constants.PID_TURN,
				Constants.kTimeoutMs);
		
		
		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroSensors();


	}

	/**
	 * @param _postion
	 * @param turnYaw
	 */
	public void setSetpoints(double _postion, double turnYaw) {
		_robotTargetAngle= turnYaw;
		robotPostion = _postion;
	}

	
	
	/**
	 * manGages motion magic during auto periood
	 */
	public void autoPeriodicSetPoint() {

		/* Calculate targets from gamepad inputs */
		double target_sensorUnits = robotPostion * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
		double target_turn = _robotTargetAngle;
		/*
		 * Configured for motion magic on Quad Encoders' Sum and Auxiliary PID
		 * on Pigeon's Yaw
		 */
		_rightMaster.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
		_leftFollower.follow(_rightMaster, FollowerType.AuxOutput1);
	}

	/** Zeroes all sensors, Both Pigeon and Talons */
	void zeroSensors() {
		_leftFollower.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_pidgey.setYaw(0, Constants.kTimeoutMs);
		_pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Sensors] All sensors are zeroed.\n");
	}

	/**
	 * Zeroes QuadEncoders, used to reset Position when performing Position Closed
	 * Loop
	 */
	void zeroDistance() {
		_leftFollower.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("[QuadEncoders] All encoders are zeroed.\n");
	}

}
