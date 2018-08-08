/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/********* Version *********/
/* TalonSRX: 3.9
 * VictorSPX: 3.9
 * Pigeon IMU: 0.41
 * Phoenix Framework: 5.6.0 
 */

package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
	AutoMovement auto;

	@Override
	public void robotInit() {
		// check that your set up is correct
		auto = new AutoMovement(false, false, false, false, 0);
		auto.configAnglePIDF(_kP, _kI, _kD, _kF, _kIzone, _kPeakOutput);
		auto.configPosPIDF(_kP, _kI, _kD, _kF, _kIzone, _kPeakOutput);
		
	}

	@Override
	public void teleopInit(){
		
	}
	

	
	@Override
	public void teleopPeriodic() {
	
	}
	
	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.IterativeRobotBase#testInit()
	 */
	public void testInit() {
		auto.autoMovementInit();
		auto.motionMagicInit(CruiseVelocity, acceloration, kSlotIdx, kPIDLoopIdx);
		auto.setSetpoints(_postion, turnYaw);
		
		
	}
	@Override
	public void testPeriodic() {
		
		auto.autoPeriodicSetPoint();
		
		
	}
	
	


	
}
