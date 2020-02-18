/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import java.io.IOException;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controls.CubicSplineFollower;
import frc.robot.Constants.DrivetrainConstants;
import frc.util.*;

public class Drivetrain extends SubsystemBase {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	WPI_TalonFX rightMotorMaster, rightMotorFollower, leftMotorMaster, leftMotorFollower;
	Encoder leftSide, rightSide;

	private final int UPDATE_RATE = 200;
	public DrivetrainModel model;
	private final double DISTANCE_PER_PULSE = model.WHEEL_RADIUS * Math.PI * 2 / 2048.0;

	public CubicSplineFollower waypointNav;

	double time;

	AHRS NavX;

	private DriveControlState controlState = DriveControlState.DISABLED;

	private ShuffleboardTab shuffle = Shuffleboard.getTab("SmartDashboard");

	AnalogInput transducer = new AnalogInput(0);

	NetworkTableEntry mainPressure = shuffle.add("Main System Pressure", 0).withWidget(BuiltInWidgets.kDial)
			.withProperties(Map.of("min", 0, "max", 130)).getEntry();

	private enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		PATH_FOLLOWING, // velocity PID control
		DISABLED,
	}

	public Drivetrain() {
		rightMotorMaster = new WPI_TalonFX(DrivetrainConstants.RIGHT_MOTOR_ONE);
		rightMotorFollower = new WPI_TalonFX(DrivetrainConstants.RIGHT_MOTOR_TWO);
		leftMotorMaster = new WPI_TalonFX(DrivetrainConstants.LEFT_MOTOR_ONE);
		leftMotorFollower = new WPI_TalonFX(DrivetrainConstants.LEFT_MOTOR_TWO);
		rightMotorFollower.set(ControlMode.Follower, DrivetrainConstants.RIGHT_MOTOR_ONE);
		leftMotorFollower.set(ControlMode.Follower, DrivetrainConstants.LEFT_MOTOR_ONE);
		rightMotorMaster.setInverted(true);
		leftMotorMaster.setInverted(false);
		rightMotorFollower.setInverted(InvertType.FollowMaster);
		leftMotorFollower.setInverted(InvertType.FollowMaster);
		rightMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
		leftMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
		// dont think this will work

		// BANANA current limiting? (This conrols max force and prevents slipping ..)

		/*
		 * Config the Velocity closed loop gains in slot0
		 * 
		 * _talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF,
		 * Constants.kTimeoutMs); _talon.config_kP(Constants.kPIDLoopIdx,
		 * Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		 * _talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI,
		 * Constants.kTimeoutMs); _talon.config_kD(Constants.kPIDLoopIdx,
		 * Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		 */

		leftSide = new Encoder(DrivetrainConstants.ENCODER_LEFT_A, DrivetrainConstants.ENCODER_LEFT_B, true,
				Encoder.EncodingType.k2X);
		rightSide = new Encoder(DrivetrainConstants.ENCODER_RIGHT_A, DrivetrainConstants.ENCODER_RIGHT_B, true,
				Encoder.EncodingType.k2X);
		leftSide.setDistancePerPulse(DISTANCE_PER_PULSE);
		rightSide.setDistancePerPulse(DISTANCE_PER_PULSE);

		model = new DrivetrainModel();
		model.setPosition(0.0, 0.0, 0.0);

		waypointNav = new CubicSplineFollower(DrivetrainModel.MAX_SPEED, DrivetrainModel.WHEEL_BASE);

		NavX = new AHRS(SPI.Port.kMXP);

		this.updateThreadStart();
	}

	private void update() {
		double time = Timer.getFPGATimestamp();
		double deltaTime = time - this.time;
		this.time = time;
		this.updatePosition(deltaTime);
		this.monitor();
		SmartDashboard.putNumber("DT", deltaTime);

		mainPressure.setDouble(250 * (transducer.getVoltage() / 5) - 25);

		switch (controlState) {
		case OPEN_LOOP:
			break;
		case PATH_FOLLOWING:
			driveWaypointNavigator();
			break;
		case DISABLED:
			break;
		}
	}

	private void updateThreadStart() {
		Thread t = new Thread(() -> {
			while (!Thread.interrupted()) {
				this.update();
				try {
					Thread.sleep(1000 / UPDATE_RATE);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
		t.start();
	}

	private void updatePosition(double time) {
		model.updateSpeed(leftSide.getRate(), rightSide.getRate(), time);
		model.updateHeading(NavX.getAngle());
		model.updatePosition(time);
	}

	public void zeroSensors() {
		leftSide.reset();
		rightSide.reset();
		NavX.reset();
	}

	public void disable() {
		if (controlState != DriveControlState.DISABLED) {
			controlState = DriveControlState.DISABLED;
		}
		setMotorPercents(0.0, 0.0);
	}

	public void test(TalonFX talon) {
		Faults faults = new Faults();
		/* update motor controller */
		talon.set(ControlMode.PercentOutput, 1.0);
		/* check our live faults */
		talon.getFaults(faults);
		System.out.println("Sensor Vel:" + talon.getSelectedSensorVelocity());
		System.out.println("Sensor Pos:" + talon.getSelectedSensorPosition());
		System.out.println("Out %" + talon.getMotorOutputPercent());
		System.out.println("Out Of Phase:" + faults.SensorOutOfPhase);
	}

	public void setBrakeMode(Boolean enabled) {
		// BANANA TODO is half brake useful??
		NeutralMode mode;
		if (enabled)
			mode = NeutralMode.Brake;
		else
			mode = NeutralMode.Coast;
		rightMotorMaster.setNeutralMode(mode);
		rightMotorFollower.setNeutralMode(mode);
		leftMotorMaster.setNeutralMode(mode);
		leftMotorFollower.setNeutralMode(mode);
	}

	public void startPathFollowing() {
		if (controlState != DriveControlState.PATH_FOLLOWING) {
			System.out.println("Switching to path following, time: " + time);
			controlState = DriveControlState.PATH_FOLLOWING;
		}
		// BANANA TODO print waypoints
	}

	private void driveWaypointNavigator() {
		Tuple output = waypointNav.updatePursuit(model.center);
		double leftSpeed = output.left;
		double rightSpeed = output.right;

		SmartDashboard.putNumber("Left Out 1", leftSpeed);
		SmartDashboard.putNumber("Right Out 1", rightSpeed);
		setSpeed(leftSpeed, rightSpeed);
		// BANANA: No, i updated so that path following now returns speed in m/s for
		// each side
		// of the drivetrain. This speed should be used in FPID (or motion magic)
		// everything else looks good

		/*
		 * double targetVelocity_UnitsPer100ms = leftYstick * 500.0 * 4096 / 600; 500
		 * RPM in either direction _talon.set(ControlMode.Velocity,
		 * targetVelocity_UnitsPer100ms);
		 */
	}

	public void driverControl(double xSpeed, double rSpeed, Boolean quickTurn) {
		// BANANA: a todo would be customize this. In my experience, curvature drive was
		// not always the best. Can easily copy paste some control code (arcade, cheesy,
		// or whatnot)
		// and modify it as needed. This gets rid of the hidden magic that
		// the wpilib drivetrain class has.
		// this is the only time DT is used, and im not a big fan
		// we only need velocity pid control (for auto), voltage control (open loop)
		// (for tele). If this was changed into say:
		// func terribleDrive(forward, turn) {
		// do this to get left right voltages
		// setOpenLoop(left, right)
		// i think it would be cleaner and more direct, and not much work (just copy
		// paste)
		// this provides a good backbone for doing other stuff to like PTR
		// and drive following heading
		if (controlState != DriveControlState.OPEN_LOOP) {
			System.out.println("Switching to open loop control, time: " + time);
			controlState = DriveControlState.OPEN_LOOP;
		}
		DT.curvatureDrive(xSpeed, rSpeed, quickTurn);
		// setOpenLoop(left, right);
	}

	public void setOpenLoop(double left, double right) {
		if (controlState != DriveControlState.OPEN_LOOP) {
			System.out.println("Switching to open loop control, time: " + time);
			controlState = DriveControlState.OPEN_LOOP;
		}
		setMotorPercents(left, right);
	}

	public void setMotorPercents(double left, double right) {
		leftMotorMaster.set(ControlMode.PercentOutput, left);
		rightMotorMaster.set(ControlMode.PercentOutput, right);
	}

	private void monitor() {
		// SmartDashboard.putNumber("Left Encoder", encLeft.getDistance());
		// SmartDashboard.putNumber("Rights Encoder", encRight.getDistance());
		SmartDashboard.putNumber("Drivetrain Model X", model.center.x);
		SmartDashboard.putNumber("Drivetrain Model Y", model.center.y);
		SmartDashboard.putNumber("Heading", model.center.heading);
		SmartDashboard.putNumber("Radians", model.center.r);
		// SmartDashboard.putNumber("Drivetrain Heading", navx.getAngle());
	}

	public static void main(String[] args) throws IOException {
	}

}