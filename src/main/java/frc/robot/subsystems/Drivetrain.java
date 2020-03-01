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
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controls.CubicSplineFollower;
import frc.robot.Constants.DrivetrainConstants;
import frc.util.*;

public class Drivetrain extends SubsystemBase {
	WPI_TalonFX rightMotorMaster, rightMotorFollower, leftMotorMaster, leftMotorFollower;

	private final int UPDATE_RATE = 200;
	public DrivetrainModel model;
	private static final double ENCODER_TICKS_PER_REV = 2048.0; // TODO this could be 2048 CHECK

	public CubicSplineFollower waypointNav;

	private static final double MAX_CURRENT = 50.0; // BANANA i think this is closer
	private StatorCurrentLimitConfiguration currentLimiter = new StatorCurrentLimitConfiguration(true, MAX_CURRENT,
			MAX_CURRENT, 0.01);

	private static final int PIDIDX = 0;
	private static final int CONFIG_TIMEOUT = 30;

	private double kFF = 1023.0 / linearSpeedToTalonSpeed(DrivetrainModel.MAX_SPEED);
	// TODO This is wrong, has to be tuned, should be (1023 * duty-cycle /
	// sensor-velocity-sensor-units-per-100ms).
	private double kP = 0.5;
	private double kD = 0.1;
	private double kI = 0.0;

	double time;

	public AHRS NavX;

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
		rightMotorMaster.configFactoryDefault();
		rightMotorFollower.configFactoryDefault();
		leftMotorMaster.configFactoryDefault();
		leftMotorFollower.configFactoryDefault();
		rightMotorFollower.set(ControlMode.Follower, DrivetrainConstants.RIGHT_MOTOR_ONE);
		leftMotorFollower.set(ControlMode.Follower, DrivetrainConstants.LEFT_MOTOR_ONE);
		rightMotorMaster.setInverted(true);
		leftMotorMaster.setInverted(false);
		rightMotorFollower.setInverted(InvertType.FollowMaster);
		leftMotorFollower.setInverted(InvertType.FollowMaster);
		setBrakeMode(false);
		rightMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
		leftMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1);
		// TODO bring up sensors
		// TODO set phase for the encoders

		leftMotorMaster.configStatorCurrentLimit(currentLimiter);
		rightMotorMaster.configStatorCurrentLimit(currentLimiter);

		leftMotorMaster.config_kF(PIDIDX, kFF);
		leftMotorMaster.config_kP(PIDIDX, kP);
		leftMotorMaster.config_kD(PIDIDX, kD);
		leftMotorMaster.config_kI(PIDIDX, kI);
		rightMotorMaster.config_kF(PIDIDX, kFF);
		rightMotorMaster.config_kP(PIDIDX, kP);
		rightMotorMaster.config_kD(PIDIDX, kD);
		rightMotorMaster.config_kI(PIDIDX, kI);

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
		this.updateOdometry(deltaTime);
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

	private void updateOdometry(double time) {
		double leftSpeed = talonSpeedToLinearSpeed(leftMotorMaster.getSelectedSensorVelocity());
		double rightSpeed = talonSpeedToLinearSpeed(rightMotorMaster.getSelectedSensorVelocity());
		model.updateSpeed(leftSpeed, rightSpeed, time);
		model.updateHeading(NavX.getAngle());
		model.updatePosition(time);
	}

	public void zeroSensors() {
		// TODO talon sensors reset
		NavX.reset();
	}

	public void disable() {
		if (controlState != DriveControlState.DISABLED) {
			controlState = DriveControlState.DISABLED;
		}
		leftMotorMaster.stopMotor();
		leftMotorFollower.stopMotor();
		rightMotorMaster.stopMotor();
		rightMotorFollower.stopMotor();
	}

	public void test(WPI_TalonFX talon) {
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

	private double m_quickStopThreshold = 0.2;
	private double m_quickStopAlpha = 0.1;
	private double m_quickStopAccumulator;

	public void terribleDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		double m_deadband = 0.02;
		xSpeed = Utils.limit(xSpeed);
		xSpeed = Utils.applyDeadband(xSpeed, m_deadband);

		zRotation = Utils.limit(zRotation);
		zRotation = Utils.applyDeadband(zRotation, m_deadband);

		double angularPower;
		boolean overPower;

		if (isQuickTurn) {
			if (Math.abs(xSpeed) < m_quickStopThreshold) {
				m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
						+ m_quickStopAlpha * zRotation * 2;
			}
			overPower = true;
			angularPower = zRotation;
		} else {
			overPower = false;
			angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

			if (m_quickStopAccumulator > 1) {
				m_quickStopAccumulator -= 1;
			} else if (m_quickStopAccumulator < -1) {
				m_quickStopAccumulator += 1;
			} else {
				m_quickStopAccumulator = 0.0;
			}
		}

		double leftMotorOutput = xSpeed + angularPower;
		double rightMotorOutput = xSpeed - angularPower;

		// If rotation is overpowered, reduce both outputs to within acceptable range
		if (overPower) {
			if (leftMotorOutput > 1.0) {
				rightMotorOutput -= leftMotorOutput - 1.0;
				leftMotorOutput = 1.0;
			} else if (rightMotorOutput > 1.0) {
				leftMotorOutput -= rightMotorOutput - 1.0;
				rightMotorOutput = 1.0;
			} else if (leftMotorOutput < -1.0) {
				rightMotorOutput -= leftMotorOutput + 1.0;
				leftMotorOutput = -1.0;
			} else if (rightMotorOutput < -1.0) {
				leftMotorOutput -= rightMotorOutput + 1.0;
				rightMotorOutput = -1.0;
			}
		}

		// Normalize the wheel speeds
		double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
		if (maxMagnitude > 1.0) {
			leftMotorOutput /= maxMagnitude;
			rightMotorOutput /= maxMagnitude;
		}

		setOpenLoop(leftMotorOutput, rightMotorOutput);
	}

	private void setSpeed(double left, double right) {
		double leftTalonSpeed = linearSpeedToTalonSpeed(left);
		double rightTalonSpeed = linearSpeedToTalonSpeed(right);
		leftMotorMaster.set(ControlMode.Velocity, leftTalonSpeed);
		rightMotorMaster.set(ControlMode.Velocity, rightTalonSpeed);
	}

	private static double linearSpeedToTalonSpeed(double linearSpeed) {
		double wheelRotationalSpeed = linearSpeed / DrivetrainModel.WHEEL_CIRCUMFERENCE;
		double encoderRotationSpeed = wheelRotationalSpeed * ENCODER_TICKS_PER_REV;
		double talonSpeed = encoderRotationSpeed / 10.0;
		return talonSpeed;
	}

	private static double talonSpeedToLinearSpeed(double talonSpeed) {
		double ticksPerSecond = talonSpeed * 10.0;
		double wheelRotationalSpeed = ticksPerSecond / ENCODER_TICKS_PER_REV;
		double wheelSpeed = wheelRotationalSpeed / DrivetrainModel.WHEEL_CIRCUMFERENCE;
		return wheelSpeed;
	}

	private void setOpenLoop(double left, double right) {
		if (controlState != DriveControlState.OPEN_LOOP) {
			System.out.println("Switching to open loop control, time: " + time);
			controlState = DriveControlState.OPEN_LOOP;
		}
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
	}

	public static void main(String[] args) throws IOException {
	}

}