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
import com.ctre.phoenix.motorcontrol.can.*;

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

	WPI_TalonFX rightMotor1, rightMotor2, leftMotor1, leftMotor2;
	SpeedControllerGroup Right, Left;
	DifferentialDrive Drivetrain;
	Encoder leftSide, rightSide;
	DifferentialDrive DT;

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
		rightMotor1 = new WPI_TalonFX(DrivetrainConstants.RIGHT_MOTOR_ONE);
		rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
		rightMotor2 = new WPI_TalonFX(DrivetrainConstants.RIGHT_MOTOR_TWO);
		rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
		leftMotor1 = new WPI_TalonFX(DrivetrainConstants.LEFT_MOTOR_ONE);
		leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
		leftMotor2 = new WPI_TalonFX(DrivetrainConstants.LEFT_MOTOR_TWO);
		leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
		Right = new SpeedControllerGroup(rightMotor1, rightMotor2);
		Left = new SpeedControllerGroup(leftMotor1, leftMotor2);
		DT = new DifferentialDrive(Left, Right);

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
		setMotorControllers(new Tuple(0.0, 0.0));
	}

	public void startPathFollowing() {
		controlState = DriveControlState.PATH_FOLLOWING;
	}

	private void driveWaypointNavigator() {
		Tuple output = waypointNav.updatePursuit(model.center);
		double leftSpeed = output.left;
		double rightSpeed = output.right;

		SmartDashboard.putNumber("Left Out 1", leftSpeed);
		SmartDashboard.putNumber("Right Out 1", rightSpeed);
		setVoltages(leftSpeed, leftSpeed /* Is this Right??? */ );
		// BANANA: No, i updated so that path following now returns speed in m/s for
		// each side
		// of the drivetrain. This speed should be used in FPID (or motion magic)
		// everything else looks good
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
		setMotorControllers(left, right);
	}

	private void setMotorControllers(double left, double right) {
		Left.set(left);
		Right.set(right);
	}

	private void setMotorControllers(Tuple out) {
		setMotorControllers(out.left, out.right);
	}

	private void setVoltages(double left, double right) {
		setMotorControllers(left / 12.0, right / 12.0);
	}

	private void setVoltages(Tuple voltages) {
		setVoltages(voltages.left, voltages.right);
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