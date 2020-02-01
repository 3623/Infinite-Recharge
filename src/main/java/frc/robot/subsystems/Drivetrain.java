/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import java.io.IOException;

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
  SpeedControllerGroup Right,Left;
  DifferentialDrive Drivetrain;
  Encoder leftSide, rightSide;

  private final int UPDATE_RATE = 200;
	public DrivetrainModel model;
  private final double DISTANCE_PER_PULSE = model.WHEEL_RADIUS * Math.PI * 2 / 2048.0;

  public CubicSplineFollower waypointNav;

  double time;

  AHRS NavX;

  private DriveControlState controlState = DriveControlState.DISABLED;

  private ShuffleboardTab shuffle = Shuffleboard.getTab("SmartDashboard");

  NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight"); // The Limelight Vision system posts several useful bits
                                                                              // of data to Network Tables.
		NetworkTableEntry tx = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)
  		NetworkTableEntry ty = Lime.getEntry("ty"); // Vertical Offset From Crosshair to Target (-20.5 to 20.5 degrees)
  		NetworkTableEntry ta = Lime.getEntry("ta"); // Target Area (0% of Image to 100% of Image)
		NetworkTableEntry tv = Lime.getEntry("tv"); // Valid Targets (0 or 1, False/True)
		  
	AnalogInput transducer = new AnalogInput(0);
	
	NetworkTableEntry mainPressure = 
		shuffle.add("Main System Pressure", 0)
			.getEntry();
	double x,y,area,valid;

	private enum DriveControlState {
		OPEN_LOOP, // open loop voltage control
		PATH_FOLLOWING, // velocity PID control
		DISABLED,
	}

  public Drivetrain(){
    rightMotor1 = new WPI_TalonFX(DrivetrainConstants.RIGHT_MOTOR_ONE);
    rightMotor2 = new WPI_TalonFX(DrivetrainConstants.RIGHT_MOTOR_TWO);
    leftMotor1 = new WPI_TalonFX(DrivetrainConstants.LEFT_MOTOR_ONE);
    leftMotor2 = new WPI_TalonFX(DrivetrainConstants.LEFT_MOTOR_TWO);
    Right = new SpeedControllerGroup(rightMotor1, rightMotor2);
    Left = new SpeedControllerGroup(leftMotor1, leftMotor2);

    leftSide = new Encoder(DrivetrainConstants.ENCODER_LEFT_A, DrivetrainConstants.ENCODER_LEFT_B, true,
        Encoder.EncodingType.k2X);
    rightSide = new Encoder(DrivetrainConstants.ENCODER_RIGHT_A, DrivetrainConstants.ENCODER_RIGHT_B, true,
        Encoder.EncodingType.k2X);
    leftSide.setDistancePerPulse(DISTANCE_PER_PULSE);
    rightSide.setDistancePerPulse(DISTANCE_PER_PULSE);

    model = new DrivetrainModel();
    model.setPosition(0.0, 0.0, 0.0);
    
    waypointNav = new CubicSplineFollower();

    NavX = new AHRS(SPI.Port.kMXP);

    this.updateThreadStart();
  }

  public void disable() {
		if (controlState != DriveControlState.DISABLED) {
			controlState = DriveControlState.DISABLED;
		}
		setMotorControllers(new Tuple(0.0, 0.0));
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

	public void startPathFollowing() {
		controlState = DriveControlState.PATH_FOLLOWING;
	}

	private void driveWaypointNavigator() {
		Tuple output = waypointNav.updatePursuit(model.center);
		Tuple limitedOut = model.limitAcceleration(output);
		// double leftSpeed = limitedOut.left;
		// double rightSpeed = limitedOut.right;
		double leftSpeed = output.left;
		double rightSpeed = output.right;

		SmartDashboard.putNumber("Left Out 1", leftSpeed);
		SmartDashboard.putNumber("Right Out 1", rightSpeed);
		setVoltages(leftSpeed, leftSpeed /*Is this Right???*/ );
	}

	public void zeroSensors() {
		leftSide.reset();
		rightSide.reset();
		NavX.reset();
	}

	public void driverControl(double xSpeed, double rSpeed, Boolean quickTurn) {
		// setOpenLoop(0.0, 0.0);
		setOpenLoop(xSpeed, rSpeed);
	}

	private void update() {
		double time = Timer.getFPGATimestamp();
		double deltaTime = time - this.time;
		this.time = time;
		this.updatePosition(deltaTime);
		this.monitor();
		SmartDashboard.putNumber("DT", deltaTime);
		
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
		valid = tv.getDouble(0.0);

		mainPressure.setDouble(250*(transducer.getVoltage()/5)-25);

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