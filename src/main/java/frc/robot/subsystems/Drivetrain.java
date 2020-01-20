/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX rightMotor1, rightMotor2, leftMotor1, leftMotor2;
  SpeedControllerGroup right,left;
  DifferentialDrive Drivetrain;
  Encoder leftSide,rightSide;

  private final double DISTANCE_PER_PULSE = RobotMap.WHEEL_RADIUS*Math.PI*2/2048.0;

  AHRS NavX;

  public Drivetrain(){
    rightMotor1 = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_ONE);
    rightMotor2 = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_TWO);
    leftMotor1 = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_ONE);
    leftMotor2 = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_TWO);
    right = new SpeedControllerGroup(rightMotor1, rightMotor2);
    left = new SpeedControllerGroup(leftMotor1, leftMotor2);
    Drivetrain = new DifferentialDrive(left, right);

    leftSide = new Encoder(RobotMap.ENCODER_LEFT_A,RobotMap.ENCODER_LEFT_B,true,Encoder.EncodingType.k2X);
    rightSide = new Encoder(RobotMap.ENCODER_RIGHT_A,RobotMap.ENCODER_RIGHT_B,true,Encoder.EncodingType.k2X);
    leftSide.setDistancePerPulse(DISTANCE_PER_PULSE);
    rightSide.setDistancePerPulse(DISTANCE_PER_PULSE);

    NavX = new AHRS(SPI.Port.kMXP);
  }

  public void stop(){
    left.disable();
    right.disable();
  }

  public void openLoopControl(double xSpeed, double rSpeed, Boolean quickTurn){
      Drivetrain.curvatureDrive(-xSpeed, rSpeed, quickTurn);
  }

  public void directMotorControl(double leftSpeed, double rightSpeed){
      Drivetrain.tankDrive(leftSpeed, rightSpeed, false);
  }

  public void zeroAllSensors(){
    zeroEncoders();
    zeroNavX();
  }

  public void zeroNavX(){
    NavX.reset();
  }

  public void zeroEncoders(){
    leftSide.reset();
    rightSide.reset();
  }
}
