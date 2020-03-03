/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.Turret;
import frc.util.Utils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
    private static final double SPEED_THRESHOLD = 100.0;
    private CANSparkMax /*shooterMaster,*/ shooterFollower;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private double speedSetpoint = 0.0;

    ShuffleboardTab settings = Shuffleboard.getTab("Tuning");
    // private NetworkTableEntry shooterPIDSystem = settings.addPersistent("Shooter
    // PID System", shooterPID)
    // .withWidget(BuiltInWidgets.kPIDController).getEntry();

    NetworkTableEntry currentRPM = Shuffleboard.getTab("In-Match").add("Shooter RPM", 0)
            .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 11050)).getEntry();
    public double x, y, area;

    public Flywheel() {
        //shooterMaster = new CANSparkMax(1, MotorType.kBrushless);
        //shooterMaster.setInverted(true);
        shooterFollower = new CANSparkMax(2, MotorType.kBrushless);
        //shooterFollower.follow(shooterMaster, true);
        //shooterMaster.restoreFactoryDefaults();
        shooterFollower.restoreFactoryDefaults();

        maxRPM = 5700;
        kP = 6e-5; // BANANA why is this not outside of constructor?
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 1/maxRPM;
        kMaxOutput = 1.0;
        kMinOutput = 0;
        

        /*shooterMaster.getPIDController().setP(kP);
        shooterMaster.getPIDController().setI(kI);
        shooterMaster.getPIDController().setD(kD);
        shooterMaster.getPIDController().setIZone(kIz);
        shooterMaster.getPIDController().setFF(kFF);
        shooterMaster.getPIDController().setOutputRange(kMinOutput, kMaxOutput);*/
        shooterFollower.getPIDController().setP(kP);
        shooterFollower.getPIDController().setI(kI);
        shooterFollower.getPIDController().setD(kD);
        shooterFollower.getPIDController().setIZone(kIz);
        shooterFollower.getPIDController().setFF(kFF);
        shooterFollower.getPIDController().setOutputRange(kMinOutput, kMaxOutput);
    }

    public void setSpeed(double RPM) { // ALWAYS USE FINAL OUTPUT TARGET RPM!!!!!!!!!
        speedSetpoint = RPM * 35.0 / 18.0;
        //shooterMaster.getPIDController().setReference(speedSetpoint, ControlType.kVelocity);
        shooterFollower.getPIDController().setReference(speedSetpoint, ControlType.kVelocity);
    }

    Boolean isAtSpeed() {
        return Utils.withinThreshold(getVelocity(), speedSetpoint, SPEED_THRESHOLD);
    }

    public boolean getRunning() {
        if (speedSetpoint > 0) {
            return true;
        } else {
            return false;
        }
    }

    public double getVelocity() {
        //return shooterMaster.getEncoder().getVelocity() * 18.0 / 35.0;
        return shooterFollower.getEncoder().getVelocity() * 18.0 / 35.0;
    }

    public void monitor() {
        SmartDashboard.putNumber("Shooter velocity", getVelocity());
        SmartDashboard.putNumber("Shooter setpoint", speedSetpoint);
        //SmartDashboard.putNumber("Shooter output", shooterMaster.getAppliedOutput());
        SmartDashboard.putNumber("Shooter output", shooterFollower.getAppliedOutput());
    }

    public void disable() {
        //shooterMaster.disable();
        shooterFollower.disable();
    }
}
