/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;

import frc.robot.Constants.SpinnerConstants;
import frc.robot.commands.EnterLowGear;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Spinner extends CommandBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 Sensor;
  ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
  final ColorMatch colorMatcher = new ColorMatch();
  final Color kBlueTarget = ColorMatch.makeColor(.12, .41, .47);
  final Color kGreenTarget = ColorMatch.makeColor(.16, .58, .26);
  final Color kRedTarget = ColorMatch.makeColor(.53, .33, .13);
  final Color kYellowTarget = ColorMatch.makeColor(.33, .53, .14);
  TalonSRX spinnerMotor;
  
  NetworkTableEntry colorDetected = tab.add("Sensor Sees", "Unknown")
      .getEntry();
  NetworkTableEntry DetectionConfidence = tab.add("Match Confidence", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();
  
  public Spinner(){
    Sensor = new ColorSensorV3(i2cPort);
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
    spinnerMotor = new TalonSRX(SpinnerConstants.SPINNER_MOTOR);
  }

  public String getColorMatch(){
    Color detectedColor = Sensor.getColor();
    
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget){
      colorString = "Blue";
    }
    else if (match.color == kGreenTarget){
      colorString = "Green";
    }
    else if (match.color == kRedTarget){
      colorString = "Red";
    }
    else if (match.color == kYellowTarget){
      colorString = "Yellow";
    }
    else {
      colorString = "Unknown Color";
    }
    colorDetected.setString(colorString);
    DetectionConfidence.setNumber(match.confidence);
    return colorString;
  }

  public void Spin(double speed){
    spinnerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void Stop(){
    spinnerMotor.set(ControlMode.PercentOutput, 0.0);
  }

  

}
