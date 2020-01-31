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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.*;

import frc.robot.Constants.SpinnerConstants;

public class Spinner extends SubsystemBase {

  Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 Sensor;
  final ColorMatch colorMatcher = new ColorMatch();
  final Color kBlueTarget = ColorMatch.makeColor(.12, .41, .47);
  final Color kGreenTarget = ColorMatch.makeColor(.16, .58, .26);
  final Color kRedTarget = ColorMatch.makeColor(.53, .33, .13);
  final Color kYellowTarget = ColorMatch.makeColor(.33, .53, .14);
  VictorSPX spinnerMotor;

  
  public Spinner(){
    Sensor = new ColorSensorV3(i2cPort);
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
    spinnerMotor = new VictorSPX(SpinnerConstants.SPINNER_MOTOR);
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
    // Shuffleboard Output Shows Black or the Color that's Matched.
    Shuffleboard.getTab("SmartDashboard")
      .addBoolean("isRed", () -> match.color == kRedTarget)
      .withProperties(Map.of("colorWhenTrue", "red"));
    Shuffleboard.getTab("SmartDashboard")
      .addBoolean("isYellow", () -> match.color == kYellowTarget)
      .withProperties(Map.of("colorWhenTrue", "yellow"));
    Shuffleboard.getTab("SmartDashboard")
      .addBoolean("isBlue", () -> match.color == kBlueTarget)
      .withProperties(Map.of("colorWhenTrue", "blue"));
    Shuffleboard.getTab("SmartDashboard")
      .addBoolean("isGreen", () -> match.color == kGreenTarget)
      .withProperties(Map.of("colorWhenTrue", "green"));
    return colorString;
  }

  public void Spin(double speed){
    spinnerMotor.set(ControlMode.PercentOutput, speed);
  }

  public void Stop(){
    spinnerMotor.set(ControlMode.PercentOutput, 0.0);
  }

  

}
