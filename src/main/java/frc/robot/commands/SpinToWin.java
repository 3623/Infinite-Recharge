/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

/**
 * An example command.  You can replace me with your own command.
 */

public class SpinToWin extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Spinner ColorWheelSpinner;
  private final String ColorData;
  private char targetColor;
  private char LastState;
  private double spins;
  public SpinToWin(Spinner colorSpinner){
    ColorWheelSpinner = colorSpinner;
    addRequirements(colorSpinner);
    ColorData = DriverStation.getInstance().getGameSpecificMessage();
  }

  public void initialize() {
    spins = 0.0;
    if (ColorData.length() > 0){
      switch(ColorData.charAt(0)){
        case 'b' :
          targetColor = 'r';
          break;
        case 'g' :
          targetColor = 'y';
          break;
        case 'r' :
          targetColor = 'b';
          break;
        case 'y' :
          targetColor = 'g';
          break;
        default:
          LastState = 'f';
      }
    }
      String ColorSeen = ColorWheelSpinner.getColorMatch();
      switch (ColorSeen){
        case "Blue" :
          LastState = 'b';
          break;
        case "Green" :
          LastState = 'g';
          break;
        case "Red" :
          LastState = 'r';
          break;
        case "Yellow" :
          LastState = 'y';
          break;
        default :
          LastState = 'f';
          break;
      }
      ColorWheelSpinner.Spin(0.5);
    }

  public void execute(){
    if (LastState != 'f'){
      if (ColorData.length() > 0){
        String CurrentColor = ColorWheelSpinner.getColorMatch();
        if (Character.toLowerCase(CurrentColor.charAt(0)) == targetColor){
          LastState = 'f';
        }
      }
      else {
        String CurrentColor = ColorWheelSpinner.getColorMatch();
        if (Character.toLowerCase(CurrentColor.charAt(0)) != LastState){
          switch (LastState){
            case 'b' :
              if (CurrentColor == "Yellow"){
                spins -= .25;
              }
              else if (CurrentColor == "Green"){
                spins += .25;
              }
              break;
            case 'g' :
              if (CurrentColor == "Blue"){
                spins -= .25;
              }
              else if (CurrentColor == "Red"){
                spins += .25;
              }
              break;
            case 'r' :
              if (CurrentColor == "Green"){
                spins -= .25;
              }
              else if (CurrentColor == "Yellow"){
                spins += .25;
              }
              break;
            case 'y' :
              if (CurrentColor == "Red"){
                spins -= .25;
              }
              else if (CurrentColor == "Blue"){
                spins += .25;
              }
              break;
            default :
              break;
          }
          LastState = Character.toLowerCase(CurrentColor.charAt(0));
          if (spins > 3.0){
            LastState = 'f';
          }
        }
      }
    }
  }
    

  public boolean isFinished(){
    if (LastState == 'f'){
      ColorWheelSpinner.Spin(0.0);
      return true;
    }
    else{
      return false;
    }
  }
}
