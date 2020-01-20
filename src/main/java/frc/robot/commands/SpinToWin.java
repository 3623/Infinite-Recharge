/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

/**
 * An example command.  You can replace me with your own command.
 */
public class SpinToWin extends CommandBase {
  private final Spinner ColorWheelSpinner;
  private final String ColorData;
  private char LastState;
  private double spins;
  public SpinToWin(Spinner colorSpinner, String data){
    ColorWheelSpinner = colorSpinner;
    addRequirements(ColorWheelSpinner);
    ColorData = data;
  }

  public void initialize() {
    spins = 0.0;
    if (ColorData.length() > 0){
      
    }
    else {
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
  }

  public void execute(){
    if (LastState != 'f'){
      if (ColorData.length() > 0){

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
      return true;
    }
    else{
      return false;
    }
  }
}
