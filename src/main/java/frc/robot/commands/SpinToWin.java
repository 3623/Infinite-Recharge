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


public class SpinToWin extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Spinner ColorWheelSpinner;
  private final String ColorData;
  private char targetColor;
  private char LastState;
  private double spins;

  /* Constructor of The Command
      Arguments - Spinner Subsystem
  */
  public SpinToWin(Spinner colorSpinner){
    ColorWheelSpinner = colorSpinner;
    addRequirements(ColorWheelSpinner);
    ColorData = DriverStation.getInstance().getGameSpecificMessage();
  }

  // Initialize is called immediately when the command is scheduled.
  public void initialize() {
    spins = 0.0;
    if (ColorData.length() > 0){ // If there is Color Data, we are in Stage 3 for the Color Wheel.
      switch(ColorData.charAt(0)){ // By choosing the opposite color on the wheel as our target,
        case 'b' :                // we know that the color that we want is under the field sensor.
          targetColor = 'r';      // Red is Opposite Blue, Yellow is Opposite Green.
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
          LastState = 'f'; // Defaults to f if color data is corrupt, failed and will end the command after the first run.
      }
    }
      String ColorSeen = ColorWheelSpinner.getColorMatch(); // Figure out where we are on the wheel right now.
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
      ColorWheelSpinner.Spin(0.5); // Start the spinner
    }

  public void execute(){
    if (LastState != 'f'){
      if (ColorData.length() > 0){ // Stage 3 Execution. Continue until we get what we want.
        String CurrentColor = ColorWheelSpinner.getColorMatch();
        if (Character.toLowerCase(CurrentColor.charAt(0)) == targetColor){
          LastState = 'f'; // End the command if we're where we want to be.
        }
      }
      else { // Stage 2 Execution. Use the Color Sensor to determine how many spins we've done. 
        String CurrentColor = ColorWheelSpinner.getColorMatch(); // TODO: Implement Encoder Tracking for this, Color Sensor Backup
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
            LastState = 'f'; // If we've reached the requisite number of spins, end the command
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
