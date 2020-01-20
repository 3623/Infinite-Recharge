/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static int SPINNER_MOTOR = 0;
  public static int RIGHT_MOTOR_ONE = 1;
  public static int RIGHT_MOTOR_TWO = 2;
  public static int LEFT_MOTOR_ONE = 3;
  public static int LEFT_MOTOR_TWO = 4;

  public static int SHIFTER_SOLENOID = 0;

  public static int DRIVER_CONTROLLER = 0;
  public static int OPERATOR_CONTROLLER = 1;
  public static int ENCODER_LEFT_A = 0;
  public static int ENCODER_LEFT_B = 1;
  public static int ENCODER_RIGHT_A = 2;
  public static int ENCODER_RIGHT_B = 3;

  public static double WHEEL_RADIUS = 2.0;
}
