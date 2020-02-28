/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivetrainConstants{
        public static int RIGHT_MOTOR_ONE = 1;
        public static int RIGHT_MOTOR_TWO = 2;
        public static int LEFT_MOTOR_ONE = 3;
        public static int LEFT_MOTOR_TWO = 4;
        public static int ENCODER_LEFT_A = 0;
        public static int ENCODER_LEFT_B = 1;
        public static int ENCODER_RIGHT_A = 2;
        public static int ENCODER_RIGHT_B = 3;
    }

    public static final class ShooterConstants {
        public static int SHOOTER_LEFT_MOTOR_NEO = 1;
        public static int SHOOTER_RIGHT_MOTOR_NEO = 2;
        public static int SHOOTER_TURRET_MOTOR_SRX = 5;
        public static int SHOOTER_HOOD_MOTOR_SPX = 1;
        // Limelight Pipeline Constants
        public static int CONTROL_PANEL_REAR_PIPELINE = 0;
        public static int INITITATION_LINE_PIPELINE = 1;
        public static int CONTROL_PANEL_FRONT_PIPELINE = 2;
        public static int END_OF_TRENCH_PIPELINE = 3;
        public static int POINT_BLANK_RANGE_PIPELINE = 4;
        // Limelight LED Settings Definitions
        public static int LIMELIGHT_LED_DEFAULT = 0;
        public static int LIMELIGHT_LED_FORCE_OFF = 1;
        public static int LIMEIGHT_LED_BLINK = 2;
        public static int LIMELIGHT_LED_FORCE_ON = 3;
    }

    public static final class IntakeConstants {
        public static int INTAKE_COLLECTOR_MOTOR_SPX = 4;
        public static int INTAKE_DROP_SOLENOID = 6;
        public static int INTAKE_ELEVATOR_MOTOR_SRX = 6;
    }

    public static final class SpinnerConstants {
        public static int SPINNER_MOTOR_SPX = 2;
    }

    public static final class ShifterConstants {
        public static int SHIFTER_SOLENOID = 1;
    }

    public static final class ClimberConstants {
        public static int CLIMBER_LOCK_SOLENOID_LEFT = 2;
        public static int CLIMBER_PTO_SOLENOID_LEFT = 4;
        public static int CLIMBER_LOCK_SOLENOID_RIGHT = 3;
        public static int CLIMBER_PTO_SOLENOID_RIGHT = 5;
    }

    public static final class IOConstants {
        public static int DRIVER_CONTROLLER = 0;
        public static int OPERATOR_CONTROLLER = 1;
    }

}
