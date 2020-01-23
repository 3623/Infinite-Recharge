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
    public static final class DrivetrainConstants {
        public static int RIGHT_MOTOR_ONE = 1;
        public static int RIGHT_MOTOR_TWO = 2;
        public static int LEFT_MOTOR_ONE = 3;
        public static int LEFT_MOTOR_TWO = 4;
        public static double WHEEL_RADIUS = 2.0; // this is wrong, wheel radius is in m for the
                                                 // purpose of odometry
                                                 // also in general i dont like defining constants
                                                 // like this as i found it to be unneccesary
                                                 // and even more obscure, especially for something
                                                 // like the drivetrain which is very independent
        public static int ENCODER_LEFT_A = 0;
        public static int ENCODER_LEFT_B = 1;
        public static int ENCODER_RIGHT_A = 2;
        public static int ENCODER_RIGHT_B = 3;
    }

    public static final class ShooterConstants {

    }

    public static final class IntakeConstants {

    }

    public static final class SpinnerConstants {
        public static int SPINNER_MOTOR = 0;
    }

    public static final class ShifterConstants {
        public static int SHIFTER_SOLENOID = 0;
    }

    public static final class ClimberConstants {

    }

    public static final class IOConstants {
        public static int DRIVER_CONTROLLER = 0;
        public static int OPERATOR_CONTROLLER = 1;
    }

}
