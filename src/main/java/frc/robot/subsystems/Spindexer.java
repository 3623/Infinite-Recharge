/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
* An example subsystem. You can replace me with your own Subsystem.
*/
public class Spindexer extends SubsystemBase {
    public static final double SHOOT_TIME = 1.0;
    public static final double INDEX_TIME = 1.0;
    private static final double INDEX_SPEED = -0.5;
    private static final double SHOOT_SPEED = 1.0;
    private boolean indexing = false;
    private boolean shooting = false;
    private WPI_TalonSRX spindexerSRX;

    // private DigitalInput ballSensor1, ballSensor2, ballSensor3, ballSensor4,
    // ballSensor5;

    public Spindexer() { // TODO this should be the spindexer motor
        spindexerSRX = new WPI_TalonSRX(Constants.Shooter.FEEDER_MOTOR_SRX);
    }

    /**
     * @param indexing True if trying to shoot
     */
    public void setIndexing(boolean indexing) {
        this.indexing = indexing;
        chooseOutput();
    }

    /**
     * @param shooting True if trying to shoot
     */
    public void setShooting(boolean shooting) {
        this.shooting = shooting;
        chooseOutput();
    }

    private void chooseOutput() {
        if (shooting) setSpinning(SHOOT_SPEED);
        else if (indexing) setSpinning(INDEX_SPEED);
        else setSpinning(0.0);
    }

    private void setSpinning(double feederSpeed) {
        spindexerSRX.set(ControlMode.PercentOutput, feederSpeed);
    }
}
