package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Hood extends PIDSubsystem {
    WPI_VictorSPX motor;

    private Encoder encoder;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DISTANCE_PER_PULSE = 1.0 / TICKS_PER_ENCODER_REV * 24.0 / 40.5 * 45.0;

    private double MAX_GOAL = 35.0;
    private double MIN_GOAL = 0.0;

    private static final double kP = 0.65 / 35.0;
    private static final double kI = kP / 1000.0;
    private static final double kD = kP * 0.1;
    private static final double DEADBAND = 1.5;

    public Hood() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(DEADBAND);

        motor = new WPI_VictorSPX(Constants.ShooterConstants.SHOOTER_HOOD_MOTOR_SPX);
        motor.setNeutralMode(NeutralMode.Brake);
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public void monitor() {
        SmartDashboard.putNumber("Hood Angle", this.getMeasurement());
        SmartDashboard.putNumber("Hood Output", (int) motor.getMotorOutputPercent() * 100);
        SmartDashboard.putNumber("Hood Error", getController().getPositionError());
    }

    public void setPosition(double position) {
        if (position > MAX_GOAL) {
            position = MAX_GOAL;
        } else if (position < MIN_GOAL) {
            position = MIN_GOAL;
        }
        setSetpoint(position);
    }

    public void setRelative(double offSet) {
        setPosition(this.getMeasurement() + offSet);
    }

    public Boolean atTarget() {
        return getController().atSetpoint();
    }

    public void zero() {
        encoder.reset();
    }

    public void stop() {
        motor.disable();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMeasurement() {
        return encoder.getDistance();
    }
}
