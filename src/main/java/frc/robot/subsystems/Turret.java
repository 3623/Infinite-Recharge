package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;


// TODO Switch this to motion magic!!
// or feedforward with current limiting for smooth motion
public class Turret extends PIDSubsystem {
    WPI_TalonSRX turretMotor;

    private Encoder turretEncoder;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double ENCODER_REVS_PER_TURRET_REV = 196.0 / 18.0;
    private static final double DISTANCE_PER_PULSE = 360.0 / ENCODER_REVS_PER_TURRET_REV / TICKS_PER_ENCODER_REV;

    private double MAX_GOAL = 180.0;
    private double MIN_GOAL = -180.0;

    private static final double kP = 0.65 / 18.0;
    private static final double kI = kP / 500.0;
    private static final double kD = kP * 0.1;
    private static final double DEADBAND = 0.25;

    public Turret() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(DEADBAND);

        turretMotor = new WPI_TalonSRX(Constants.Shooter.SHOOTER_TURRET_MOTOR_SRX);
        turretEncoder = new Encoder(0, 1);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public void monitor() {
        SmartDashboard.putNumber("Turret Angle", this.getMeasurement());
        SmartDashboard.putNumber("Turret Output", (int) turretMotor.getMotorOutputPercent() * 100);
        SmartDashboard.putNumber("Turret Error", getController().getPositionError());
    }

    public void zero() {
        turretEncoder.reset();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        turretMotor.set(ControlMode.PercentOutput, output);
    }

    public void runwithOutput(double output) {
        turretMotor.set(output);
    }

    @Override
    protected double getMeasurement() {
        return turretEncoder.getDistance();
    }
}
