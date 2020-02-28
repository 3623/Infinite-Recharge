package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Turret extends SubsystemBase {
    WPI_TalonSRX turretMotor;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double ENCODER_REVS_PER_TURRET_REV = 18.0 / 196.0;

    private double MAX_GOAL = 180.0;
    private double MIN_GOAL = -180.0;

    private static final double kP = 1023.0 * 0.65 / (double) degreesToTalonUnits(360.0);
    private static final double kI = kP / 1000.0;
    private static final double kD = 0.01;
    private static final double DEADBAND = 1.5;

    private Drivetrain dt;

    private double setAngle = 0.0;

    public Turret() {
        turretMotor = new WPI_TalonSRX(Constants.ShooterConstants.SHOOTER_TURRET_MOTOR_SRX);
        turretMotor.configFactoryDefault();
        turretMotor.setInverted(false);
        turretMotor.setNeutralMode(NeutralMode.Brake);

        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        turretMotor.configForwardSoftLimitThreshold(degreesToTalonUnits(MAX_GOAL), 0);
        turretMotor.configReverseSoftLimitThreshold(degreesToTalonUnits(MIN_GOAL), 0);
        turretMotor.configForwardSoftLimitEnable(true, 0);
        turretMotor.configReverseSoftLimitEnable(true, 0);

        turretMotor.configAllowableClosedloopError(0, (int) degreesToTalonUnits(DEADBAND));
        turretMotor.config_kF(0, 0.0);
        turretMotor.config_kP(0, kP);
        turretMotor.config_kD(0, kD);
        turretMotor.config_kI(0, kI);

        dt = new Drivetrain();
    }

    public void monitor() {
        SmartDashboard.putNumber("Turret Angle", turretMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Turret Output", (int) turretMotor.getMotorOutputPercent() * 100);
        SmartDashboard.putNumber("Turret Error", turretMotor.getClosedLoopError());
    }

    private static int degreesToTalonUnits(double degrees) {
        double turretRevs = degrees / 360.0;
        double encoderRevs = turretRevs * ENCODER_REVS_PER_TURRET_REV;
        double encoderTicks = encoderRevs * TICKS_PER_ENCODER_REV;
        return (int) encoderTicks; // BANANA check this
    }

    public Boolean setAngle(double angle) {
        setAngle = degreesToTalonUnits(angle);
        turretMotor.set(ControlMode.Position, setAngle);
        return atTarget();
    }

    public Double getAngle() {
        return setAngle; // BANANA TODO
    }

    private Boolean atTarget() {
        return (turretMotor.getClosedLoopError() < degreesToTalonUnits(DEADBAND));
    }

    public void zero() {
        turretMotor.setSelectedSensorPosition(0);
    }

    public void stop() {
        turretMotor.disable();
    }
}
