package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

// public class Turret extends SubsystemBase {
//     WPI_TalonSRX turretMotor;
//     private static final double TICKS_PER_ENCODER_REV = 2048.0;
//     private static final double ENCODER_REVS_PER_TURRET_REV = 18.0 / 196.0;

//     private double MAX_GOAL = 180.0;
//     private double MIN_GOAL = -180.0;

//     private static final double kP = 1023.0 * 0.65 / (double) degreesToTalonUnits(360.0);
//     private static final double kI = kP / 1000.0;
//     private static final double kD = 0.01;
//     private static final double DEADBAND = 1.5;

//     private double setAngle = 0.0;

//     public Turret() {
//         turretMotor = new WPI_TalonSRX(Constants.ShooterConstants.SHOOTER_TURRET_MOTOR_SRX);
//         turretMotor.configFactoryDefault();
//         turretMotor.setInverted(false);
//         turretMotor.setNeutralMode(NeutralMode.Brake);

//         turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
//         turretMotor.configForwardSoftLimitThreshold(degreesToTalonUnits(MAX_GOAL), 0);
//         turretMotor.configReverseSoftLimitThreshold(degreesToTalonUnits(MIN_GOAL), 0);
//         turretMotor.configForwardSoftLimitEnable(true, 0);
//         turretMotor.configReverseSoftLimitEnable(true, 0);

//         turretMotor.configAllowableClosedloopError(0, (int) degreesToTalonUnits(DEADBAND));
//         turretMotor.config_kF(0, 0.0);
//         turretMotor.config_kP(0, kP);
//         turretMotor.config_kD(0, kD);
//         turretMotor.config_kI(0, kI);
//     }

//     public void monitor() {
//         SmartDashboard.putNumber("Turret Angle", turretMotor.getSelectedSensorPosition());
//         SmartDashboard.putNumber("Turret Output", (int) turretMotor.getMotorOutputPercent() * 100);
//         SmartDashboard.putNumber("Turret Error", turretMotor.getClosedLoopError());
//     }

//     private static int degreesToTalonUnits(double degrees) {
//         double turretRevs = degrees / 360.0;
//         double encoderRevs = turretRevs * ENCODER_REVS_PER_TURRET_REV;
//         double encoderTicks = encoderRevs * TICKS_PER_ENCODER_REV;
//         return (int) encoderTicks; // BANANA check this
//     }

//     public Boolean setAngle(double angle) {
//         setAngle = angle;
//         turretMotor.set(ControlMode.Position, degreesToTalonUnits(setAngle));
//         return atTarget();
//     }

//     public Double getAngle() {
//         return setAngle; // BANANA TODO
//     }

//     private Boolean atTarget() {
//         return (turretMotor.getClosedLoopError() < degreesToTalonUnits(DEADBAND));
//     }

//     public void zero() {
//         turretMotor.setSelectedSensorPosition(0);
//     }

//     public void stop() {
//         turretMotor.disable();
//     }
// }

public class Turret extends PIDSubsystem {
    WPI_TalonSRX turretMotor;

    private Encoder turretEncoder;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double ENCODER_REVS_PER_TURRET_REV = 196.0 / 18.0;
    private static final double DISTANCE_PER_PULSE = 360.0 / ENCODER_REVS_PER_TURRET_REV / TICKS_PER_ENCODER_REV;

    private double MAX_GOAL = 180.0;
    private double MIN_GOAL = -180.0;

    private static final double kP = 0.65 / 180.0;
    private static final double kI = kP / 500.0;
    private static final double kD = kP * 0.1;
    private static final double DEADBAND = 1.5;

    private double setAngle = 0.0;

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

    private static int degreesToTalonUnits(double degrees) {
        double turretRevs = degrees / 360.0;
        double encoderRevs = turretRevs * ENCODER_REVS_PER_TURRET_REV;
        double encoderTicks = encoderRevs * TICKS_PER_ENCODER_REV;
        return (int) encoderTicks; // BANANA check this
    }

    public void setAngle(double angle) {
        if (angle > MAX_GOAL) {
            angle = MAX_GOAL;
        } else if (angle < MIN_GOAL) {
            angle = MIN_GOAL;
        }
        setAngle = angle;
        setSetpoint(angle);
    }

    public void setRelative(double offset) {
        setAngle(this.getMeasurement() + offset);
        if (Math.abs(offset) > 0.01)
            System.out.println(offset + " relative movement turret");
    }

    public Double getAngle() {
        return this.getMeasurement(); // BANANA TODO
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
