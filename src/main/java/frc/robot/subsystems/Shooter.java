/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final int UPDATE_RATE = 200;
  private CANSparkMax shooterLeft, shooterRight;
  private TalonSRX turret;
  private Encoder turretEnc;

  private boolean targetAcquired;

  /* Shooter States:
      0: Idle. Waiting for Commands
      1: Revving Up. Running Motors to Prepare for shooting
      2: Ready to Fire. At speed, and target acquired.
      3: Revving Down. Motor Output to 0, status 0 when stopped.
  */
  private int state;

  NetworkTable Lime = NetworkTableInstance.getDefault().getTable("limelight"); // The Limelight Vision system posts several useful bits
                                                                              // of data to Network Tables.
		NetworkTableEntry tx = Lime.getEntry("tx"); // Horizontal Offset From Crosshair to Target (-27 to 27 degrees)
  	NetworkTableEntry ty = Lime.getEntry("ty"); // Vertical Offset From Crosshair to Target (-20.5 to 20.5 degrees)
  	NetworkTableEntry ta = Lime.getEntry("ta"); // Target Area (0% of Image to 100% of Image)
		NetworkTableEntry tv = Lime.getEntry("tv"); // Valid Targets (0 or 1, False/True)
    double x,y,area,valid;

  public Shooter(){
    shooterLeft = new CANSparkMax(1, MotorType.kBrushless);
    shooterRight = new CANSparkMax(2, MotorType.kBrushless);
    turret = new TalonSRX(1);
    turretEnc = new Encoder(4, 5);

    this.updateThreadStart();
  }

  public void runShooter(double speed){
    shooterLeft.set(speed);
    shooterRight.set(-speed);
  }

  public void runTurret(double speed){
    turret.set(ControlMode.PercentOutput, speed);
  }

  public int getState(){
    return state;
  }

  public boolean isTargetAcquired(){
    return targetAcquired;
  }

  public void setTargetAcquired(boolean isAcquired){
    targetAcquired = isAcquired;
  }

  public void setState(int stateToSet){
    state = stateToSet;
  }

  private void updateThreadStart() {
		Thread t = new Thread(() -> {
			while (!Thread.interrupted()) {
				this.update();
				try {
					Thread.sleep(1000 / UPDATE_RATE);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
		t.start();
	}

  private void update(){
    x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
    valid = tv.getDouble(0.0);
    this.monitor();
  }

  private void monitor(){

  }
}
