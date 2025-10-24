// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX; 

public class ShooterSubSystem extends SubsystemBase {
  private TalonFX shooterMotor;
  /** Creates a new ShooterSubSystem. */
  public ShooterSubSystem() {
    shooterMotor = new TalonFX(Constants.ShooterSubSystemConstants.CAN_ID_SHOOTER);
    
  }

  public void setSpeed(double speed){
    shooterMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
