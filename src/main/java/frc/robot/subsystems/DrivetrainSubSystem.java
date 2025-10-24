// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubSystem extends SubsystemBase {
  public TalonFX driveLeft;
  public TalonFX driveRight;
  private final DifferentialDrive robotDrive =
  new DifferentialDrive(driveLeft::set, driveRight::set);
  

  /** Creates a new DrivetrainSubSystem. */
  public DrivetrainSubSystem() {
    driveLeft = new TalonFX(Constants.DrivetrainSubSystemConstants.CAN_ID_DRIVE_LEFT);
    driveRight = new TalonFX(Constants.DrivetrainSubSystemConstants.CAN_ID_DRIVE_RIGHT);

    driveRight.setInverted(true);
  }

  public void arcadeDrive(double fwd, double rot){
    robotDrive.arcadeDrive(fwd, rot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
