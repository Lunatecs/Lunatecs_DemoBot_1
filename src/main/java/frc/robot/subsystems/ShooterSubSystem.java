// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX; 
import static edu.wpi.first.units.Units.*;

public class ShooterSubSystem extends SubsystemBase {
  private TalonFX shooterMotor;
  //12 divided by 24
  private double beltRatio = 0.5;

  VelocityTorqueCurrentFOC velocityTorque;
  TalonFXConfiguration configs = new TalonFXConfiguration();
  /** Creates a new ShooterSubSystem. */
  public ShooterSubSystem() {
    shooterMotor = new TalonFX(Constants.ShooterSubSystemConstants.CAN_ID_SHOOTER);


    velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);

    configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot1.kP = 0.005; 
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));
    
  }

  public void setSpeed(double speed){
    shooterMotor.set(speed);
  }

  public double getRPM(){
      double rotatepersec = shooterMotor.getVelocity().getValueAsDouble();

      double rotateperminutewithratio = rotatepersec * 60.0 * beltRatio;
  
      return rotateperminutewithratio; 
  }

  public void setRPM(double desiredRPM){
    shooterMotor.setControl(velocityTorque.withVelocity(desiredRPM)); 
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
