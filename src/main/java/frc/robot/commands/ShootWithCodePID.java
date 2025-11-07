// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootWithCodePID extends Command {
  ShooterSubSystem shooter;
  PIDController pid;
  private double targetFlywheelRPM;

  /** Creates a new ShootWithCodePID. */
  public ShootWithCodePID(ShooterSubSystem shooter, double targetFlywheelRPM) {
    this.shooter=shooter;
    this.targetFlywheelRPM = targetFlywheelRPM;

    addRequirements(shooter);

    //UNTUNED PID LOOP
    pid = new PIDController(0.005, 0, 0);
    pid.setSetpoint(targetFlywheelRPM);
    pid.setTolerance(50);
        // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentMotorRPM = shooter.getRPM();

    SmartDashboard.putNumber("Current Motor RPM", currentMotorRPM);

    double output = pid.calculate(currentMotorRPM, targetFlywheelRPM);

    //clampingg
    output = Math.max(-1, Math.min(1, output));

    shooter.setSpeed(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
