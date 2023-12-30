// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveStraightUsingGyro extends Command {
  /** Creates a new DriveStraightUsingGyro. */
  private Drivetrain m_drivetrain;
  private double m_targetDirection;
  private double m_speed;
  public DriveStraightUsingGyro(double speed, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetDirection = m_drivetrain.getGyroAngleZ();
  }

  private double diffDegrees(double x, double y) {
    double diff = x - y;
    while (diff < -180) diff += 180;
    while (diff >= 180) diff -= 180;
    return diff;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDirection = m_drivetrain.getGyroAngleZ();
    double angularError = diffDegrees(currentDirection, m_targetDirection);
    double rotationRate = angularError / 45; // 45 is a fudge factor
    rotationRate = Math.min(1, Math.max(-1, rotationRate));
    m_drivetrain.arcadeDrive(m_speed, -rotationRate);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
