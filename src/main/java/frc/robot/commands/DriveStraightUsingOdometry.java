// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveStraightUsingOdometry extends Command {
  /** Creates a new DriveStraight. */
  private double m_speed;
  private Drivetrain m_drivetrain;
  private double m_kP;

  public DriveStraightUsingOdometry(double speed, double kP, Drivetrain drivetrain) {
    m_speed = speed;
    m_kP = kP;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.initiateDriveStraightUsingOdometry(m_kP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveStraightUsingOdometry(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}