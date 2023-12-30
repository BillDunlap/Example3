// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
/**
 * Command to move by giving the two motors the same power.
 * Use no odometry, gyro, and any other sort of feedback.
 */
public class DriveStraightWithNoFeedback extends Command {
  private Drivetrain m_drivetrain;
  private double m_speed;
  /**
   * Create a new DriveStraightWithNoFeedback command.
   * @param speed power to give to motors, on a scale of -1 to +1.
   * @param drivetrain the robot's drivetrain subsystem.
   */
  public DriveStraightWithNoFeedback(double speed, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
