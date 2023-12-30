// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
/**
 * A command to move forward, stopping at 'inches' inches from a wall.
 * This uses the ultrasonice distance sensor.  It is considered finished
 * when it is within inchesTolerance of the desired distance and its
 * speed is no more than m_speedTolerance.
 */
public class GoUpToWall extends Command {
  private double m_inches;
  // private double m_inchesTolerance = 1;
  // private double m_speedTolerance = .5;

  Drivetrain m_driveTrain;
  XRPRangefinder m_rangefinder;

  /** Creates a new GoUpToWall. */
  public GoUpToWall(double inches, Drivetrain driveTrain, XRPRangefinder rangefinder) {
    m_inches = inches;
    m_driveTrain = driveTrain;
    m_rangefinder = rangefinder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    double inchesToWall = m_rangefinder.getDistanceInches();
    /* Note that rangefinder reports 157.48031496062993 inches (== 4 meters)
     * if it cannot sense a wall in front of it.
     */
    SmartDashboard.putNumber("Inches to wall", inchesToWall);
    // System.out.println("-- inches to wall -->" + inchesToWall);
    double inchesToGo = inchesToWall - m_inches;
    double minMotorSpeed = 0.4; // minimum 'speed' which makes motors turn
    double speed = Math.abs(inchesToGo) * (1.0 - minMotorSpeed)/(2.0 - 0.0) + minMotorSpeed;
    speed = Math.min(+1.0, speed);
    speed = Math.signum(inchesToGo) * speed;
    SmartDashboard.putNumber("speed", speed);
    m_driveTrain.arcadeDrive(speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
