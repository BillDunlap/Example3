// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
/**
 * Print the axis (joystick and "trigger") values from the Xboxcontroller.
 * This is so we can estimate their dead zones.
 */
public class PrintJoystickValues extends Command {
  /** Creates a new PrintJoystickValues. */
  private XboxController m_xboxController;
  public PrintJoystickValues(XboxController xboxController) {
    m_xboxController = xboxController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("% LeftJoystickX, LeftJoystickY, RightJoystickX, RightJoystickY, LeftTrigger, RightTrigger");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("%" + 
        m_xboxController.getLeftX() + "," +
        m_xboxController.getLeftY() + "," +
        m_xboxController.getRightX() + "," +
        m_xboxController.getRightY() + "," +
        m_xboxController.getLeftTriggerAxis() + "," +
        m_xboxController.getRightTriggerAxis());
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
