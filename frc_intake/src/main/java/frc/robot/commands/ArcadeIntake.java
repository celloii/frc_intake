// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Intake;

public class ArcadeIntake extends CommandBase {
  /** Creates a new ArcadeIntake. */
  private static final class Config{
    public static final int kJoystickButton = 1;
  }

  Intake m_intake;
  Joystick m_joystick;
  JoystickButton m_joystickButton;

  public ArcadeIntake(Joystick joystick, Intake intake) {
    m_intake = intake;
    m_joystick = joystick;
    m_joystickButton = new JoystickButton(m_joystick, Config.kJoystickButton);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_joystickButton.whileTrue(m_intake.goForward());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
