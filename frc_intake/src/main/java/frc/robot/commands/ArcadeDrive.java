// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {

  private static final class Config{
      public static final int kLeftStickY = 1;
      public static final int kRightStickX=4;
      public static final double kSpeedMultiplier = 0.2;
      public static final double kTurnMultiplier = 0.5;
  }
  private Drivetrain m_drivetrain;
  private Joystick m_joystick;
  
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Joystick joystick, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_joystick = joystick;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -m_joystick.getRawAxis(Config.kLeftStickY) * Config.kSpeedMultiplier;
    double turn = -m_joystick.getRawAxis(Config.kRightStickX) * Config.kTurnMultiplier;
    double left = speed+turn;
    double right = speed-turn;
    m_drivetrain.setLeftSpeed(left);
    m_drivetrain.setRightSpeed(right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setLeftSpeed(0);
    m_drivetrain.setRightSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
