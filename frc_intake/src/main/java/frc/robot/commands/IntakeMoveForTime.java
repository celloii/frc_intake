// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeMoveForTime extends CommandBase {
  
  private Intake m_intake;
  private double m_speed;
  private double m_timeInSeconds;
  private double m_startTime;
  private Timer m_timer;
  /** Creates a new IntakeMoveForTime. */
  public IntakeMoveForTime(Intake intake, double speed, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_speed = speed;
    m_timeInSeconds = seconds;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = m_timer.get();
    m_timer.start();
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_timer.get() - m_startTime) <= m_timeInSeconds){
      m_intake.setSpeed(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get()-m_startTime > m_timeInSeconds;
    
  
  }
}
