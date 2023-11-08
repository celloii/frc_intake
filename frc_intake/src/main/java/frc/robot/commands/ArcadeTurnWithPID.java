// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeTurnWithPID extends CommandBase {
  private static final class Config{
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double ticksPerFoot = 1024*Math.PI;
    public static final double kMotorSpeed = 0.4;
    public static final double robotWidth = 2.25;
  }

  private Drivetrain m_drivetrain;
  private PIDController m_pid = new PIDController(Config.kP,Config.kI,Config.kD);
  private double m_distance;
  private double m_leftStartPosition;
  private double m_angle;

  /** Creates a new ArcadeTurnWithPID. */
  public ArcadeTurnWithPID(double distance, Drivetrain drivetrain, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_distance = distance;
    m_drivetrain = drivetrain;
    m_angle = angle;
  }

  private double m_radius = m_distance/m_angle;
  private double m_rightMotorGoal = m_angle*(m_radius + Config.robotWidth/2)*Config.ticksPerFoot;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftStartPosition = m_drivetrain.getLeftTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_PIDspeed = m_pid.calculate(m_drivetrain.getRightTicks() - m_leftStartPosition, m_rightMotorGoal);
    m_drivetrain.setLeftSpeed(((m_radius - Config.robotWidth/2)/(m_radius + Config.robotWidth/2))*Config.kMotorSpeed*m_PIDspeed);
    m_drivetrain.setRightSpeed(Config.kMotorSpeed*m_PIDspeed);
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
    return Math.abs(m_drivetrain.getLeftTicks()-m_leftStartPosition) < 0.0002;
  }
}
