// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeMoveWithPID extends CommandBase {

  private static final class Config{
    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double ticksPerFoot = 4096/Math.PI;
    public static final double kMotorSpeed = 0.4;
  }
  private Drivetrain m_drivetrain;
  private double m_distance;
  private double m_leftStartPosition;
  private PIDController m_PID = new PIDController(Config.kP, Config.kI, Config.kD);

  /** Creates a new ArcadeMoveWithPID. */
  public ArcadeMoveWithPID(Drivetrain drivetrain, double distance) {
    m_drivetrain = drivetrain;
    m_distance = distance*Config.ticksPerFoot;
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftStartPosition = m_drivetrain.getLeftTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_PIDspeed = m_PID.calculate(m_drivetrain.getLeftTicks() - m_leftStartPosition, m_distance);
    m_drivetrain.setLeftSpeed(-Config.kMotorSpeed*m_PIDspeed);
    m_drivetrain.setRightSpeed(-Config.kMotorSpeed*m_PIDspeed);
    System.out.println(m_PIDspeed);
    System.out.println(m_drivetrain.getLeftTicks() - m_leftStartPosition);
    System.out.println(m_distance);
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
    if(Math.abs((m_drivetrain.getLeftTicks() - m_leftStartPosition - m_distance)) < 0.0002){
      return true;
    }
    else{
      return false;
    }
  }
}
