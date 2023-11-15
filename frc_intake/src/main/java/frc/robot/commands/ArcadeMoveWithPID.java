// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeMoveWithPID extends CommandBase {

  private static final class Config{
    public static final double kP = 0.005;
    public static final double kI = 0;
    public static final double kD = 0.001;
    public static final double ticksPerFoot = 1024*Math.PI;
    public static final double kGearboxReduction = 10.65;
  }
  private Drivetrain m_drivetrain;
  private double m_distance;
  private double m_leftStartPosition;
  private PIDController m_PID = new PIDController(Config.kP, Config.kI, Config.kD);

  /** Creates a new ArcadeMoveWithPID. */
  public ArcadeMoveWithPID(Drivetrain drivetrain, double distance) {
    m_drivetrain = drivetrain;
    m_distance = distance*Config.ticksPerFoot/Config.kGearboxReduction;
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftStartPosition = m_drivetrain.getLeftTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_PIDspeed = m_PID.calculate(m_drivetrain.getRightTicks() - m_leftStartPosition, m_distance);
    m_drivetrain.setLeftSpeed(m_PIDspeed);
    m_drivetrain.setRightSpeed(m_PIDspeed);
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
    if(Math.abs((m_drivetrain.getLeftTicks() - m_leftStartPosition - m_distance)) < 200){
      return true;
    }
    else{
      return false;
    }
  }
}
