// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeMoveWithPID extends CommandBase {

  private static final class Config{
    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double ticksPerFoot = 15000/Math.PI;
    public static final double kMotorSpeed = 0.4;
  }
  private Drivetrain m_drivetrain;
  private double m_distance;
  private double m_leftStartPosition;
  private double m_rightStartPosition;
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
    m_rightStartPosition = m_drivetrain.getRightTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_PIDRightspeed = m_PID.calculate(m_drivetrain.getRightTicks() - m_rightStartPosition, m_distance);
    double m_PIDLeftSpeed = m_PID.calculate(m_drivetrain.getLeftTicks() - m_leftStartPosition, m_distance);
    /**m_drivetrain.setRightSpeed(-Config.kMotorSpeed*m_PIDRightspeed);
    m_drivetrain.setLeftSpeed(-Config.kMotorSpeed*m_PIDLeftSpeed);**/
    SmartDashboard.putNumber("speed", m_PIDRightspeed);
    SmartDashboard.putNumber("leftTicks", m_drivetrain.getLeftTicks() - m_leftStartPosition);
    SmartDashboard.putNumber("rightTicks", m_drivetrain.getRightTicks() - m_rightStartPosition);
    SmartDashboard.putNumber("goal", m_distance);
    m_drivetrain.setRightSpeed(-0.2);
    m_drivetrain.setLeftSpeed(-0.2);

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
