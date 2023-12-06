// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArcadeDrive;

/** Add your docs here. */
public class Drivetrain extends SubsystemBase{
    private static final class Config{
        public static final int kRightPrimaryID = 3;
        public static final int kRightSecondaryID = 7;
        public static final int kLeftPrimaryID=6;
        public static final int kLeftSecondaryID=8;
      }
    private WPI_TalonFX m_rightPrimary = new WPI_TalonFX(Config.kRightPrimaryID);
    private VictorSPX m_rightSecondary = new VictorSPX(Config.kRightSecondaryID);
    private WPI_TalonFX m_leftPrimary = new WPI_TalonFX(Config.kLeftPrimaryID);
    private VictorSPX m_leftSecondary = new VictorSPX(Config.kLeftSecondaryID);

    public Drivetrain() {
      m_leftSecondary.follow(m_leftPrimary);
      m_rightSecondary.follow(m_rightPrimary);
      
      m_rightPrimary.setInverted(true);
      m_rightSecondary.setInverted(true);
    }

    public void setRightSpeed(double rightSpeed){
      m_rightPrimary.set(rightSpeed);
    }
  
    public void setLeftSpeed(double leftSpeed){
      m_leftPrimary.set(leftSpeed);
    }
  
    public void setIdle(NeutralMode idleMode){
      m_rightPrimary.setNeutralMode(idleMode);
      m_rightSecondary.setNeutralMode(idleMode);
      m_leftPrimary.setNeutralMode(idleMode);
      m_leftSecondary.setNeutralMode(idleMode);
    }
  
    public double getRightTicks(){
      return m_rightPrimary.getSelectedSensorPosition();
  
    }
    public double getLeftTicks(){
      return m_leftPrimary.getSelectedSensorPosition();
    }
    
    //@Override
    public void periodic() {
      SmartDashboard.putNumber("left", m_leftPrimary.get());
      SmartDashboard.putNumber("right", m_rightPrimary.get());
    }
  

}
