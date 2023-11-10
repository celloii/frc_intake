// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.commands.ArcadeDrive;

/** Add your docs here. */
public class Drivetrain {
    private static final class Config{
        public static final int kRightPrimaryID = 3;
        public static final int kRightSecondaryID = 4;
        public static final int kLeftPrimaryID=1;
        public static final int kLeftSecondaryID=2;
      }
    private WPI_TalonFX m_rightPrimary = new WPI_TalonFX(Config.kRightPrimaryID);
    private WPI_TalonFX m_rightSecondary = new WPI_TalonFX(Config.kRightSecondaryID);
    private WPI_TalonFX m_leftPrimary = new WPI_TalonFX(Config.kLeftPrimaryID);
    private WPI_TalonFX m_leftSecondary = new WPI_TalonFX(Config.kLeftSecondaryID);

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
      // This method will be called once per scheduler run
    }

    public void getDefaultCommand(ArcadeDrive m_arcadeDrive) {}
  

}
