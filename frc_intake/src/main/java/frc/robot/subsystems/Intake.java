// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Intake extends SubsystemBase{
    private static final class Config{
        private static final int deviceNumber = 0;

    }
    private WPI_TalonFX m_intake = new WPI_TalonFX(Config.deviceNumber);

    public Intake(){

    }
    public void setSpeed(double speed){
        m_intake.set(speed);

    }

    //method
    
    @Override
    public void periodic(){

    }
    public void schedule() {
    }

}
