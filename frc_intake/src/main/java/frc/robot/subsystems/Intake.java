// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Intake extends SubsystemBase{
    private static final class Config{
        private static final int deviceNumber = 0;
    }
    private CANSparkMax m_intake = new CANSparkMax(Config.deviceNumber, MotorType.kBrushless);

    public Intake(){

    }
    public void setForward(){
        m_intake.set(1);
    }
    public void setBackward(){
        m_intake.set(-1);
    }
    public void setSpeed(double speed){
        m_intake.set(speed);
    }
    public InstantCommand goForward(){
        return new InstantCommand(this::setForward, this);
    }
    public InstantCommand goBackward(){
        return new InstantCommand(this::setBackward, this);
    }
    //method
    
    @Override
    public void periodic(){

    }
    public void schedule() {
    }

}
