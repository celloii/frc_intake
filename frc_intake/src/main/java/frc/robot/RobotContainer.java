// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArcadeIntake;
import frc.robot.commands.ArcadeMoveWithPID;
import frc.robot.commands.ArcadeTurnWithPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer{
  // The robot's subsystems and commands are defined here...
  private static final class Config{
    public static final int k_JoystickPort = 0;
}
private Joystick m_joystick = new Joystick(Config.k_JoystickPort);
  private Drivetrain m_drivetrain = new Drivetrain();
  private Intake m_intake = new Intake();
  private ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_joystick,m_drivetrain);
  private ArcadeIntake m_arcadeIntake = new ArcadeIntake(m_joystick,m_intake);
  private SequentialCommandGroup m_sequentialCommandGroup = new SequentialCommandGroup();


  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_drivetrain.getDefaultCommand(m_arcadeDrive);
    m_arcadeIntake.schedule();
    m_sequentialCommandGroup.addCommands(new ArcadeTurnWithPID(Math.PI, m_drivetrain, Math.PI/2));
    
    return null;
  }
}
