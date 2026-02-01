// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.LoggingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final LoggingSubsystem m_loggingSubsystem = new LoggingSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_loggingSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_loggingSubsystem));
   
    //command for getting raw boolean values for each button on controller (acts as a connector from RC to subsystem)
    
    m_loggingSubsystem.setDefaultCommand(m_loggingSubsystem.buttonsPressed(() -> m_driverController.a().getAsBoolean(), 
                                                                           () -> m_driverController.y().getAsBoolean(), 
                                                                           () -> m_driverController.b().getAsBoolean(), 
                                                                           () -> m_driverController.x().getAsBoolean(), 
                                                                           () -> m_driverController.leftBumper().getAsBoolean(), 
                                                                           () -> m_driverController.rightBumper().getAsBoolean(), 
                                                                           () -> m_driverController.leftTrigger().getAsBoolean(), 
                                                                           () -> m_driverController.rightTrigger().getAsBoolean()));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_loggingSubsystem);
  }
}
