// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Util.SOTMSetpointGenerator;
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final Feeder kicker;
  public IntegerSubscriber scoringHeight;
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SOTMSetpointGenerator shooterInterpolationMap;




  private static final EnhancedCommandController driver = new EnhancedCommandController(0);

  public RobotContainer() {
    kicker = new Feeder();

    configureBindings();
    configureDefaultCommands();
      
    SmartDashboard.putData(autoChooser);

    DataLogManager.start();
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
 
    driver.b().onTrue(kicker.stop()); 
    driver.y().onTrue(kicker.setVoltage(() -> 10));

  }

  public void configureDefaultCommands(){
    //kicker.setDefaultCommand(kicker.joystickBangBangCommand(() -> driver.getRightX()));
    kicker.setDefaultCommand(kicker.setVelocityMPS(() -> driver.getRightY() * kicker.maxSpeedRPS * kicker.surfaceMetersPerMotorRotation));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}