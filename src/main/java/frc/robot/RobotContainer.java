// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import frc.BisonLib.BaseProject.Util.ShooterInterpolationMap;
import frc.BisonLib.BaseProject.Util.ShooterInterpolationMap.ShooterSetpoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.wpilibj2.command.Commands.*;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  ShooterInterpolationMap shooterInterpolationMap;





  private static final EnhancedCommandController driver =
      new EnhancedCommandController(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    shooterInterpolationMap = new ShooterInterpolationMap("simulated_optimal_trajectories.csv");

    // Configure the trigger bindings
    configureBindings();

      
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
    driver.a().onTrue(putInterpolatedShooterSetpointsToNetworktables(()-> 1.5, ()-> 0.0918)); // 72.1278,2296.06
    driver.b().onTrue(putInterpolatedShooterSetpointsToNetworktables(()-> 2.2627, ()-> 1.7449)); // 81.6115,2326.14
    driver.x().onTrue(putInterpolatedShooterSetpointsToNetworktables(()-> 3.1102, ()-> 3.0306)); // 86.2431, 2362.77
    driver.x().onTrue(putInterpolatedShooterSetpointsToNetworktables(()-> 5.1441, ()-> -0.0918)); // 53.9323,3200.07
  }

  private Command putInterpolatedShooterSetpointsToNetworktables(DoubleSupplier d, DoubleSupplier v){
    return runOnce(()-> {
      ShooterSetpoint s = shooterInterpolationMap.getSetpoint(d.getAsDouble(), v.getAsDouble());
      SmartDashboard.putNumber("Setpoint RPM", s.rpm());
      SmartDashboard.putNumber("Setpoint Angle", s.angle());
    });
  }

  // The command specified in here is run in autonomous
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}