// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.BisonLib.BaseProject.Controller.EnhancedCommandController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import com.ctre.phoenix6.signals.InvertedValue;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final EnhancedCommandController driver = new EnhancedCommandController(0);
  private final Shooter leftShooter;
  private final Shooter middleShooter;
  private final Shooter rightShooter;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    leftShooter = new Shooter(54, InvertedValue.Clockwise_Positive, 0, 0.09, 0.15);
    middleShooter = new Shooter(55, InvertedValue.CounterClockwise_Positive, 0, 0.09, 0.15);
    rightShooter = new Shooter(52, InvertedValue.CounterClockwise_Positive, 0, 0.09, 0.15);

    configureBindings();
    configureDefaultCommands();
  }


  private void configureBindings() {
    driver.a().whileTrue(leftShooter.setVelocityMPS(()-> driver.getLeftY() * leftShooter.kMaxSpeedMPS));
    driver.b().whileTrue(middleShooter.setVelocityMPS(()-> driver.getLeftY() * middleShooter.kMaxSpeedMPS));
    driver.x().whileTrue(rightShooter.setVelocityMPS(()-> driver.getLeftY() * rightShooter.kMaxSpeedMPS));

    // driver.a().whileTrue(leftShooter.setDutyCycle(()-> driver.getLeftY()));
    // driver.b().whileTrue(middleShooter.setDutyCycle(()-> driver.getLeftY()));
    // driver.x().whileTrue(rightShooter.setDutyCycle(()-> driver.getLeftY()));

    driver.rightBumper().whileTrue(
      parallel(
        leftShooter.setVelocityMPS(()-> driver.getLeftY() * leftShooter.kMaxSpeedMPS),
        middleShooter.setVelocityMPS(()-> driver.getLeftY() * leftShooter.kMaxSpeedMPS),
        rightShooter.setVelocityMPS(()-> driver.getLeftY() * leftShooter.kMaxSpeedMPS)
      )
    );
  }

  public void configureDefaultCommands(){
    leftShooter.setDefaultCommand(leftShooter.setVelocityMPS(()-> 0));
    middleShooter.setDefaultCommand(middleShooter.setVelocityMPS(()-> 0));
    rightShooter.setDefaultCommand(rightShooter.setVelocityMPS(()-> 0));

  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

}