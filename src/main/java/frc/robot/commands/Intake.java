package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase{
    private SparkFlex intakeMotor;

    public Intake() {
        intakeMotor = new SparkFlex(53, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless); //not correct id
    }

    public Command runIntake(DoubleSupplier value){
        return runOnce(
            () -> {
                intakeMotor.set(value.getAsDouble());
            }
        );
    }

}
