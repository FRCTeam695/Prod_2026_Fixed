package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.intakeConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class Intake extends SubsystemBase {
    private TalonFX pivot;
    private TalonFX roller;

    private MotionMagicVoltage m_pivot; //maybe make better name?

    public Intake() {
        pivot = new TalonFX(0); //not correct id
        roller = new TalonFX(1); //not correct id

        m_pivot = new MotionMagicVoltage(0);
       
        TalonFXConfiguration config1 = new TalonFXConfiguration();
            config1.CurrentLimits.SupplyCurrentLimit = 20.0;
        TalonFXConfiguration config2 = new TalonFXConfiguration();
            config2.CurrentLimits.SupplyCurrentLimit = 20.0;

    }

    public Command angle(DoubleSupplier value) {
        return runOnce(
            () -> {
                pivot.set(intakeConstants.SET_POSITION);
            }
        );
    }

    public Command intake() {
        return runOnce(
            () -> {
                roller.set(intakeConstants.SET_SPEED);
            }
        );
    }

}
