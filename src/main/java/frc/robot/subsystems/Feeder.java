package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

import edu.wpi.first.networktables.DoublePublisher;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.RPM;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleSupplier;

public class Feeder extends SubsystemBase {
    
    public TalonFX floorFeederMotor;
    
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VelocityVoltage velocitySetter = new VelocityVoltage(0);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable feederTable = inst.getTable("Feeder");

    private final DoublePublisher mpsPub = feederTable.getDoubleTopic("Velocity Meters Per Second").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher dutyCycleOutPub = feederTable.getDoubleTopic("DutyCycleOut").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher voltagePub = feederTable.getDoubleTopic("Voltage").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setPointPub = feederTable.getDoubleTopic("Set Point").publish(PubSubOption.periodic(0.02));

    public static final double metersPerRotationOfMotor = ((12/36.)*30*5)/1000.;
    //1st pulley on motor = 12 teeth
    //2nd pulley on shaft = 36 teeth
    //3rd pulley on same shaft = 30 teeth
    //5mm belt pitch
    //mm to meters  = 25.4/1

    public Feeder(){
        
        floorFeederMotor = new TalonFX(56);
        TalonFXConfigurator configurator = floorFeederMotor.getConfigurator();
    
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 0;
        slot0.kS = 0.23;
        slot0.kV = 0.12;

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimit = 120;
        currentLimitsConfigs.SupplyCurrentLimit = 40;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        configurator.apply(currentLimitsConfigs);
        
        configurator.apply(slot0);
        
    }

    public Command setVelocityMPS(DoubleSupplier MPS){
        return setVelocityRPS(
            ()-> (1/metersPerRotationOfMotor*MPS.getAsDouble())
        );
    }
    public Command setVelocityRPS(DoubleSupplier rps){
        return run(()->{
        SmartDashboard.putNumber("Setpoint Vel", rps.getAsDouble() * metersPerRotationOfMotor);
        floorFeederMotor.setControl(velocitySetter.withVelocity(rps.getAsDouble()));
        });
       
    }

    public Command openLoopSet(DoubleSupplier percentVbus){
        return run(()->{
            floorFeederMotor.setControl(dutyCycleOut.withOutput(percentVbus.getAsDouble()));
        });
    }

    

    public void periodic(){

        mpsPub.set(floorFeederMotor.getVelocity(true).getValue().in(RPM)*metersPerRotationOfMotor/60);
        dutyCycleOutPub.set(floorFeederMotor.getDutyCycle().getValueAsDouble());
        voltagePub.set(floorFeederMotor.getMotorVoltage().getValueAsDouble());  
        setPointPub.set(floorFeederMotor.getClosedLoopReference(true).getValueAsDouble()*metersPerRotationOfMotor);

        SmartDashboard.putNumber("vel", floorFeederMotor.getVelocity(true).getValue().in(RPM)*metersPerRotationOfMotor/60);
    }
}