package frc.robot;

    //necessity
    import edu.wpi.first.wpilibj2.command.SubsystemBase;

    //command
    import edu.wpi.first.wpilibj2.command.Command;

    //motor
    import com.ctre.phoenix6.hardware.TalonFX;
    
    import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
//PID
    import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.networktables.DoublePublisher;

//Network Tables
    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Units
    import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;




public class Indexer extends SubsystemBase {
    
    public TalonFX floorIndexerMotor;
    
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VelocityVoltage velocitySetter = new VelocityVoltage(0);

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable indexerTable = inst.getTable("Indexer");

    private final DoublePublisher mpsPub = indexerTable.getDoubleTopic("Velocity Meters Per Second").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher dutyCycleOutPub = indexerTable.getDoubleTopic("DutyCycleOut").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher voltagePub = indexerTable.getDoubleTopic("Voltage").publish(PubSubOption.periodic(0.02));
    private final DoublePublisher setPointPub = indexerTable.getDoubleTopic("Set Point").publish(PubSubOption.periodic(0.02));

    public static final double metersPerRotationOfMotor = ((12/65.)*30*5)/1000.;
    //1st pulley on motor = 12 teeth
    //2nd pulley on shaft = 65 teeth
    //3rd pulley on same shaft = 30 teeth
    //5mm belt pitch
    //mm to meters  = 25.4/1

    public Indexer(){
        
        floorIndexerMotor = new TalonFX(56);
        TalonFXConfigurator configurator = floorIndexerMotor.getConfigurator();
    
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
        return setVelocity(
            ()-> (1/metersPerRotationOfMotor*MPS.getAsDouble())
        );
    }
    public Command setVelocity(DoubleSupplier rps){
        return run(()->{
        SmartDashboard.putNumber("Setpoint Vel", rps.getAsDouble() * metersPerRotationOfMotor);
        floorIndexerMotor.setControl(velocitySetter.withVelocity(rps.getAsDouble()));
        });
       
    }

    public Command openLoopSet(DoubleSupplier percentVbus){
        return run(()->{
            floorIndexerMotor.setControl(dutyCycleOut.withOutput(percentVbus.getAsDouble()));
        });
    }

    

    public void periodic(){

        mpsPub.set(floorIndexerMotor.getVelocity(true).getValue().in(RPM)*metersPerRotationOfMotor/60);
        dutyCycleOutPub.set(floorIndexerMotor.getDutyCycle().getValueAsDouble());
        voltagePub.set(floorIndexerMotor.getMotorVoltage().getValueAsDouble());  
        setPointPub.set(floorIndexerMotor.getClosedLoopReference(true).getValueAsDouble()*metersPerRotationOfMotor);

        SmartDashboard.putNumber("vel", floorIndexerMotor.getVelocity(true).getValue().in(RPM)*metersPerRotationOfMotor/60);
    }
}
