package frc.robot;

    //necessity
    import edu.wpi.first.wpilibj2.command.SubsystemBase;

    //command
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.FunctionalCommand;

    //motor
    import com.ctre.phoenix6.hardware.TalonFXS;



    import com.ctre.phoenix6.configs.TalonFXSConfiguration;

    //Network Tables
    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

    //Units
    import static edu.wpi.first.units.Units.RPM;




public class Indexer extends SubsystemBase {
    
    public TalonFXS floorIndexerMotor;


    //booleans
    public boolean isIndexing;

    public Indexer(){

        floorIndexerMotor = new TalonFXS(1); //insert deviceID later, this deviceID is wrong
        
        isIndexing = false;
    }


    public void set(double speed){
        if(speed == 0){
            isIndexing = false;
        }else{
            isIndexing = true;
        }
        floorIndexerMotor.set(speed);
    }

    public Command feedCommand(){
        return startEnd(()->set(1),()-> set(0)); //arbitrary run speed, needs tuning
    }

    public void periodic(){
        SmartDashboard.putNumber("RPM",floorIndexerMotor.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("DutyCycleOut", floorIndexerMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Voltage", floorIndexerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("isIndexing", isIndexing);
        
    }
}
