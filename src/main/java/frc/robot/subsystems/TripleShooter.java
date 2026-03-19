package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IndividualShooter.ShooterMiniConfig;

public class TripleShooter extends SubsystemBase{

    public final IndividualShooter shooterLeft;
    public final IndividualShooter shooterMiddle;
    public final IndividualShooter shooterRight;

    private ShuffleboardTab shootTab = Shuffleboard.getTab("Shooter");
    private GenericEntry rpmPercent = shootTab.add("RPM Percent Adjustment", 1.0).getEntry();
    private double prevRPMEntry = 1.0;

    public final double kShooterRotationsToMeters = 2 * Math.PI * Units.inchesToMeters(2);
    public final double kMaxSpeedMPS = kShooterRotationsToMeters * 100;
    public final Trigger allShootersWithinTolerance;
    public TripleShooter(ShooterMiniConfig configLeft, ShooterMiniConfig configMiddle, ShooterMiniConfig configRight){
        shooterLeft = new IndividualShooter(configLeft, kShooterRotationsToMeters);
        shooterMiddle = new IndividualShooter(configMiddle, kShooterRotationsToMeters);
        shooterRight = new IndividualShooter(configRight, kShooterRotationsToMeters);
        allShootersWithinTolerance = shooterLeft.withinTolerance.and(shooterMiddle.withinTolerance).and(shooterRight.withinTolerance);


    }


    public Command setVelocityMPS(DoubleSupplier velocityMPS){
        return run(
            ()->{
                shooterLeft.setVelocityRPS(velocityMPS.getAsDouble() / kShooterRotationsToMeters);
                shooterMiddle.setVelocityRPS(velocityMPS.getAsDouble() / kShooterRotationsToMeters);
                shooterRight.setVelocityRPS(velocityMPS.getAsDouble() / kShooterRotationsToMeters);
            }
        );
    }

    public Command setVelocityTorqueCurrentMPS(DoubleSupplier velocityMPS){
        return 
            run(()-> {

                double velocity = velocityMPS.getAsDouble() * rpmPercent.getDouble(1.0);

                SmartDashboard.putNumber("Commanded Vel MPS shooter", velocity);
                shooterLeft.setTorqueCurrent(velocityMPS.getAsDouble()  / velocity);
                shooterMiddle.setTorqueCurrent(velocityMPS.getAsDouble() / velocity);
                shooterRight.setTorqueCurrent(velocityMPS.getAsDouble() / velocity);
            }
        );
    }

    public Command setVelocityMPSWithCondition(DoubleSupplier velocityMPS, Trigger condition){
        return 
        runOnce(
            ()-> {

                double velocity = velocityMPS.getAsDouble() * rpmPercent.getDouble(1.0);

                shooterLeft.configForVelocityControl();
                shooterMiddle.configForVelocityControl();
                shooterRight.configForVelocityControl();

                shooterLeft.setVelocityRPS(velocity / kShooterRotationsToMeters);
                shooterMiddle.setVelocityRPS(velocity / kShooterRotationsToMeters);
                shooterRight.setVelocityRPS(velocity / kShooterRotationsToMeters);
            }
        ).andThen(
            run(()-> {

                double velocity = velocityMPS.getAsDouble() * rpmPercent.getDouble(1.0);

                shooterLeft.setVelocityRPS(velocity / kShooterRotationsToMeters);
                shooterMiddle.setVelocityRPS(velocity / kShooterRotationsToMeters);
                shooterRight.setVelocityRPS(velocity / kShooterRotationsToMeters);
            }).until(condition)
        );
    }

    public Command setDutyCycle(DoubleSupplier percentVbus){
        return run(
            ()-> {
                shooterLeft.setDutyCycle(percentVbus.getAsDouble());
                shooterMiddle.setDutyCycle(percentVbus.getAsDouble());
                shooterRight.setDutyCycle(percentVbus.getAsDouble());
            }
        );
    }

    @Override
    public void periodic(){
        shooterLeft.sendSendables();
        shooterMiddle.sendSendables();
        shooterRight.sendSendables();
    }


}