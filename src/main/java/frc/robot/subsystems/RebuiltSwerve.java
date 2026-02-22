package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;


public class RebuiltSwerve extends SwerveBase{

    public final double BUMP_THRESHOLD = 1.0;
    
    private final ProfiledPIDController turnController = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(700, 1000));

    public final Trigger onBump;

    
    // bump constants
    private final double blueLowerX = 0;
    private final double blueUpperX = 0;
    private final double blueLowerY = 0;
    private final double blueUpperY = 0;

    private final double redLowerX = 0;
    private final double redUpperX = 0;
    private final double redLowerY = 0;
    private final double redUpperY = 0;

    public RebuiltSwerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);
        turnController.enableContinuousInput(-180, 180);

        onBump = new Trigger(
            () -> (isOnBumpWithAngle() && isOnBumpWithLocation())
        );
    }


    public Command rotateTowardsVirtualHub(Supplier<ChassisSpeeds> wantedSpeeds, Supplier<Pose2d> hubPose){
        return 
        runOnce(
            ()-> {
                Pose2d robotPose = getSavedPose();
                ChassisSpeeds latestSpeeds = getLatestChassisSpeed();
                turnController.reset(robotPose.getRotation().getDegrees(), Math.toDegrees(latestSpeeds.omegaRadiansPerSecond));
            }
        ).andThen(
        run(()->{
            Pose2d virtualPose = hubPose.get();
            Pose2d robotPose = getSavedPose();
            ChassisSpeeds currentSpeeds = getLatestChassisSpeed();

            double dx = virtualPose.getX() - robotPose.getX();
            double dy = virtualPose.getY() - robotPose.getY();
            double norm = Math.hypot(dx, dy);
            double theta = Math.toDegrees(Math.atan2(dy, dx));

            double feedForward = (currentSpeeds.vxMetersPerSecond * dy - currentSpeeds.vyMetersPerSecond * dx)/ (norm * norm);

            ChassisSpeeds speeds = wantedSpeeds.get();
            double pidOutput = turnController.calculate(robotPose.getRotation().getDegrees(), new State(
                theta, currentSpeeds.omegaRadiansPerSecond
            ));

            getAngularComponentFromRotationOverride(theta); // this line is currently just for logging purposes

            speeds.omegaRadiansPerSecond = feedForward + Math.toRadians(turnController.getSetpoint().velocity) + pidOutput;
            drive(speeds, true, true);
        }));
    }

     public boolean isOnBumpWithAngle() {
        double pitch = Math.abs(pigeon.getPitch().getValueAsDouble());
        double roll = Math.abs(pigeon.getRoll().getValueAsDouble());

        //System.out.println("pitch = " + pitch);
        //System.out.println("roll = " + roll);

        return pitch > BUMP_THRESHOLD || roll > BUMP_THRESHOLD;
    }

    public boolean isOnBumpWithLocation() {
       
        Pose2d robotPose = getSavedPose();
        double x = robotPose.getX();
        double y = robotPose.getY();
        
        if (( x > blueLowerX && x < blueUpperX && y > blueLowerY && y < blueUpperY ) 
        || (x > redLowerX && x < redUpperX && y > redLowerY && y < redUpperY)) {
            return true;
        } else {
            return false;
        }

    }

    public double calculateBumpRotationSetpointDegrees() {
        
        double r = getSavedPose().getRotation().getDegrees() + 180;

        double bestAngle = 0;

        // if between 0 and 90, turnm to 45
        // if between 91 and 180, turn to 135
        // if between 181 and 270, turn to 225
        // if betwenen 271 and 360, turn to 315

        if (r > 0 && r < 90) {
            bestAngle = 45;
        } else if (r > 91 && r < 180) {
            bestAngle = 135;
        } else if (r > 181 && r < 270) {
            bestAngle = 225;
        } else if (r > 271 && r < 360) {
            bestAngle = 315;
        }

        return bestAngle;
    }

    public Command turnOnBump(){
        return new ConditionalCommand(
            rotateToAngle(() -> calculateBumpRotationSetpointDegrees(), null), 
            null, 
            onBump
        );
    }

}