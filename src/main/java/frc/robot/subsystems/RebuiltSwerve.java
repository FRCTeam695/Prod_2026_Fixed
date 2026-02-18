package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;

import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;


public class RebuiltSwerve extends SwerveBase{
    
    private final ProfiledPIDController turnController = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(700, 1000));

    public RebuiltSwerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);
        turnController.enableContinuousInput(-180, 180);
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

}