package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.SwerveBase;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.robot.Constants;

public class Swerve extends SwerveBase{
    public Swerve(String[] camNames, TalonFXModule[] modules, int[] reefTags) {
        super(camNames, modules, reefTags);
    }

    public boolean isOnBump() {
        double pitch = Math.abs(pigeon.getPitch().getValueAsDouble());
        double roll = Math.abs(pigeon.getRoll().getValueAsDouble());

        System.out.println("pitch = " + pitch);
        System.out.println("roll = " + roll);

        return pitch > Constants.BUMP_THRESHOLD || roll > Constants.BUMP_THRESHOLD;
    }

    public Command bumpTest() {
        return runOnce(() -> {
            if (isOnBump()) {
                System.out.println("ON BUMP");
            }
        });
    }
}
