package frc.BisonLib.BaseProject.Controller;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;


public class EnhancedCommandController extends CommandXboxController{

    
    public Trigger flick;
    boolean isFlicked = false;
    double oldTime = 0;
    double oldLeftX = 0;

    boolean upJump = false;
    boolean downJump = false;
    double upJumpTime = 1;
    double downJumpTime = 1000;
    double timeSinceUp = 0;
    double timeSinceDown = 0;
    boolean jumpTimeSpanValid = false;
    double oldJumpTimeSpan = 0;


    boolean isJoystickStill = true;
    boolean wasCenteredBeforeMove = false;
    double lastTimeCentered = 0;
    double timeSinceFlick = 0;
    boolean flickCooldownDone = false;
    

    public EnhancedCommandController(int port){
        super(port);

        flick = new Trigger(()->isFlicked);
        
        
    }


    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                SmartDashboard.putBoolean("Alliance", temp);
                return temp;
              }
        SmartDashboard.putBoolean("Alliance", false);
        return false;
    }

    public boolean getFlick(){
        return isFlicked;
    }

    public ChassisSpeeds getRequestedChassisSpeeds(){
        
        // +X is forward and +Y is left in wpilib coordinates
        double Xj = getLeftY();
        double Yj = getLeftX();

        double newTime = Timer.getFPGATimestamp();
        

            
            double elapsedTime = newTime - oldTime;
            double newLeftX = getLeftX();
            double derivativeAbs = Math.abs((newLeftX-oldLeftX)/elapsedTime);
            double derivative = (newLeftX-oldLeftX)/elapsedTime;

            timeSinceUp = newTime - upJumpTime;
            timeSinceDown = newTime - downJumpTime;

            if(Math.abs(newLeftX) < 0.2 && derivativeAbs < 0.05){
                lastTimeCentered = newTime;
            }

            wasCenteredBeforeMove = (newTime - lastTimeCentered) > 0.05;

            if(derivative > 0 && derivativeAbs > 0.01){
                upJump = true;
                
                upJumpTime = newTime;
            }
            
            if(derivative < 0 && derivativeAbs > 0.01){
                downJump = true;
                
                downJumpTime = newTime;
            }

            // SmartDashboard.putBoolean("upJump", upJump);
            // SmartDashboard.putBoolean("downJump", downJump);
            // SmartDashboard.putNumber("upJumpTime", upJumpTime);
            // SmartDashboard.putNumber("downJumpTime", downJumpTime);

            double jumpTimeSpanAbs = Math.abs(upJumpTime - downJumpTime);
            double newjumpTimeSpan = upJumpTime - downJumpTime;
                //if negative, up happened first. If positive, down happened first
            double timeSinceLastJump = 0;
            if(newjumpTimeSpan < 0){
                timeSinceLastJump = timeSinceDown;
                //down is currently happening

            }
            if(newjumpTimeSpan > 0){
                timeSinceLastJump = timeSinceUp;
            }
            oldJumpTimeSpan = newjumpTimeSpan;

            if(derivativeAbs < 0.01){
                isJoystickStill = true;
            }else{
                isJoystickStill = false;
            }
            
            if((newTime - timeSinceFlick) < 0.04){
                flickCooldownDone = false;
            }else{
                flickCooldownDone = true;
            }

            // SmartDashboard.putNumber("newjumpTimeSpan", newjumpTimeSpan);
            // SmartDashboard.putNumber("jumpTimeSpanAbs", jumpTimeSpanAbs);
            jumpTimeSpanValid = false;

            if(jumpTimeSpanAbs <= 0.1 && !isFlicked && timeSinceLastJump < 0.04 && wasCenteredBeforeMove && flickCooldownDone){
                upJump = false;
                downJump = false;
                jumpTimeSpanValid = true;
                
                isFlicked = true;
                timeSinceFlick = newTime;

                
            }else{
                if(isFlicked && isJoystickStill)
                 isFlicked = false;
                
                
            }
            oldTime = newTime;
            oldLeftX = newLeftX;

            // SmartDashboard.putBoolean("jumpTimeSpanValid", jumpTimeSpanValid);
            // SmartDashboard.putNumber("Derivative of Flick", derivative);
            // SmartDashboard.putNumber("oldLeftX", oldLeftX);
            // SmartDashboard.putNumber("newLeftX", newLeftX);
            // SmartDashboard.putBoolean("isFlicked", isFlicked);

        // +Z is ccw
        double Zj = -getControllableRightStick();


        if(!isRedAlliance()){
            Xj *= -1;
            Yj *= -1;
        }
        double db = 0.05;

        Xj = MathUtil.applyDeadband(Xj, db);
        Yj = MathUtil.applyDeadband(Yj, db);
        Zj = MathUtil.applyDeadband(Zj, db);

        //WANTED FIELD RELATIVE VELOCITIES
        Xj *= Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP;
        Yj *= Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP;
        Zj *= Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND;

        return new ChassisSpeeds(Xj, Yj, Zj);
    }


    /**
     * Squares the rightX stick values, makes the robot accel. less for smaller joystick inputs
     * 
     * @return the squared right stick values
     */
    public double getControllableRightStick(){
        double original = super.getRightX() * 0.75;
        return original;
    }

    /**
     * Rumbles the controller to a given strength
     * 
     * @param strength A Double Supplier that defines the rumbles intensity
     * @return A Command that rumbles to a strength and turns off the rumble when finished
     */
    public Command rumble(DoubleSupplier strength){
        return runEnd
                  (
                    ()-> super.getHID().setRumble(RumbleType.kBothRumble, strength.getAsDouble()),
                    ()-> super.getHID().setRumble(RumbleType.kBothRumble, 0)
                  );
    }
}