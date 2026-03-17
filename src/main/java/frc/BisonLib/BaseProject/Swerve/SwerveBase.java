package frc.BisonLib.BaseProject.Swerve;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.BisonLib.BaseProject.Swerve.Modules.TalonFXModule;
import frc.BisonLib.BaseProject.Util.LimelightHelpers;
import frc.robot.Constants;

public class SwerveBase extends SubsystemBase {

    private BooleanPublisher allianceShiftPulisher;

    protected TalonFXModule[] modules;

    protected final Field2d m_field = new Field2d();
    private final SwerveDrivePoseEstimator odometry;

    // NEVER DIRECTLY CALL ANY GYRO METHODS, ALWAYS USE THE SYNCHRONIZED GYRO LOCK!!
    //private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ_INTEGER);
    public final Pigeon2 pigeon;
    private final BaseStatusSignal yawSignal;
    private double gyroAccumYawOffset = 0;

    // private final LinearFilter xAccelFilter = LinearFilter.movingAverage(5);
    // private final LinearFilter yAccelFilter = LinearFilter.movingAverage(5);
    private final PIDController thetaController = new PIDController(Constants.Swerve.ROBOT_ROTATION_KP, 0, 0.0);
    private final BaseStatusSignal[] allOdomSignals;

    protected double max_accel = 0;
    protected double robotRotationError = 0;

    // used for wheel characterization
    protected double initialGyroAngle = 0;
    protected double[] initialPositions = new double[4];

    // odometry stuff
    protected double lastTime = Timer.getFPGATimestamp();
    private LinearFilter lowpass = LinearFilter.movingAverage(50);
    protected double currentTime = 0;
    protected double totalLoopTime = 0;
    protected double inc = 0;
    protected double avgLoopTIme = 0;
    protected double failedOdometryUpdates = 0;
    protected double successfulOdometryUpdates = 0;
    protected double limelightUpdateCounter = 0;


    public final Trigger atRotationSetpoint = new Trigger(()-> Math.abs(robotRotationError) < 1.2);
    public final Trigger almostAtRotationSetpoint = new Trigger(()-> Math.abs(robotRotationError) < 20);



    // this is a lock to make sure nobody acesses our pose while odometry is updating it
    private final ReentrantReadWriteLock odometryLock = new ReentrantReadWriteLock();

    // this is a lock to make sure nobody acesses our gyro while odometry is updating it
    private final Object gyroLock = new Object();

    private Pose2d currentRobotPose = new Pose2d();
    private SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    private SwerveModuleState[] currentModuleStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    protected String[] camNames;
 
    public SlewRateLimiter omegaFilter = new SlewRateLimiter(Math.toRadians(1074.5588535));
    public SlewRateLimiter xFilter = new SlewRateLimiter(7);
    public SlewRateLimiter yFilter = new SlewRateLimiter(7);
    //private Pigeon2 pigeon = new Pigeon2(8);

    //6-11
    // 17 -22
    public int[] validTagIDs;

    public double prevAccelX = 0;
    public double prevAccelY = 0; 

    String gameData = DriverStation.getGameSpecificMessage();


    
    Map<String, int[]> tagDictionary;

    public void addTagToDictionary(String tagSetName, int[] tagSet){
        tagDictionary.put(tagSetName, tagSet);
    }

    /**
     * Does all the constructing
     * 
     * @param cameras An array of cameras used for pose estinmation
     * @param moduleTypes The type of swerve module on the swerve drive
     * @param validTagIDs April Tag IDs which are safe to use for pose estimation (stable tags that don't move around too much)
     */
    public SwerveBase(String[] camNames, TalonFXModule[] modules, int[] validTagIDs) {
        allianceShiftPulisher = NetworkTableInstance.getDefault().getBooleanTopic("AllianceShift").publish();
        //vision Tag definitions
        this.validTagIDs = validTagIDs;
        tagDictionary = new HashMap<String, int[]>();
        addTagToDictionary("allTags", new int[] {
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17 ,18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32}
        );
        addTagToDictionary("hubTags", new int[] {
            // blue tags
            19, 20, 21, 24, 25, 26, 18, 27, 
            // red tags
            3, 4, 5, 8, 9, 10, 11, 2
        });

        addTagToDictionary("redHubTags", new int[]{
            3, 4, 5, 8, 9, 10, 11, 2
        });

        addTagToDictionary("blueHubTags", new int[]{
            19, 20, 21, 24, 25, 26, 18, 27
        });


        pigeon = new Pigeon2(8, "drivetrain");
        yawSignal = pigeon.getAccumGyroZ();
        gyroAccumYawOffset = -yawSignal.getValueAsDouble();
        //pigeon.setYaw(0);
        // 4 modules * 3 signals per module + 1 for pigeon
        allOdomSignals = new BaseStatusSignal[(4 * 3) + 1];
        for(int i = 0; i < modules.length; ++i){
            var signals = modules[i].getOdometrySignals();
            allOdomSignals[i*3 + 0] = signals[0]; // drive position
            allOdomSignals[i*3 + 1] = signals[1]; // drive velocity
            allOdomSignals[i*3 + 2] = signals[2]; // module rotation (cancoder)
        }
        allOdomSignals[allOdomSignals.length-1] = yawSignal;

        this.camNames = camNames;

        // Holds all the modules
        this.modules = modules;

        /*
        * Sets the gyro at the beginning of the match and 
        * also sets each module state to present
        */

        //maybe have this keep trying to reset the gyro if it fails once
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                setGyro(0);
            } catch (Exception e) {
                setGyro(0);
            }
        }).start();


        thetaController.enableContinuousInput(-180, 180);


        odometryLock.writeLock().lock();
        Rotation2d gyroHeading = getGyroHeading();
        currentModulePositions = getModulePositions();
        try{
            odometry = new SwerveDrivePoseEstimator
            (
                Constants.Swerve.kDriveKinematics, 
                gyroHeading,
                currentModulePositions,
                new Pose2d()
            );
        }finally{
            odometryLock.writeLock().unlock();
        }


        SmartDashboard.putData("field", m_field);
        // SmartDashboard.putData("Robot angle PID controller", thetaController);
        setValidTagIDs("allTags");
    }

    public void setValidTagIDs(String tagSetName) {

        validTagIDs = tagDictionary.get(tagSetName);
        SmartDashboard.putString("Current Valid Tagset", tagSetName);
        for (String cam : camNames) {
            LimelightHelpers.SetFiducialIDFiltersOverride(cam, validTagIDs);
        }

        // SmartDashboard.putString("Valid Tag IDs", Arrays.toString(validTagIDs));
        // SmartDashboard.putString(" Valid Tag Set Name ", tagSetName);
    }

    public Command setAllTagsValid() {
        return runOnce(() -> setValidTagIDs("allTags"));
    }

    public Command setHubTagsValid() {
        return runOnce(() -> setValidTagIDs("hubTags"));
    }

    public Command setAllianceHubTagsValid(){
        return runOnce(
            ()-> {
                if(isRedAlliance()){
                    setValidTagIDs("redHubTags");
                }
                else{
                    setValidTagIDs("blueHubTags");
                }
            }
        );
    }

    public void playSong(){
        if(modules[0].getModuleType().equals("TalonFXModule")){
            new Thread(() -> {
                try {
                        Orchestra orchestra = new Orchestra();
                        for(var module : this.modules){
                            orchestra.addInstrument(module.getDriveMotor());
                            orchestra.addInstrument(module.getTurnMotor());
                        }

                        var status  = orchestra.loadMusic("EmpireStrikesBack.chrp");
                        if(status.isOK()){
                            double startTime = Timer.getFPGATimestamp();
                            orchestra.play();
                            while((Timer.getFPGATimestamp() - startTime) < 10){}
                            orchestra.stop();
                        }
                        orchestra.close();
                } catch (Exception e) {
                }
            }).start();
        }
    }

    

    public void pathplannerDriveRobotRelative(ChassisSpeeds speeds){
        driveRobotRelative(speeds);
    }


    /*
     * isRedAlliance returns true if we are red alliance and returns false if we are blue alliance
     */
    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                boolean temp = (alliance.get() == DriverStation.Alliance.Red) ? true : false;
                //SmartDasboard.putBoolean("Alliance", temp);
                return temp;
              }
        //SmartDasboard.putBoolean("Alliance", false);
        return false;
    }

    /*
     * Puts whether or not it's our alliance shift on Smart Dashboard for logging purposes
     */
    public boolean isAllianceShift(){
        boolean isShift = false;
        double matchTime = DriverStation.getMatchTime();
        if(matchTime <= 130 && matchTime > 105){ //Shift 1
            isShift = true;
        } else if(matchTime <= 105 && matchTime > 80){ //Shift 2
            isShift = false;
        } else if(matchTime <= 80 && matchTime > 55){ //Shift 3
            isShift = true;
        } else if(matchTime <= 55 && matchTime > 30){ //Shift 4
            isShift = false;
        }
    
        if(gameData.length() > 0){
            if(matchTime <=130 && matchTime >= 30){
                if(gameData.charAt(0) == 'R' && isRedAlliance()){ //if the gamespecific data returns "R", and we are the Red Alliance, we are going second.
                    isShift = !isShift;
                } else if (gameData.charAt(0) == 'B' && !isRedAlliance()){
                    isShift = !isShift;
                }
            }
        }

        allianceShiftPulisher.set(isShift);

        return isShift;
    }

    /*
     * Sets all the swerve modules to the states we want them to be in (velocity + angle)
     */
    public void setModules(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP);

        for(var module : modules){
            //SmartDashboard.putString("Swerve/Module State " + module.index, desiredStates[module.index].toString());
            module.setDesiredState(desiredStates[module.index]);
        }
        
    }


    /**
     * setGyro sets the gyro to a given angle,
     * @param degrees the degree value that the gyro should be set to
     */
    public void setGyro(double degrees){
        //pigeon.setYaw(0);
        synchronized(gyroLock){
            // gyro.reset();
            // gyro.setAngleAdjustment(-degrees);
            gyroAccumYawOffset = -allOdomSignals[allOdomSignals.length-1].getValueAsDouble()/Constants.Swerve.GYRO_DRIFT_COMPENSATION + degrees;
        }
    }


    /**
     * Finds a new angular speed based on rotation override
     * 
     * @param originalSpeeds The original chassis speeds of the robot as inputted by the driver
     * 
     * @return The new rotation component as calculated by the rotation override
     */
    public double getAngularComponentFromRotationOverride(double wantedAngle){
        double currentRotation = getSavedPose().getRotation().getDegrees();
        double pidOutput = thetaController.calculate(currentRotation, wantedAngle);

        SmartDashboard.putNumber("Wanted Robot Angle", thetaController.getSetpoint());
        robotRotationError = thetaController.getError();
        return MathUtil.clamp(
                    pidOutput / Math.toDegrees(Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND), -1, 1
                ) * Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECOND;
    }


    /**
     * This method should not be touched by anything except odometry, 
     * all angles should be pulled from odometry and not directly from the gyro.
     * This is done to ensure that there is no thread contention for this method.
     * 
     * @returns the angle the gyro is facing expressed as a Rotation2d
     */
    private Rotation2d getGyroHeading() {
        //0.99622314806
        synchronized (gyroLock){
            double unmoddedGyoHeading = gyroAccumYawOffset + (allOdomSignals[allOdomSignals.length-1].getValueAsDouble()/Constants.Swerve.GYRO_DRIFT_COMPENSATION);
            // SmartDashboard.putNumber("Unmodded Gyro Heading", unmoddedGyoHeading);
            return Rotation2d.fromDegrees(Math.IEEEremainder(unmoddedGyoHeading, 360));
            //return new Rotation2d(-Math.toRadians(Math.IEEEremainder(gyro.getAngle()/Constants.Swerve.GYRO_DRIFT_COMPENSATION, 360)));
        }
    }


    /**
     * calculates the distance from the current robot pose to the supplied translation,
     * can be potentially used for figuring out if the robot is in some specific zone
     * 
     * @param other The translation to find the distance to
     * @return The distance from the robot pose to the other supplied translation
     */
    public double getDistanceToTranslation(Translation2d other){
        return getSavedPose().getTranslation().getDistance(other);
    }


    /*
     * Returns the current chassis speeds of the robot, 
     * used with pathplanner
     */
    public ChassisSpeeds getLatestChassisSpeed(){
        ChassisSpeeds speeds;
        odometryLock.readLock().lock();
        try{
            speeds = Constants.Swerve.kDriveKinematics.toChassisSpeeds(currentModuleStates);
        }finally{
            odometryLock.readLock().unlock();
        }
        return speeds;
    }


    public SwerveModuleState[] getLatestModuleStates(){
        SwerveModuleState[] states;
        odometryLock.readLock().lock();
        try{
            states = currentModuleStates;
        }finally{
            odometryLock.readLock().unlock();
        }
        return states;
    }


    /**
     * @returns an array containing the position of each swerve module (check SwerveModule.java for further details)
     */
    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(var module : modules){
            modulePositions[module.index] = module.getPosition();
        }

        return modulePositions;
    }


    /**
     * @return an array containing the state (velocity + rotation) of each swerve module
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];

        for(var module : modules){
            states[module.index] = module.getState();
        }

        return states;
    }


    /**
     * Gets the acceleration of each swerve module, 
     * keep in mind these can be slightly inaccurate because of wheel slippage
     * 
     * @return a double array that contains the acceleration of each swerve module
     */
    public double[] getModuleAccelerations(){
        double[] accelerations = new double[4];

        for(var module : modules){
            accelerations[module.index] = module.getDriveAcceleration();
        }

        return accelerations;
    }


    public double[] getRawDrivePositions(){
        double[] positions = new double[4];

        for(var module : modules) {
            positions[module.index] = module.getUncachedDrivePosition();
        }

        return positions;
    }


    /**
     * Manually resets the odometry to a given pose
     * Also resets the gyro
     * Pretty much only used at the start of auton
     * 
     * @param pose The pose to set the robot pose to
     */
    public void resetOdometry(Pose2d pose) {
        setGyro(pose.getRotation().getDegrees());
        Rotation2d gyroHeading = getGyroHeading();
        odometryLock.writeLock().lock();
        try{
            odometry.resetPosition(
                gyroHeading,
                currentModulePositions,
                pose);
        }finally{
            odometryLock.writeLock().unlock();
        }
    }


    /**
     * resetGyro sets the gyro to "facing away from the driver station"
     * 
     * @return A command that "zeroes" our gyro
     */
    public Command resetGyro() {
        return resetGyroWithAllianceFlip(180);
    }

    
    public Command backwardsResetGyro(){
        return resetGyroWithAllianceFlip(0);
    }


    public Command driveAtSpeed (double speed) {
        return run (() -> {

            ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        MathUtil.clamp(speed, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                        MathUtil.clamp(speed, -Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND, Constants.Swerve.MAX_TRACKABLE_SPEED_METERS_PER_SECOND),
                    getAngularComponentFromRotationOverride(0)
                );
                // SmartDashboard.putString("align speeds", speeds.toString());

                drive(speeds);
        });
    }

    public Command resetGyroWithAllianceFlip(double angle){
        return runOnce(
            ()-> 
                {
                    if(isRedAlliance()){
                        odometryLock.writeLock().lock();
                        try{
                            resetOdometry(new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), Rotation2d.fromDegrees(angle)));
                        }finally{
                            odometryLock.writeLock().unlock();
                        }
                    }
                    else {
                        odometryLock.writeLock().lock();
                        try{
                            resetOdometry(new Pose2d(currentRobotPose.getX(), currentRobotPose.getY(), Rotation2d.fromDegrees(angle+180)));
                        }finally{
                            odometryLock.writeLock().unlock();
                        }
                    }
                }
        ).ignoringDisable(true).andThen(cameraSeedCommand());
    }

    public Command cameraSeedCommand(){
        return runOnce(()-> {
            seedCameraHeading();
        }).ignoringDisable(true);
    }

    public void seedCameraHeading(){
        //LL RESET
        for(String cam: camNames){
            LimelightHelpers.SetIMUMode(cam, 1);
            LimelightHelpers.SetRobotOrientation(cam, getSavedPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.SetIMUMode(cam, 1);
        }
    }


    public Command requireSubsystem(){
        return new WaitCommand(0);
    }
    

    /*
     * Stops all of the swerve modules
     */
    public void stopModules() {
        for(var module : modules){
            module.stop();
        }
    }


    /**
     * Continuously rotates robot to the specified angle while maintaining normal driver control of the robot
     * 
     * @param angleDegrees The angle in degrees that the robot should turn to, 
     *                     this is a double supplier so you can continously pass different values
     * 
     * @return Returns a functional command that will rotate the robot to a specified angle, 
     *         when interrupted, will return driver control to robot rotation
     */
    public Command rotateToAngle(DoubleSupplier angleDegrees, Supplier<ChassisSpeeds> speedSupplier){
        return run
        (
            /* EXCECUTE */
            ()-> {
                    ChassisSpeeds speeds = speedSupplier.get();
                    speeds.omegaRadiansPerSecond = getAngularComponentFromRotationOverride(angleDegrees.getAsDouble());
                    drive(speeds);
                 }
        );
    }

    //Jarvis, activate swerve wheels in an x-position for max defense against bumping while shooting in place.
    public Command xLockWheels(){
        return run(  
            ()->{
                modules[0].setDesiredState( 
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45))
                );
                modules[1].setDesiredState(
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
                );
                modules[2].setDesiredState(
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45))
                );
                modules[3].setDesiredState(
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
                );
            }

        );
    }

    /**
     * Drives swerve given chassis speeds robot relative
     * DOESN'T APPLY ACCELERATION LIMITS
     * 
     * @param chassisSpeeds The chassis speeds the robot should travel at
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        var tmpStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(tmpStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP);
        var speeds = Constants.Swerve.kDriveKinematics.toChassisSpeeds(tmpStates);

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * Constants.Swerve.SKEW_COMPENSATION_RATE);

        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getSavedPose().getRotation()),
            getSavedPose().getRotation().plus(skewCompensationFactor));

        //SmartDashboard.putString("Swerve/Commanded Chassis Speeds", chassisSpeeds.toString());
        // convert chassis speeds to module states
        SwerveModuleState[] moduleStates = Constants.Swerve.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // set the modules to their desired speeds
        setModules(moduleStates);
        
    }

    /*
     * Drives the robot in teleop, we don't want it fighting the auton swerve commands
     */
    public void teleopDefaultCommand(Supplier<ChassisSpeeds> speedsSupplier, boolean fieldOriented){
        drive(speedsSupplier.get());
    } //590, 736
    
    /**
     * Drives swerve given chassis speeds
     * Should be called every loop
     * 
     * @param commandedSpeeds the commanded chassis speeds from the joysticks
     */
    public void drive(ChassisSpeeds commandedSpeeds){

        ChassisSpeeds currentFieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getLatestChassisSpeed(), getSavedPose().getRotation());

        // I made the convention "v" means robots actual vel and "w" is robots wanted vel
        double v_x = currentFieldRelSpeeds.vxMetersPerSecond;
        double v_y = currentFieldRelSpeeds.vyMetersPerSecond;
        double w_x = commandedSpeeds.vxMetersPerSecond;
        double w_y = commandedSpeeds.vyMetersPerSecond;
        double dt = 0.02;
        
        double v_mag = Math.hypot(v_x, v_y);
        double w_along_v = 0.0;
        
        if (v_mag > 1e-6) {
            // project w onto v (commanded velocity component along current velocity direction)
            double dot = v_x * w_x + v_y * w_y; // w · v
            w_along_v = dot / v_mag;            // signed scalar projection
        } else {
            // if robot is nearly stopped, just use commanded speed magnitude (all commanded vel is speeding us up)
            w_along_v = Math.hypot(w_x, w_y);
        }

        // SmartDashboard.putNumber("w along v", w_along_v);
        // SmartDashboard.putNumber("v mag", v_mag);
        
        // the signum signifies if we are requesting speed up/braking
        double max_fwd_accel = Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ *
                               (1 - Math.signum(w_along_v - v_mag) * v_mag / Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP);

        if(w_along_v - v_mag > 0){
            max_fwd_accel = Constants.Swerve.MAX_ACCELERATION_METERS_PER_SECOND_SQ *
                               (1 -  (v_mag / Constants.Swerve.MAX_SPEED_METERS_PER_SECONDS_TELEOP));
        }
        else{
            max_fwd_accel = Constants.Swerve.MAX_WHEEL_TRACTION_METERS_PER_SECOND_SQ;
        }

        // SmartDashboard.putNumber("max fwd accel", max_fwd_accel);
        
        double desiredForwardAccel = (w_along_v - v_mag)/dt;

        // project commanded vel onto forward axis, if we aren't moving rn then all wanted vel is parallel
        double w_parallel_x = (v_mag > 1e-6) ? (v_x / v_mag) * w_along_v : w_x;
        double w_parallel_y = (v_mag > 1e-6) ? (v_y / v_mag) * w_along_v : w_y;
        // SmartDashboard.putNumber("parallel cmd vel", Math.hypot(w_parallel_x, w_parallel_y));

        // perpendicular component = commanded - parallel
        double w_perp_x = w_x - w_parallel_x;
        double w_perp_y = w_y - w_parallel_y;
        double w_perp_mag = Math.hypot(w_perp_x, w_perp_y);
        // SmartDashboard.putNumber("perp cmd vel", w_perp_mag);

        // current sideways vel is always 0 since no component of the current vel doesn't point in the direction of the current vel
        double desiredSkidAccel = w_perp_mag/dt;

        // make sure total accel doesnt exceed max accel
        double norm = Math.pow(desiredForwardAccel/max_fwd_accel, 2)
                    + Math.pow(desiredSkidAccel/Constants.Swerve.MAX_SKID_ACCEL, 2);
        if(norm > 1){
            // SmartDashboard.putBoolean("scaling acceleration", true);
            double scale = 1 / Math.sqrt(norm);
            desiredForwardAccel *= scale;
            desiredSkidAccel *= scale;
        }
        else{
            // SmartDashboard.putBoolean("scaling acceleration", false);
        }
        double newForwardVel = v_mag + desiredForwardAccel * dt;
        // SmartDashboard.putNumber("new forward vel", newForwardVel);

        double vx_forward;
        double vy_forward;
        if (v_mag > 1e-6) {
            vx_forward = (v_x/v_mag) * newForwardVel;
            vy_forward = (v_y/v_mag) * newForwardVel;
        }
        else{
            // if stopped, use commanded direction for initial forward push (preserves user intent)
            double w_mag = Math.hypot(w_x, w_y);
            if (w_mag > 1e-6) {
                vx_forward = (w_x / w_mag) * newForwardVel;
                vy_forward = (w_y / w_mag) * newForwardVel;
            } else {
                vx_forward = 0.0;
                vy_forward = 0.0;
            }
        }
        
        double vx_perp = 0.0;
        double vy_perp = 0.0;
        if(w_perp_mag > 1e-6) {
            vx_perp = (w_perp_x / w_perp_mag) * desiredSkidAccel * dt;
            vy_perp = (w_perp_y / w_perp_mag) * desiredSkidAccel * dt;
        }

        // vx_perp = 0;
        // vx_perp = 0;
        commandedSpeeds.vxMetersPerSecond = vx_forward + vx_perp;
        commandedSpeeds.vyMetersPerSecond = vy_forward + vy_perp;
        //commandedSpeeds.vxMetersPerSecond = xFilter.calculate(commandedSpeeds.vxMetersPerSecond);
        //commandedSpeeds.vyMetersPerSecond = yFilter.calculate(commandedSpeeds.vyMetersPerSecond);
        commandedSpeeds.omegaRadiansPerSecond = omegaFilter.calculate(commandedSpeeds.omegaRadiansPerSecond);
        //speeds = applyAccelerationLimit(speeds);

        // SmartDashboard.putNumber("Zj", commandedSpeeds.omegaRadiansPerSecond);
        // SmartDashboard.putNumber("Xj", commandedSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("Yj", commandedSpeeds.vyMetersPerSecond);

        this.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(commandedSpeeds, getSavedPose().getRotation()));

        //SmartDashboard.putBoolean("collision", detectCollision());

    }


    public void updateOdometryWithKinematics(){
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
        totalLoopTime += (currentTime-lastTime);
        avgLoopTIme = lowpass.calculate(currentTime - lastTime);

        StatusCode signal = BaseStatusSignal.waitForAll(2 / Constants.Swerve.ODOMETRY_UPDATE_RATE_HZ_INTEGER, allOdomSignals);
        if(signal.isError()){
            ++failedOdometryUpdates;
        }
        else{
            ++successfulOdometryUpdates;
        }

        SwerveModulePosition[] positions = getModulePositions();
        SwerveModuleState[] states = getModuleStates();
        odometryLock.writeLock().lock();
        try{
            currentModulePositions = positions;
            currentRobotPose = odometry.updateWithTime(
                Timer.getFPGATimestamp(),
                getGyroHeading(),
                currentModulePositions);
            currentModuleStates = states;
        }finally{
            odometryLock.writeLock().unlock();
        }
    }


    /*
     * updateOdometryWithVision uses vision to add measurements to the odometry
     */
    public void updateOdometryWithVision(boolean reSeedGyro){
        if(reSeedGyro){
            seedCameraHeading();
        }
        for(String cam : camNames){  
            LimelightHelpers.SetRobotOrientation(cam, getSavedPose().getRotation().getDegrees(), 0, pigeon.getPitch().getValueAsDouble(), 0, pigeon.getRoll().getValueAsDouble(), 0);
            LimelightHelpers.SetFiducialIDFiltersOverride(cam, validTagIDs);
            LimelightHelpers.PoseEstimate mt2_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
        
            // Only update pose if it is valid and if we arent spinning too fast
            if(mt2_estimate != null && mt2_estimate.tagCount != 0){//remove rotation speed limit
                ++inc;
                // Finally, we actually add the measurement to our odometry
                odometryLock.writeLock().lock();
                try{
                    
                    odometry.addVisionMeasurement
                    (
                        
                        mt2_estimate.pose, 
                        mt2_estimate.timestampSeconds,
                        
                        // This way it doesn't trust the rotation reading from the vision
                        // these are all the state stdevs
                        VecBuilder.fill(mt2_estimate.avgTagDist * 0.1/4.3, mt2_estimate.avgTagDist * 0.1/4.3, 999999999)
                    );
                }finally{
                    odometryLock.writeLock().unlock();
                }
                
                // This puts the pose reading from each camera onto the Field2d Widget,
                // Docs - https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html
                m_field.getObject(cam).setPose(mt2_estimate.pose);
                // SmartDashboard.putString("mt2 pose", mt2_estimate.pose.toString());
            }

            // SmartDashboard.putString("Logged Pose", new Pose2d(avgLLx, avgLLy, Rotation2d.fromDegrees(avgLLomega)).toString());
        }  
    }


    /**
     * Used to find the max stator current to prevent wheel slip
     * https://pro.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
     * 
     * 
     * @param voltageSupplier
     */
    public void testModules(double voltage){
        //SmartDasboard.putNumber("Swerve/Module 1/Module 1 Current", modules[0].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 2/Module 2 Current", modules[1].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 3/Module 3 Current", modules[2].getDriveStatorCurrent());
        //SmartDashboard.putNumber("Swerve/Module 4/Module 4 Current", modules[3].getDriveStatorCurrent());

        for(var mod : modules){
            mod.driveWithVoltage(voltage);
        }
    }

    public Pose2d getSavedPose(){
        Pose2d pose;
        odometryLock.readLock().lock();
        try{
            pose = currentRobotPose;
        }finally{
            odometryLock.readLock().unlock();
        }
        return pose;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("average odometry loop time", avgLoopTIme);
        SmartDashboard.putNumber("failed odometry updates", failedOdometryUpdates);
        SmartDashboard.putNumber("sucessful odometry updates", successfulOdometryUpdates);
        SmartDashboard.putString("Robot Pose", getSavedPose().toString());
        SmartDashboard.putNumber("Robot Rotation Error", robotRotationError);


        updateOdometryWithVision(false);
        m_field.setRobotPose(getSavedPose());
    }
}