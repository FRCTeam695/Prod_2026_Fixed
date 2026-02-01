// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LoggingSubsystem extends SubsystemBase {

  private NetworkTable joystickTable;
  
  //buttonA
  private BooleanPublisher buttonAStatus;
  private BooleanTopic buttonA;
  private boolean pressedA = false;

  //buttonY
  private BooleanPublisher buttonYStatus;
  private BooleanTopic buttonY;
  private boolean pressedY = false;

  //buttonB
  private BooleanPublisher buttonBStatus;
  private BooleanTopic buttonB;
  private boolean pressedB = false;

  //buttonX
  private BooleanPublisher buttonXStatus;
  private BooleanTopic buttonX;
  private boolean pressedX = false;

  //leftBumper
  private BooleanPublisher leftBumperStatus;
  private BooleanTopic leftBumper;
  private boolean pressedLBumper = false;

  //rightBumper
  private BooleanPublisher rightBumperStatus;
  private BooleanTopic rightBumper;
  private boolean pressedRBumper = false;

  //leftTrigger
  private BooleanPublisher leftTriggerStatus;
  private BooleanTopic leftTrigger;
  private boolean pressedLTrigger = false;

  //rightTrigger
  private BooleanPublisher rightTriggerStatus;
  private BooleanTopic rightTrigger;
  private boolean pressedRTrigger = false;
  
  public LoggingSubsystem() {

    //initializing NT, boolean topics and boolean publishers for each button
    
    joystickTable = NetworkTableInstance.getDefault().getTable("Joystick");
    
    buttonA = joystickTable.getBooleanTopic("ButtonA");
    buttonAStatus = buttonA.publish();

    buttonY = joystickTable.getBooleanTopic("ButtonY");
    buttonYStatus = buttonY.publish();

    buttonB = joystickTable.getBooleanTopic("ButtonB");
    buttonBStatus = buttonB.publish();

    buttonX = joystickTable.getBooleanTopic("ButtonX");
    buttonXStatus = buttonX.publish();

    leftBumper = joystickTable.getBooleanTopic("leftBumper");
    leftBumperStatus = leftBumper.publish();

    rightBumper = joystickTable.getBooleanTopic("rightBumper");
    rightBumperStatus = rightBumper.publish();

    leftTrigger = joystickTable.getBooleanTopic("leftTrigger");
    leftTriggerStatus = leftTrigger.publish();

    rightTrigger = joystickTable.getBooleanTopic("rightTrigger");
    rightTriggerStatus = rightTrigger.publish();
  }


  //command for getting raw boolean values from controller in robot container and then sending them to the publishers to be published to the topics

  public Command buttonsPressed(BooleanSupplier A, BooleanSupplier Y, BooleanSupplier B, BooleanSupplier X, BooleanSupplier lBumper, BooleanSupplier rBumper, BooleanSupplier lTrigger, BooleanSupplier rTrigger){
    return run(
      () -> {
        pressedA = A.getAsBoolean();
        pressedY = Y.getAsBoolean();
        pressedB = B.getAsBoolean();
        pressedX = X.getAsBoolean();
        pressedLBumper = lBumper.getAsBoolean();
        pressedRBumper = rBumper.getAsBoolean();
        pressedLTrigger = lTrigger.getAsBoolean();
        pressedRTrigger = rTrigger.getAsBoolean();
      }
    );
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  //sets all "pressed" booleans as respective publishers to be published to the boolean topics
  @Override
  public void periodic() {

    buttonAStatus.set(pressedA);
    buttonYStatus.set(pressedY);
    buttonBStatus.set(pressedB);
    buttonXStatus.set(pressedX);
    leftBumperStatus.set(pressedLBumper);
    rightBumperStatus.set(pressedRBumper);
    leftTriggerStatus.set(pressedLTrigger);
    rightTriggerStatus.set(pressedRTrigger);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
