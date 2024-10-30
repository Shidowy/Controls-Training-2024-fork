package frc.robot.subsystems.SubFolder;

// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SubsystemNameConstants.*;
import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SubsystemName extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum SubsystemNameStates {
    StateOff,
    StateOuttake,
    StateIntake,
  }

  public static SubsystemNameStates m_subsystemNameCurrentState;
  public static SubsystemNameStates m_subsystemNameRequestedState;

  // You may need more than one motor
  private final CANSparkMax m_spark = new CANSparkMax(kSubsystemNamePort, MotorType.kBrushless);
  // Units depend on the units of the setpoint() and calculate() methods. This example will use meters
  private final PIDController m_feedback = new PIDController(kPSubsystemName,
  	kISubsystemName,
  	kDSubsystemName);
  private double feedbackOutput = 0;

  private double desiredPosition = 0;

  public SubsystemName() {
    // Misc setup goes here

    m_spark.setSmartCurrentLimit(kSubsystemNameCurrentLimit);

    m_spark.setInverted(kSubsystemInverted); 	

    m_subsystemNameCurrentState = SubsystemNameStates.StateTheSubsystemStartsIn;
    m_subsystemNameRequestedState = SubsystemNameStates.StateTheSubsystemStartsIn;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
    m_spark.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    switch (m_subsystemNameRequestedState) {
      case StateA:
        desiredPosition = 0;
        break;
      case StateB:
        desiredPosition = 0;
        break;
      ...
    }
	
    runControlLoop();

    if (getError() < kSubsystemNameErrorTolerance) 
      m_subsystemNameCurrentState = m_subsystemNameRequestedState;
    else
      m_subsystemNameCurrentState = SubsystemNameStates.StateMovingToRequestedState;	
  }

  public void runControlLoop() {
    feedbackOutput = m_feedback.calculate(getPosition(), desiredPosition);
    m_spark.set(feedbackOutput / kNominalVoltage);
  }

  private double getPosition() {
    return m_spark.getEncoder().getPosition() * kSubsystemMetersPerRev;
  }

  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }
  
  // example of a "setter" method
  public void requestState(SubsystemNameStates desiredState) {
    m_subsystemNameRequestedState = desiredState;
  }
 
  // example of a "getter" method
  public SubsystemNameStates getCurrentState() { 
    return m_subsystemNameCurrentState; 
  }

  // misc methods go here, getters and setters should follow above format
}

