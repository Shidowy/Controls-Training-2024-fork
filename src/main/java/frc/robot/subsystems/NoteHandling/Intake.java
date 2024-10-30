package frc.robot.subsystems.SubFolder;


// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static frc.robot.Constants.SubsystemNameConstants.*;
import static frc.robot.Constants.*;


import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Intake extends SubsystemBase {
  // Misc variables for specific subsystem go here


  // Enum representing all of the states the subsystem can be in
  public enum Intake {
    StateOff,
    StateOuttake,
    StateIntake,
  }


 public static IntakeStates m_subsystemNameCurrentState;
 public static IntakeStates m_subsystemNameRequestedState;

  // You may need more than one motor
  private final CANSparkMax m_spark = new CANSparkMax(kIntakePort, MotorType.kBrushless);
  // Units depend on the units of the setpoint() and calculate() methods. This example will use meters
  private double desiredVoltage = 0;

  public Intake() {
    // Misc setup goes here
    m_spark.setSmartCurrentLimit(kIntakeCurrentLimit);
    m_spark.setInverted(kIntakeInverted); 	
    m_spark.enableVoltageCompensation(kNominalVoltage);
    m_spark.setIdleMode(IdleMode.kCoast);

    m_subsystemNameCurrentState = IntakeStates.StateOff;
    m_subsystemNameRequestedState = IntakeStates.StateOff;
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





