package frc.robot.subsystems.SubFolder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SubsystemNameConstants.*;

import static frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Intake extends SubsystemBase {

  public enum Intake {
    StateOff,
    StateOuttake,
    StateIntake,
  }


 public static IntakeStates m_subsystemNameCurrentState;
 public static IntakeStates m_subsystemNameRequestedState;


  private final CANSparkMax m_spark = new CANSparkMax(kIntakePort, MotorType.kBrushless);

  private double desiredVoltage = 0;

  public Intake() {
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
      case StateOff:
        desiredVoltage = 0;
        break;
      case StateIntake:
        desiredVoltage = 5;
        break;
      case StateOuttake:
        desiredVoltage = -5;
        break;
    }
	
    runControlLoop();

    m_subsystemNameCurrentState = m_subsystemNameRequestedState;
  }
}