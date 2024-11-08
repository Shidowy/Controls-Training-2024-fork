package frc.robot.subsystems.NoteHandling;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.PivotConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Pivot extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum PivotStates {
    StateDown,
    StateAmp,
    StateMovingToRequestedState,
    StateShooter,
    StateOuttake,
    StateAutoIntake,
    StatePass,
    StatePodiumSetpoint,
    StateStageSetpoint,
  }

  public static PivotStates m_PivotCurrentState;
  public static PivotStates m_PivotRequestedState;

  GenericEntry armPosition;

  // You may need more than one motor
  private final TalonFX m_talon = new TalonFX(kPivotPort, "Mast");
  DoubleSupplier angle; 
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
  // Unit default for TalonFX libraries is rotations
  private double desiredPosition = 0;

  public Pivot(DoubleSupplier angle) {

    this.angle = angle;
    // Misc setup goes here

    armPosition = Shuffleboard.getTab("Swerve").add("ArmPosition", 0).getEntry();

    var talonFXConfigs = new TalonFXConfiguration();
    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kV outputs n volts when the velocity target is 1 rotation per second
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = kSPivot;
    slot0Configs.kV = kVPivot;
    slot0Configs.kP = kPPivot;
    slot0Configs.kI = kIPivot;
    slot0Configs.kD = kDPivot;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kPivotCruiseVelocity;
    // vel/acc = time to reach constant velocity
    motionMagicConfigs.MotionMagicAcceleration = kPivotAcceleration;
    // acc/jerk = time to reach constant acceleration
    motionMagicConfigs.MotionMagicJerk = kPivotJerk;

    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kPivotClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = kSensorToMechanismGearRatio;

    var currentConfigs = talonFXConfigs.CurrentLimits;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs .StatorCurrentLimit = kCurrentLimit;

    m_talon.getConfigurator().apply(talonFXConfigs);

    m_PivotCurrentState = PivotStates.StateDown;
    m_PivotRequestedState = PivotStates.StateDown;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
    m_talon.setPosition(0);
  }

  @Override
  public void periodic() {

    armPosition.setDouble(getError());

    switch (m_PivotRequestedState) {
      case StateDown:
        desiredPosition = 0;
        break;
      case StateAmp:
        desiredPosition = .25;
        break;
      case StateOuttake:
        desiredPosition = .06;
        break;
      case StateAutoIntake:
        desiredPosition = 0;
        break;
      case StateShooter:
        desiredPosition = MathUtil.clamp(angle.getAsDouble(), 0, .25);
        break;
      case StatePass:
        desiredPosition = 0.05;
        break; 
      case StatePodiumSetpoint:
        desiredPosition = 0.075;
        break;
      case StateStageSetpoint:
        desiredPosition = 0.099;
        break;
    }
 
    runControlLoop();

    if (getError() < kPivotErrorTolerance)
      m_PivotCurrentState = m_PivotRequestedState;
    else
      m_PivotCurrentState = PivotStates.StateMovingToRequestedState;  
  }

  public void runControlLoop() {
    m_talon.setControl(request.withPosition(desiredPosition));
  }

  private double getPosition() {
    return m_talon.getPosition().getValue();
  }

  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }
 
  // example of a "setter" method
  public void requestState(PivotStates requestedState) {
    m_PivotRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public PivotStates getCurrentState() {
    return m_PivotCurrentState;
  }

  // misc methods go here, getters and setters should follow above format
}