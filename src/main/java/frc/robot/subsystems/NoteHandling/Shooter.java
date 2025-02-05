package frc.robot.subsystems.NoteHandling;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Interpolation.InterpolatingTable;
import frc.robot.commands.Interpolation.ShotParameter;
import frc.robot.subsystems.NoteHandling.Pivot.PivotStates;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*;


public class Shooter extends SubsystemBase {

    public enum ShooterStates{
        StateOff,
        StateMovingToRequestedState,
        StateCoast,
        StateSpeaker,
        StateSubwoofer,
        StatePodium,
        StateAmp,
        StateTrap,
        StatePass
    }

    public static ShooterStates m_shooterRequestedState;
    public static ShooterStates m_shooterCurrentState;
    
    GenericEntry shooterError;
    
    private final TalonFX m_talonRight = new TalonFX(kShooterRightPort,"Mast");
    private final TalonFX m_talonLeft = new TalonFX(kShooterLeftPort,"Mast");

    private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0).withSlot(0);


    private double desiredVelocity = 0;
    private double desiredVoltage = 0;
    DoubleSupplier distanceFromSpeaker;

    public Shooter(DoubleSupplier distanceFromSpeaker) {

      this.distanceFromSpeaker = distanceFromSpeaker;

        var talonFXConfigs = new TalonFXConfiguration();


        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = kSShooter;
        slot0Configs.kV = kVShooter;
        slot0Configs.kA = kAShooter; 

        slot0Configs.kP = kPShooter;
        slot0Configs.kI = kIShooter;
        slot0Configs.kD = kDShooter; 


        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = kShooterCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = kShooterAcceleration;
        motionMagicConfigs.MotionMagicJerk = kShooterJerk;

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        motorOutputConfigs.NeutralMode=NeutralModeValue.Coast;
       
        if (kShooterClockwisePositive)
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        var feedbackConfigs = talonFXConfigs.Feedback;
        feedbackConfigs.SensorToMechanismRatio = kSensorToMechanismGearRatio;

        var currentConfigs = talonFXConfigs.CurrentLimits;
        currentConfigs.StatorCurrentLimitEnable = true;
        currentConfigs.StatorCurrentLimit = kCurrentLimit;

        m_talonLeft.getConfigurator().apply(talonFXConfigs);
        
        if (kShooterClockwisePositive) {
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        else motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        m_talonRight.getConfigurator().apply(talonFXConfigs);
        
        m_shooterCurrentState = ShooterStates.StateOff;
        m_shooterRequestedState = ShooterStates.StateOff;

        shooterError = Shuffleboard.getTab("Swerve").add("ShooterError", 0).getEntry();
    }


    @Override
    public void periodic() {
        // if (m_shooterCurrentState == ShooterStates.StateSpeaker)
        //     shooterError.setDouble(1);
        // else shooterError.setDouble(0);

       shooterError.setDouble(getError());

        switch (m_shooterRequestedState) {
          case StateOff:
            desiredVelocity = 0; //21
            desiredVoltage = 0;
            break;
          case StateCoast:
            desiredVelocity = 0;
            desiredVoltage = 0;
            break;
          case StateSpeaker:
            desiredVelocity = InterpolatingTable.get(distanceFromSpeaker.getAsDouble()).shooterSpeedRotationsPerSecond; // -1
            desiredVoltage = 0;
            break;
          case StateAmp:
            desiredVelocity = 47.5; 
            desiredVoltage = 0;
            break;
          case StatePass:
            desiredVelocity = 50;
            desiredVoltage = 0;
            break;
        }
     
        runControlLoop();
    
        if (getError() < kShooterErrorTolerance)
          m_shooterCurrentState = m_shooterRequestedState;
        else
          m_shooterCurrentState = ShooterStates.StateMovingToRequestedState;  
        }
    
      public void runControlLoop() {
        if(desiredVelocity!=0){
            m_talonRight.setControl(request.withVelocity(desiredVelocity).withLimitReverseMotion(true).withEnableFOC(true));
            //m_talonLeft.setVoltage(m_talonRight.getMotorVoltage().getValueAsDouble());
            double voltagePercent = m_talonRight.getMotorVoltage().getValueAsDouble() / kNominalVoltage;
            m_talonLeft.setControl(new DutyCycleOut(voltagePercent).withEnableFOC(true));
            // m_talonLeft.setVoltage(.25);
            // m_talonRight.setVoltage(.25);

        }
        else{
            m_talonLeft.setVoltage(desiredVoltage);
            m_talonRight.setVoltage(desiredVoltage);
        }
      }
    
      public double getVelocity() {
        return m_talonLeft.getVelocity().getValue();
      }
    
      public double getError() {
        return Math.abs(getVelocity() - desiredVelocity);
      }
     
      // example of a "setter" method
      public void requestState(ShooterStates requestedState) {
        m_shooterRequestedState = requestedState;
      }
     
      // example of a "getter" method
      public ShooterStates getCurrentState() {
        return m_shooterCurrentState;
      }
    
      // misc methods go here, getters and setters should follow above format
    }

    




    






  