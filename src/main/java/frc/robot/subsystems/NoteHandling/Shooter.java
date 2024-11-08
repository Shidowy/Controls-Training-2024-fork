package frc.robot.subsystems.NoteHandling;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*;

// FOLLOW ALONG THIS DOCUMENTATION: https://docs.google.com/document/d/143tNsvYQFAErQTJDxO9d1rwM7pv80vpLfLK-WiIEOiw/edit?tab=t.0

public class Shooter extends SubsystemBase {

    public enum ShooterStates{
        OFF,
        LOW_GOAL,
        MID_GOAL,
        HIGH_GOAL,
        TRANSITION
    }

    public static ShooterStates m_shooterRequestedState;
    public static ShooterStates m_shooterCurrentState;
 
    // CREATE TALON MOTORS HERE
    // the shooter has two talon motors on it, have fun

    // ||||||||||||||||||||||||||||||||

    private double desiredVelocity = 0;
    private double desiredVoltage = 0;

    // you might notice a new type right below here called a "DoubleSupplier," don't worry about it, you won't need to use distanceFromSpeaker for this
    // incase you were wonder though, it is a lambda, cause of course it is
    public Shooter(DoubleSupplier distanceFromSpeaker) {

        leftShooterMotor = new TalonFX(LEFT_SHOOTER_MOTOR_ID);
        rightShooterMotor = new TalonFX(RIGHT_SHOOTER_MOTOR_ID);

        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.neutralDeadband = 0.001;
        talonFXConfigs.voltageCompSaturation = 12.0;
        talonFXConfigs.neutralMode = NeutralModeValue.Coast;
        leftShooterMotor.getConfigurator().apply(talonFXConfigs);
        rightShooterMotor.getConfigurator().apply(talonFXConfigs);

        leftShooterMotor.setInverted(InvertedValue.Clockwise);
        // rightShooterMotor.setInverted(InvertedValue.CounterClockwise);

    }
        
    @Override
    public void periodic() {

        switch (m_shooterRequestedState) {
            case OFF:
                desiredVelocity = 0;
                break;
     
        runControlLoop();
    
        // ERROR CHECKING GOES HERE

        // ||||||||||||||||||||||||||||||||

    }

      public void runControlLoop() {
        // SHOOTER SHENANIGANS GO HERE UNLESS YOU ARE TOO COOL FOR THAT

        // ||||||||||||||||||||||||||||||||
      }
    
      // SO MANY METHODS TO MAKE (like 4), SO LITTLE TIME TO DO IT (literally 6 hours)

      public double getVelocity() {
        // CHANGE DIS PLZ
        return 0;
      }
    
      public double getError() {
        // CHANGE DIS PLZ
        return 0;
      }
     
      public void requestState(ShooterStates requestedState) {
        // CHANGE DIS PLZ
        m_shooterRequestedState = requestedState;
      }
     
      public ShooterStates getCurrentState() {
        // CHANGE DIS PLZ
        return m_shooterCurrentState;
      }

        // ||||||||||||||||||||||||||||||||

    }

    