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
        // MAKE STATES
        // some considerations: off state, states for shooter at each type of scoring location, and a transition state between states

        // ||||||||||||||||||||||||||||||||
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

        // CREATE THE CONFIGURATIONS FOR THE TALONS HERE
        // talon configs are set up differently than sparks, please use the doc if you want to spare your sanity
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput = new MotorOutput();
        talonFXConfigs.MotorOutput.neutralMode = NeutralModeValue.Coast;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Positive_Clockwise;

        talonFXConfigs.CurrentLimits = new CurrentLimitConfiguration();
        talonFXConfigs.CurrentLimits.enable = true;
        talonFXConfigs.CurrentLimits.currentLimit = 40;

        talonFXMotor1 = new TalonFX(1);
        talonFXMotor2 = new TalonFX(2);
        talonFXMotor1.apply(talonFXConfigs);
        talonFXMotor2.apply(talonFXConfigs);
    }
        
    @Override
    public void periodic() {


        // SWITCH/IF STATEMENT GOES HERE

        // ||||||||||||||||||||||||||||||||
     
        runControlLoop();
    
        // ERROR CHECKING GOES HERE

        // ||||||||||||||||||||||||||||||||

    }

      public void runControlLoop() {
        // SHOOTER SHENANIGANS GO HERE UNLESS YOU ARE TOO COOL FOR THAT
        // create a Dynamic Motion Magic request, voltage output
        // default velocity of 80 rps, acceleration of 400 rot/s^2, and jerk of 4000 rot/s^3
        final DynamicMotionMagicVoltage m_request = new DynamicMotionMagicVoltage(0, 80, 400, 4000);

        if (m_joy.getAButton()) {
          // while the joystick A button is held, use a slower profile
          m_request.Velocity = 40; // rps
          m_request.Acceleration = 80; // rot/s^2
          m_request.Jerk = 400; // rot/s^3
        } else {
          // otherwise use a faster profile
          m_request.Velocity = 80; // rps
          m_request.Acceleration = 400; // rot/s^2
          m_request.Jerk = 4000; // rot/s^3
        }

// set target position to 100 rotations
        m_talonFX.setControl(m_request.withPosition(100));
        // ||||||||||||||||||||||||||||||||
      }
    
      // SO MANY METHODS TO MAKE (like 4), SO LITTLE TIME TO DO IT (literally 6 hours)

      public double getVelocity() {
        // CHANGE DIS PLZ
        return 0;
      }
    
      public double getError() {
        return abs(shooter);
      }
     
      public void requestState(ShooterStates requestedState) {
        // CHANGE DIS PLZ
      }
     
      public ShooterStates getCurrentState() {
        return ;
      }

        // ||||||||||||||||||||||||||||||||

    }

    