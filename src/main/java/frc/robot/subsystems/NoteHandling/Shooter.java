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
package frc.robot.subsystems.NoteHandling;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.commands.Interpolation.InterpolatingTable;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {

    public enum ShooterStates {
        StateOff,
        StateMovingToRequestedState,
        StateCoast,
        StateSpeaker,
        StateAmp,
        StatePass
    }

    private static ShooterStates m_shooterRequestedState = ShooterStates.StateOff;
    private static ShooterStates m_shooterCurrentState = ShooterStates.StateOff;

    private final GenericEntry shooterError;
    private final TalonFX m_talonRight;
    private final TalonFX m_talonLeft;
    private final MotionMagicVelocityVoltage requestRight;
    private final MotionMagicVelocityVoltage requestLeft;
    private final DoubleSupplier distanceFromSpeaker;

    private double desiredVelocity = 0;

    public Shooter(DoubleSupplier distanceFromSpeaker) {
        this.distanceFromSpeaker = distanceFromSpeaker;
        this.m_talonRight = new TalonFX(kShooterRightPort, "Mast");
        this.m_talonLeft = new TalonFX(kShooterLeftPort, "Mast");
          
        this.requestRight = new MotionMagicVelocityVoltage(0).withSlot(0);
        this.requestLeft = new MotionMagicVelocityVoltage(0).withSlot(0);

        configureMotors();

        shooterError = Shuffleboard.getTab("Swerve").add("ShooterError", 0).getEntry();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kPShooter;
        config.Slot0.kI = kIShooter;
        config.Slot0.kD = kDShooter;
        config.Slot0.kS = kSShooter;
        config.Slot0.kV = kVShooter;
        config.Slot0.kA = kAShooter;

        config.MotionMagic.MotionMagicCruiseVelocity = kShooterCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = kShooterAcceleration;
        config.MotionMagic.MotionMagicJerk = kShooterJerk;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = kSensorToMechanismGearRatio;

        config.CurrentLimits.StatorCurrentLimit = kCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = kShooterClockwisePositive ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;
        m_talonRight.getConfigurator().apply(config);

        config.MotorOutput.Inverted = kShooterClockwisePositive ? 
            InvertedValue.CounterClockwise_Positive : 
            InvertedValue.Clockwise_Positive;
        m_talonLeft.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        shooterError.setDouble(getError());
        updateDesiredVelocity();
        runMotionMagic();
        updateState();
    }

    private void updateDesiredVelocity() {
        switch (m_shooterRequestedState) {
            case StateOff:
            case StateCoast:
                desiredVelocity = 0;
                break;
            case StateSpeaker:
                desiredVelocity = InterpolatingTable.get(distanceFromSpeaker.getAsDouble())
                    .shooterSpeedRotationsPerSecond;
                break;
            case StateAmp:
                desiredVelocity = 47.5;
                break;
            case StatePass:
                desiredVelocity = 50;
                break;
        }
    }

    private void runMotionMagic() {
        m_talonRight.setControl(requestRight
            .withVelocity(desiredVelocity)
            .withLimitReverseMotion(true)
            .withEnableFOC(true));
            
        m_talonLeft.setControl(requestLeft
            .withVelocity(desiredVelocity)
            .withLimitReverseMotion(true)
            .withEnableFOC(true));
    }

    private void updateState() {
        if (getError() < kShooterErrorTolerance) {
            m_shooterCurrentState = m_shooterRequestedState;
        } else {
            m_shooterCurrentState = ShooterStates.StateMovingToRequestedState;
        }
    }

    public double getVelocity() {
        return (m_talonLeft.getVelocity().getValue() + 
                m_talonRight.getVelocity().getValue()) / 2.0;
    }

    public double getError() {
        return Math.abs(getVelocity() - desiredVelocity);
    }

    public void requestState(ShooterStates requestedState) {
        m_shooterRequestedState = requestedState;
    }

    public ShooterStates getCurrentState() {
        return m_shooterCurrentState;
    }

    public double getLeftVelocity() {
        return m_talonLeft.getVelocity().getValue();
    }

    public double getRightVelocity() {
        return m_talonRight.getVelocity().getValue();
    }
}
