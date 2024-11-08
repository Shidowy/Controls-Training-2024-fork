package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*;

public class Shooter extends SubsystemBase {

    public enum ShooterStates {
        OFF,
        LOW_GOAL,
        HIGH_GOAL,
        TRANSITION
    }

    public static ShooterStates m_shooterRequestedState = ShooterStates.OFF;
    public static ShooterStates m_shooterCurrentState = ShooterStates.OFF;
 
    private TalonFX leftShooterMotor;
    private TalonFX rightShooterMotor;

    private PIDController pidController;
    
    private double desiredVelocity = 0;
    private double desiredVoltage = 0;

    public Shooter(DoubleSupplier distanceFromSpeaker) {

        leftShooterMotor = new TalonFX(kShooterLeftPort, "ShooterControlLoop");
        rightShooterMotor = new TalonFX(kShooterRightPort, "ShooterControlLoop");

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.voltageCompSaturation = 12.0;
        talonFXConfigs.neutralDeadband = 0.01;
        leftShooterMotor.getConfigurator().apply(talonFXConfigs);
        rightShooterMotor.getConfigurator().apply(talonFXConfigs);

        m_shooterCurrentState = ShooterStates.OFF;

        pidController = new PIDController(0.1, 0.0, 0.0); 
    }

    @Override
    public void periodic() {
        if (m_shooterRequestedState != m_shooterCurrentState) {
            handleStateChange();
        }

        runControlLoop();
    }

    public void handleStateChange() {
        switch (m_shooterRequestedState) {
            case OFF:
                leftShooterMotor.set(ControlMode.PercentOutput, 0);
                rightShooterMotor.set(ControlMode.PercentOutput, 0);
                break;
            case LOW_GOAL:
                desiredVelocity = 1500; 
                break;
            case HIGH_GOAL:
                desiredVelocity = 3000;
                break;
            case TRANSITION:
                break;
        }

        m_shooterCurrentState = m_shooterRequestedState;
    }

    public void runControlLoop() {

        double currentVelocity = getVelocity();
        double velocityError = desiredVelocity - currentVelocity;

        double pidOutput = pidController.calculate(currentVelocity, desiredVelocity);

        leftShooterMotor.set(ControlMode.PercentOutput, pidOutput);
        rightShooterMotor.set(ControlMode.PercentOutput, pidOutput);
    }

    public double getVelocity() {
        return leftShooterMotor.getSelectedSensorVelocity();
    }

    public double getError() {
        return desiredVelocity - getVelocity();
    }

    public void requestState(ShooterStates requestedState) {
        m_shooterRequestedState = requestedState;
    }

    public ShooterStates getCurrentState() {
        return m_shooterCurrentState;
    }
}
