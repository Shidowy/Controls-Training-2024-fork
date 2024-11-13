package frc.robot.subsystems.NoteHandling;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private enum ShooterStates {
        OFF, MOVING_TO_REQUESTED, COAST, SPEAKER, AMP, PASS
    }

    private ShooterStates requestedState = ShooterStates.OFF;
    private ShooterStates currentState = ShooterStates.OFF;

    private final GenericEntry shooterError;
    private final TalonFX talonRight;
    private final TalonFX talonLeft;
    private final MotionMagicVelocityVoltage requestRight;
    private final MotionMagicVelocityVoltage requestLeft;
    private final DoubleSupplier distanceFromSpeaker;

    private double desiredVelocity = 0;

    public Shooter(DoubleSupplier distanceFromSpeaker) {
        this.distanceFromSpeaker = distanceFromSpeaker;
        this.talonRight = new TalonFX(kShooterRightPort, "Mast");
        this.talonLeft = new TalonFX(kShooterLeftPort, "Mast");
        
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

        config.MotorOutput.Inverted = kShooterClockwisePositive ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonRight.getConfigurator().apply(config);

        config.MotorOutput.Inverted = kShooterClockwisePositive ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        talonLeft.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        shooterError.setDouble(getError());
        updateDesiredVelocity();
        executeMotionMagic();
        updateState();
    }

    private void updateDesiredVelocity() {
        switch (requestedState) {
            case OFF:
            case COAST:
                desiredVelocity = 0;
                break;
            case SPEAKER:
                desiredVelocity = InterpolatingTable.get(distanceFromSpeaker.getAsDouble()).shooterSpeedRotationsPerSecond;
                break;
            case AMP:
                desiredVelocity = 6;
                break;
            case PASS:
                desiredVelocity = 12;
                break;
        }
    }

    private void executeMotionMagic() {
        talonRight.setControl(requestRight.withVelocity(desiredVelocity).withLimitReverseMotion(true).withEnableFOC(true));
        talonLeft.setControl(requestLeft.withVelocity(desiredVelocity).withEnableFOC(true));
    }

    private void updateState() {
        currentState = getError() < kShooterErrorTolerance ? requestedState : ShooterStates.MOVING_TO_REQUESTED;
    }

    public double getVelocity() {
        return (talonLeft.getVelocity().getValue() + talonRight.getVelocity().getValue()) / 2.0;
    }

    public double getError() {
        return Math.abs(getVelocity() - desiredVelocity);
    }

    public void requestState(ShooterStates state) {
        requestedState = state;
    }

    public ShooterStates getCurrentState() {
        return currentState;
    }

    // public double getLeftVelocity() {
    //     return talonLeft.getVelocity().getValue();
    // }

    // public double getRightVelocity() {
    //     return talonRight.getVelocity().getValue();
    // }
}
