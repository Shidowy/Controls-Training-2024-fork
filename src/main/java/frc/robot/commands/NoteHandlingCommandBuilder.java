package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.NoteHandling.GeneralRoller;
import frc.robot.subsystems.NoteHandling.GeneralRoller.GeneralRollerStates;

import java.util.function.Supplier;

// We are gonna be writing commands today, use this doc to progress through the training:
// https://docs.google.com/document/d/1iaDaoYRCIEgX1hY3d1hFmAgDCdyEqP5SY_TVuxs04kA/edit?tab=t.0

public class NoteHandlingCommandBuilder {

    public static Command generalRollerRunForward(GeneralRoller generalRoller) {
        return new InstantCommand(() -> generalRoller.requestState(GeneralRollerStates.StateForward), generalRoller);
    }
    

}