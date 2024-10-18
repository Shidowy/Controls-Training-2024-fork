// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.WarlordsLib.WL_CommandXboxController;
//import frc.robot.commands.AutoCommandBuilder;
//import frc.robot.commands.ClimbCommandBuilder;
//import frc.robot.commands.DriveCommandBuilder;
//import frc.robot.commands.DriveWithController;
import frc.robot.commands.NoteHandlingCommandBuilder;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NoteHandling.GeneralRoller;
//import frc.robot.subsystems.Climb.Climber;
//import frc.robot.subsystems.NoteHandling.GeneralRoller;
import frc.robot.subsystems.NoteHandling.Intake;
//import frc.robot.subsystems.NoteHandling.Pivot;
//import frc.robot.subsystems.NoteHandling.Shooter;
//import frc.robot.subsystems.NoteHandling.Pivot.PivotStates;
//import frc.robot.subsystems.Vision.PoseEstimation;
//import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.Swerve.wheelCircumference;

import javax.swing.text.Style;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static frc.robot.Constants.GeneralRollerConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final WL_CommandXboxController m_driver = new WL_CommandXboxController(kDriverPort);
  private final WL_CommandXboxController m_operator = new WL_CommandXboxController(kOperatorPort);


  private final GeneralRoller roller = new GeneralRoller(kFeederPort, false); //Todo: replace port with an actual damn number, just give it the indexer or sm.
  GenericEntry constantsTest;


  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // constantsTest = Shuffleboard.getTab("Swerve").add("GoalXPos", 0).getEntry();
    // Configure the trigger bindings
    configureBindings();
   
   //NamedCommands.registerCommand("RollerForward", NoteHandlingCommandBuilder.generalRollerRunForward(roller));
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   
    m_driver.rightTrigger().onTrue(

      NoteHandlingCommandBuilder.generalRollerRunForward(roller)

    );
     m_driver.leftTrigger().onTrue(

      NoteHandlingCommandBuilder.generalRollerRunReverse(roller)

    );
     m_driver.rightBumper().onTrue(

      NoteHandlingCommandBuilder.generalRollerStop(roller)

    );
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand(()-> System.out.print("autochooser disabled"));
  }
}
