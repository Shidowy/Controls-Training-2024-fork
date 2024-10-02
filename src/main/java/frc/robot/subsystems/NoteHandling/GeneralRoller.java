package frc.robot.subsystems.NoteHandling;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.GeneralRollerConstants.*;

import org.opencv.features2d.MSER;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class GeneralRoller extends SubsystemBase {
    /*
     * 
     * You will want to write this class with the guidance of this document, or at least the latter portion....
     * https://docs.google.com/document/d/16XcUJPxMh9GqO4A-GVb6mW3it_Yp4bLcdosbXM8OW3I/edit?usp=sharing
     * 
     * The GeneralRoller class is representative of any roller that makes contact with a note. Specifically, all rollers on Vivaldi use SparkMaxes, 
     * so the framework in this class is designed around them. Your goal is to fill the missing methods. Some variables have been provided for you,
     * but others will need to be programmed in based on your approach--- remember, programming is problem-solving, and there's multiple ways to 
     * validly complete this excersise. You may look at the Team2485/frc-2024 repo for reference, but we need you to explain the code you wrote in your
     * groups back to us. Documentation for all of the WPILib provided classes is available online:
     * Documentation sources:
     * 
     * https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax | Documentation for the CANSparkMax class, which is the motor controller you'll use.
     *  ^^^ can also navigate to other pages on these docs to find other classes related to SparkMax's, etc 
     * https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/api-overview.html | API reference for Cross the Road Electronics. Won't help too much here specifically.
     * https://github.wpilib.org/allwpilib/docs/release/java/index.html | provides docs for a lot of stuff you probably won't use here, like competition APIs. Still a good tool. 
     *
     * //GEORGE is nice and kind and cool and smart and gets good grade
     */



  // Misc variables for specific subsystem go at the top of the class, right here

  // Enum representing all of the states the subsystem can be in
  public enum GeneralRollerStates {
    StateOff,
    StateForward,
    StateReverse,
    StateForwardFast,
  }
  
  public LinearFilter filter = LinearFilter.singlePoleIIR(0.5, 0.2);

  // You generally only need one motor for the rollers on Vivaldi
  private final CANSparkMax m_spark;
  private double desiredVoltage = 0;
  private GeneralRollerStates currentState = GeneralRollerStates.StateOff;

  public GeneralRoller(int port, boolean setInverted) {
    m_spark = new CANSparkMax(port, MotorType.kBrushless);

   

    // You have been given the CANSparkMax here, which is representative of the motor driving this shaft, 
    // you still need to confige it! Look at the docs and the provided arguments to this subsystem, and determine what those configs should be.
    m_spark.setInverted(setInverted);
    m_spark.setIdleMode(IdleMode.kBrake);
    m_spark.setSmartCurrentLimit(40); 
    m_spark.enableVoltageCompensation(12);

  }

  @Override
  public void periodic() {
    //This function runs ~20 times per second. It is in every subsytem, and is effectively your "while" loop or "update" loop.
    //Thus, the usage of while(true) and similar loops is generally avoided--- they can cause memory-leaks and other jank! Instead, put looping code here. 
    switch (currentState) {
      case StateOff:
        m_spark.setVoltage(0);
        break;
      case StateForward:
        m_spark.setVoltage(6);
        break;
      case StateReverse:
        m_spark.setVoltage(-6);
        break;
      case StateForwardFast:
        m_spark.set(12);
      default:
        break;
    }
  }

  public double getCurrent() {
    //hint: this method wants you to return the Amperage (Current, or A) to the motor. the LinearFilter is useful here.
    return m_spark.getOutputCurrent(); //replace 1.0 with your return value
  }


  
  
  public void requestState(GeneralRollerStates desiredState) {
    // hint: this method is called with a GeneralRollerState when the state is to be changed.
    currentState = desiredState;
  }
 
  
  public GeneralRollerStates getCurrentState() { 
    return GeneralRollerStates.desiredState(); //You should change this!
    
  }

  // misc/processing methods go here, getters and setters should follow above format
}