// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


//import frc.robot.commands.*;
//import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax motorController3;
    private CANSparkMax motorController4;
    private DutyCycleEncoder m_DutyCycleEncoder;

    public ArmSubsystem() {
        m_DutyCycleEncoder = new DutyCycleEncoder(0);
        motorController3 = new CANSparkMax(14, MotorType.kBrushless);
        motorController3.setInverted(true);
    
        motorController4 = new CANSparkMax(15, MotorType.kBrushless);
        motorController4.setInverted(false);


        // Add Arm Subsystem to Competition Shuffleboard Tab
        ShuffleboardTab tab_competition = Shuffleboard.getTab("Competition");

        ShuffleboardLayout encoder_layout = tab_competition.getLayout("Arm Encoder", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(8, 0);
        //encoder_layout.addNumber("Absolute", m_DutyCycleEncoder::getAbsolutePosition);
        //encoder_layout.addNumber("Relative", m_DutyCycleEncoder::get);
        //encoder_layout.addNumber("Absolute*360", () -> m_DutyCycleEncoder.getAbsolutePosition() * 360);
        encoder_layout.addNumber("Relative*90", () -> m_DutyCycleEncoder.get() * 90)
            .withPosition(0, 0);
        encoder_layout.addNumber("Scaled 0 to 1", () -> (79-(m_DutyCycleEncoder.get() * 90))/144)
            .withPosition(0, 1);
        encoder_layout.addBoolean("Is at max height", () -> (m_DutyCycleEncoder.get() * 90 <= -65))
            .withPosition(0, 2);
        encoder_layout.addBoolean("Is at min height", () -> (m_DutyCycleEncoder.get() * 90 >= 79))
            .withPosition(0, 3);

    }

    public void set_speed(double value){
         motorController3.set(value);
        motorController4.set(value);
    }

    public double getAngle(){
      return m_DutyCycleEncoder.get() * 90;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

