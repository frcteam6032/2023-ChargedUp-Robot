// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Intake_Pickup;
import frc.robot.commands.Intake_Eject;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.ArmLower;
import frc.robot.commands.ArmRaise;
import frc.robot.commands.AutoArmLower;
import frc.robot.commands.AutoArmRaise;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoEject;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller2 = new XboxController(1);
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();


  private final Command IntakePickupCommand = new Intake_Pickup(m_intake);
  private final Command IntakeEjectCommand = new Intake_Eject(m_intake);
  private final Command ArmRaiseCommand = new ArmRaise(m_arm);
  private final Command ArmLowerCommand = new ArmLower(m_arm);
  private final Command AutoDriveCommand = new AutoDrive(m_drivetrainSubsystem);
  private final Command AutoEjectCommand = new AutoEject(m_intake);
  private final Command AutoArmRaiseCommand = new AutoArmRaise(m_arm);
   private final Command AutoArmLowerCommand = new AutoArmLower(m_arm);
   private final Command AutoEjectCommand2 = new AutoEject(m_intake);
  private final Command AutoArmRaiseCommand2 = new AutoArmRaise(m_arm);
   private final Command AutoArmLowerCommand2 = new AutoArmLower(m_arm);


  //private final AutoSetWeels = new AutoSetWeels();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:

    // Originally use right stick FWD/BACK/Left/Right, Left Stick Pivot
    // Now use left stick FWD/BACK/Left/Right, Right Stick Pivot
  
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            // Forwards And Back
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            // Left & Right
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            // Pivot
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
    
    // Add commands to the autonomous command chooser
  //  m_chooser.addOption("Eject only", (AutoArmRaiseCommand.withTimeout(4)).andThen(AutoEjectCommand).andThen(AutoArmLowerCommand.withTimeout(3.5)));

   // m_chooser.setDefaultOption("Full auto", (AutoEjectCommand).andThen(AutoDriveCommand));

   m_chooser.addOption("No drive Auto", (AutoArmRaiseCommand2.withTimeout(4)).andThen(AutoEjectCommand2).andThen(AutoArmLowerCommand2.withTimeout(3.5)));

   m_chooser.addOption("Full Auto", (AutoArmRaiseCommand.withTimeout(4)).andThen(AutoEjectCommand).andThen(AutoArmLowerCommand.withTimeout(3.5)).andThen(AutoDriveCommand));

    // Put the chooser on the dashboard
     Shuffleboard.getTab("Competition")
      .add("Auto Chooser",m_chooser)
      .withPosition(6, 3)
      .withSize(2, 1);

  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // X button zeros the gyroscope
    new Trigger(m_controller::getBButton)
            // No requirements because we don't need to interrupt anything
            
            .onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.zeroGyroscope()));
            
    // Lambda expression are function that return a vale, calls a different function, or can call a method. They do not accept conditionals such as "if" statements unless braces are used. example: parameter -> expression; or () -> Sysyem.out.println("Hello, World"); if you are farmilar with arrow functions in other languages (SUch as Javascript) which have a syntax like so: param => expression (You can also use braces to to more complex operations). Overall, a lambda expression is a function that takes a paramater and an expression and can call, return, etc something. 
    
    // Set Button for Intake Pickup
    new Trigger(m_controller2::getXButton).whileTrue(IntakePickupCommand);
    //getXButton
    // Set Button for Intake Pickup
    new Trigger(m_controller2::getBButton).whileTrue(IntakeEjectCommand);


 

     new Trigger(m_controller2::getRightBumper).whileTrue(ArmRaiseCommand);

     new Trigger(m_controller2::getLeftBumper).whileTrue(ArmLowerCommand);

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return AutoDriveCommand;
    //return (IntakePickupCommand.withTimeout(3)).andThen((IntakeEjectCommand).withTimeout(2)).andThen(AutoDriveCommand);

    return m_chooser.getSelected();

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }




}
