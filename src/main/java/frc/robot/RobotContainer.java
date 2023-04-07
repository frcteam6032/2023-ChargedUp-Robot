package frc.robot;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoEject;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.PitLeftDrive;
import frc.robot.commands.PitRightDrive;
import frc.robot.commands.FwrdDrive;






/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

    private final XboxController m_controller = new XboxController(0);
    private final XboxController m_controller2 = new XboxController(1);
    public final IntakeSubsystem m_intake = new IntakeSubsystem();
    public final ArmSubsystem m_arm = new ArmSubsystem();


    private final Command IntakePickupCommand = new Intake_Pickup(m_intake);
    private final Command IntakeEjectCommand = new Intake_Eject(m_intake);
    private final Command ArmRaiseCommand = new ArmRaise(m_arm);
    private final Command ArmLowerCommand = new ArmLower(m_arm);
    private final Command AutoDriveCommand = new AutoDrive(m_drivetrainSubsystem);
    private final Command AutoDriveCommand2 = new AutoDrive(m_drivetrainSubsystem);
    private final Command AutoDriveCommand3 = new AutoDrive(m_drivetrainSubsystem);

    private final Command AutoEjectCommand = new AutoEject(m_intake);
    private final Command AutoArmRaiseCommand = new AutoArmRaise(m_arm);
    private final Command AutoArmLowerCommand = new AutoArmLower(m_arm);
    private final Command AutoEjectCommand2 = new AutoEject(m_intake);
    private final Command AutoEjectCommand3 = new AutoEject(m_intake);
    private final Command AutoEjectCommand4 = new AutoEject(m_intake);
    private final Command AutoEjectCommand5 = new AutoEject(m_intake);


    private final Command AutoArmRaiseCommand2 = new AutoArmRaise(m_arm);
    private final Command AutoArmLowerCommand2 = new AutoArmLower(m_arm);
    private final Command AutoArmRaiseCommand3 = new AutoArmRaise(m_arm);
    private final Command AutoArmLowerCommand3 = new AutoArmLower(m_arm);
    private final Command AutoBalanceCommand = new AutoBalance(m_drivetrainSubsystem);
    private final Command AutoBalanceCommand2 = new AutoBalance(m_drivetrainSubsystem);

    // A chooser for autonomous commands
    SendableChooser < Command > m_chooser = new SendableChooser < > ();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:

        // Originally use right stick FWD/BACK/Left/Right, Left Stick Pivot

        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            // Forwards And Back
            () -> -modifyAxis(m_controller.getRightY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            // Left & Right
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            // Pivot
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        // Configure the button bindings
        configureButtonBindings();


        // Add commands to the autonomous command chooser
        m_chooser.setDefaultOption("No drive Auto", (AutoArmRaiseCommand2.withTimeout(3)).andThen(AutoEjectCommand2).andThen(AutoArmLowerCommand2.withTimeout(3.5)));
        m_chooser.addOption("Full Auto Balance", (AutoArmRaiseCommand3.withTimeout(2)).andThen(AutoEjectCommand3).andThen(AutoArmLowerCommand3.withTimeout(1.5)).andThen(AutoBalanceCommand)); //AutoDriveCommand
        m_chooser.addOption("Full Auto Drive Far", (AutoArmRaiseCommand.withTimeout(3)).andThen(AutoEjectCommand).andThen(AutoArmLowerCommand.withTimeout(3.5)).andThen(AutoDriveCommand)); //AutoDriveCommand
        m_chooser.addOption("Eject Only", AutoEjectCommand5);
        m_chooser.addOption("Eject n Drive", AutoEjectCommand4.andThen(AutoDriveCommand3));
        m_chooser.addOption("Drive Only Far", AutoDriveCommand2); // AutoDriveCommand2
        m_chooser.addOption("Drive Only Balance", AutoBalanceCommand2);

        // Put the chooser on the dashboard
        Shuffleboard.getTab("Competition")
            .add("Auto Chooser", m_chooser)
            .withPosition(6, 3)
            .withSize(2, 1);


        // Add a pit control option that doesn't need a controller
        ShuffleboardTab tab_pit = Shuffleboard.getTab("Pit Tests");
        tab_pit.add("Arm Raise", new AutoArmRaise(m_arm))
            .withSize(2, 1).withPosition(0, 0);
        tab_pit.add("Arm Lower", new AutoArmLower(m_arm))
            .withSize(2, 1).withPosition(0, 1);
        tab_pit.add("Intake Eject", new AutoEject(m_intake))
            .withSize(2, 1).withPosition(2, 0);
        tab_pit.add("Auto Drive", new AutoDrive(m_drivetrainSubsystem))
            .withSize(2, 1).withPosition(4, 0);

        // Either a button for each direction, or a complex command which moves in each direction automatically
        tab_pit.add("Forward", new FwrdDrive(m_drivetrainSubsystem));
        tab_pit.add("Left", new PitLeftDrive(m_drivetrainSubsystem));
        tab_pit.add("Right", new PitRightDrive(m_drivetrainSubsystem));
        tab_pit.add("Reverse", new AutoDrive(m_drivetrainSubsystem));

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

        // Set Button for Intake Pickup
        new Trigger(m_controller2::getXButton).whileTrue(IntakePickupCommand);

        // Set Button for Intake Eject
        new Trigger(m_controller2::getBButton).whileTrue(IntakeEjectCommand);

        // Set Button for Arm Raise
        new Trigger(m_controller2::getRightBumper).whileTrue(ArmRaiseCommand);

        // Set Button for Arm Lower
        new Trigger(m_controller2::getLeftBumper).whileTrue(ArmLowerCommand);
        
        new Trigger(m_controller2::getStartButton).onTrue(Commands.runOnce(() -> m_arm.reset()));

        new Trigger(m_controller::getStartButton).onTrue(Commands.runOnce(() -> m_drivetrainSubsystem.resetWheels()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
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

    public void setGyroscope(double angle) {
        m_drivetrainSubsystem.setGyroscope(angle);
    }
}