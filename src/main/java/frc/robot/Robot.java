// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import javax.crypto.NullCipher;

import com.ctre.phoenix.sensors.Pigeon2;
//import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  

// Making 2 new camera instances to stream footage
private UsbCamera camera = CameraServer.startAutomaticCapture(0); 
private UsbCamera camera2 = CameraServer.startAutomaticCapture(1); 

  
  //camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);









  private Spark Led_Strips = new Spark(9);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Led_Strips.set(0.77);
  }

  @Override
  public void disabledPeriodic() {
    //Led_Strips.set(-0.25);
    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Led_Strips.set(-0.25);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    


    final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5461;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */

    final double DRIVETRAIN_WHEELBASE_METERS = 0.5461;
    final double MAX_VOLTAGE = 12.0;
    final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

          final SwerveModule m_frontLeftModule;
          final SwerveModule m_frontRightModule;
          final SwerveModule m_backLeftModule;
          final SwerveModule m_backRightModule;
          ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

          final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3; // FIXED Set front left module drive motor ID
    final int FRONT_LEFT_MODULE_STEER_MOTOR = 7; // FIXED Set front left module steer motor ID
    final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; // FIXED Set front left steer encoder ID
    final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(274.48243526947124); // FIXED Measure and set front left steer offset

    final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // FIXED Set front right drive motor ID
    final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // FIXED Set front right steer motor ID
    final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; // FIXED Set front right steer encoder ID
    final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(140.88867525530583); // FIXED Measure and set front right steer offset

    final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXED Set back left drive motor ID
    final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // FIXED Set back left steer motor ID
    final int BACK_LEFT_MODULE_STEER_ENCODER = 12; // FIXED Set back left steer encoder ID
    final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(178.4179692726413); // FIXED Measure and set back left steer offset

    final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXED Set back right drive motor ID
    final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; // FIXED Set back right steer motor ID
    final int BACK_RIGHT_MODULE_STEER_ENCODER = 9; // FIXED Set back right steer encoder ID
    final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(103.53515615626065); // FIXED Measure and set back right steer offset
    final int RIGHT_ARM_SPARKMAX_CAN_ID = 14; // FIXED Set back right steer encoder ID
    final int LEFT_ARM_SPARKMAX_CAN_ID = 15; // FIXED Set back right steer encoder ID
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
       // FIXED Setup motor configuration
       m_frontLeftModule = Mk4SwerveModuleHelper.createNeo(
        // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk4SwerveModuleHelper.GearRatio.L2,
        // This is the ID of the drive motor (drive = turn wheel)
        FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor (steer = rotate wheel)
        FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
        FRONT_LEFT_MODULE_STEER_OFFSET
);

// We will do the same for the other modules
m_frontRightModule = Mk4SwerveModuleHelper.createNeo(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        FRONT_RIGHT_MODULE_STEER_MOTOR,
        FRONT_RIGHT_MODULE_STEER_ENCODER,
        FRONT_RIGHT_MODULE_STEER_OFFSET
);

m_backLeftModule = Mk4SwerveModuleHelper.createNeo(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        BACK_LEFT_MODULE_DRIVE_MOTOR,
        BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER,
        BACK_LEFT_MODULE_STEER_OFFSET
);

m_backRightModule = Mk4SwerveModuleHelper.createNeo(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        BACK_RIGHT_MODULE_DRIVE_MOTOR,
        BACK_RIGHT_MODULE_STEER_MOTOR,
        BACK_RIGHT_MODULE_STEER_ENCODER,
        BACK_RIGHT_MODULE_STEER_OFFSET
);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());


    // All stuff goes here 
   


// Change from null to something that will make it work
 final DrivetrainSubsystem m_DrivetrainSubsystem = null;
 final int DRIVETRAIN_PIGEON_ID = 20; // FIXED Set Pigeon ID

  final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);


  m_DrivetrainSubsystem.drive(
  ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0.5,  m_DrivetrainSubsystem.getGyroscopeRotation())
 );





  }

  @Override
  public void teleopInit() {

    Led_Strips.set(-0.23);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
