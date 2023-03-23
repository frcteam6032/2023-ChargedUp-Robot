// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
//import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    // FIXED Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *
            SdsModuleConfigurations.MK4_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    // FIXED Remove if you are using a Pigeon
    private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);

    // FIXED Uncomment if you are using a NavX
    // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {
        ShuffleboardTab tab_drivetrain = Shuffleboard.getTab("Drivetrain");
        ShuffleboardTab tab_competition = Shuffleboard.getTab("Competition");

        MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerNEO();
        moduleConfig.setDriveCurrentLimit(40.0);
        moduleConfig.setSteerCurrentLimit(20.0);

        // Democat swerve-lib module construction
        m_frontLeftModule= new MkSwerveModuleBuilder(moduleConfig)
            .withLayout(tab_drivetrain.getLayout("Front Left Module", BuiltInLayouts.kList)
                            .withSize(2, 4)
                            .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
            .build();

        m_frontRightModule= new MkSwerveModuleBuilder(moduleConfig)
            .withLayout(tab_drivetrain.getLayout("Front Right Module", BuiltInLayouts.kList)
                            .withSize(2, 4)
                            .withPosition(2, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
            .build();

        m_backLeftModule= new MkSwerveModuleBuilder(moduleConfig)
            .withLayout(tab_drivetrain.getLayout("Back Left Module", BuiltInLayouts.kList)
                            .withSize(2, 4)
                            .withPosition(4, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.NEO, BACK_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, BACK_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER)
            .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
            .build();

        m_backRightModule= new MkSwerveModuleBuilder(moduleConfig)
            .withLayout(tab_drivetrain.getLayout("Back Right Module", BuiltInLayouts.kList)
                            .withSize(2, 4)
                            .withPosition(6, 0))
            .withGearRatio(SdsModuleConfigurations.MK4_L2)
            .withDriveMotor(MotorType.NEO, BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotor(MotorType.NEO, BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER)
            .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
            .build();

        // Set each steer motor encoder equal to the CANCoder absolute after a brief pause
        //https://www.chiefdelphi.com/t/democat-sds-swervelib-with-cancoders/425355/23?page=2
        Timer.delay(1.0);
        m_frontLeftModule.resetToAbsolute();
        m_frontRightModule.resetToAbsolute();
        m_backLeftModule.resetToAbsolute();
        m_backRightModule.resetToAbsolute();

        // Burn all the SparkMAX Configs to Flash (in case of power drop out)
        CANSparkMax motor_fld = (CANSparkMax) m_frontLeftModule.getDriveMotor();
        CANSparkMax motor_fls = (CANSparkMax) m_frontLeftModule.getSteerMotor();
        CANSparkMax motor_frd = (CANSparkMax) m_frontRightModule.getDriveMotor();
        CANSparkMax motor_frs = (CANSparkMax) m_frontRightModule.getSteerMotor();
        CANSparkMax motor_bld = (CANSparkMax) m_backLeftModule.getDriveMotor();
        CANSparkMax motor_bls = (CANSparkMax) m_backLeftModule.getSteerMotor();
        CANSparkMax motor_brd = (CANSparkMax) m_backRightModule.getDriveMotor();
        CANSparkMax motor_brs = (CANSparkMax) m_backRightModule.getSteerMotor();
        motor_fld.burnFlash();
        motor_fls.burnFlash();
        motor_frd.burnFlash();
        motor_frs.burnFlash();
        motor_bld.burnFlash();
        motor_bls.burnFlash();
        motor_brd.burnFlash();
        motor_brs.burnFlash();
      
        // Add the Gyroscope readings to the Shuffleboard Competition Tab
        ShuffleboardLayout gyro_layout = tab_competition.getLayout("Pigeon Gyro", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(6, 0);
        //gyro_layout.addNumber("Roll", m_pigeon::getRoll);
        gyro_layout.addNumber("Pitch", m_pigeon::getPitch);
        gyro_layout.addNumber("Heading", () -> -1*m_pigeon.getYaw()).withWidget(BuiltInWidgets.kGyro);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
    }

    // Set the gyroscope to a specified value (likely 180 when starting match at that pose)
    public void setGyroscope(double angle) {
        m_pigeon.setYaw(angle);
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

    // Make gyro readings ccessible via drivetrain subsystem methods
    public double getYaw() {return m_pigeon.getYaw();}
    public double getPitch() {return m_pigeon.getPitch();}
    public double getRoll() {return m_pigeon.getRoll();}

}
