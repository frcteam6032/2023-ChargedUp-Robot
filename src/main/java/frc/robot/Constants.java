 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5461;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5461;

    public static final int DRIVETRAIN_PIGEON_ID = 20; // FIXED Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3; // FIXED Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 7; // FIXED Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 11; // FIXED Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(274.48243526947124); // FIXED Measure and set front left steer offset
    //public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // FIXED Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; // FIXED Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; // FIXED Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(140.88867525530583); // FIXED Measure and set front right steer offset
    //public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXED Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // FIXED Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; // FIXED Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(178.4179692726413); // FIXED Measure and set back left steer offset
    //public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0);


    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXED Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; // FIXED Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9; // FIXED Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(103.53515615626065); // FIXED Measure and set back right steer offset
    //public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0);
    
    
    
    public static final int RIGHT_ARM_SPARKMAX_CAN_ID = 14; 
    public static final int LEFT_ARM_SPARKMAX_CAN_ID = 15;

    //public static final double ARM_UPPER_LIMIT = -65; //-65 to -58
    public static final double ARM_LOWER_LIMIT = 87;
    public static final double ARM_UPPER_LIMIT = ARM_LOWER_LIMIT - 144;


    public static final double ARM_TRAVEL = ARM_LOWER_LIMIT - ARM_UPPER_LIMIT;
}
