## FRC Team 6032 Swerve
This project is based on the Democat3457 SDS swerve-lib fork:
https://github.com/democat3457/swerve-lib

## Democat's initilization instructions
https://www.chiefdelphi.com/t/democat-sds-swervelib-with-cancoders/425355/14
1. In code, set the offsets to 0 and deploy.
1. Power cycle the robot.
1. Open Phoenix Tuner.
1. Rotate the modules until the bevel gears on the modules face the same way (for us it’s to the “left” of the robot). The wheels should be parallel to the side of the robot.
1. Use Self-Test Snapshot on the cancoders to get the reported absolute position of each cancoder. In code, put the absolute position into the offset degrees for each module.
1. Deploy.
1. Power cycle the robot.
1. Zero the gyro and make sure the robot drives forward!




## Original SDS Template Instructions:
### Configuring the code for your robot
1. Set your team number.
2. Read through and configure `frc.robot.Constants`. Parts of the code that need to be configured for your specific
robot setup are marked with comments starting with `FIXME`.
> Don't configure the `*_MODULE_STEER_OFFSET` constants yet. Those are configured after code is deployed.
3. Read through and configure `frc.robot.subsystems.DrivetrainSubsystem`. Parts of the code that need to be configured
for your specific robot setup are marked with comments starting with `FIXME`.
4. At this point deploy the code. If it endlessly crashes at startup make sure your CAN IDs are properly set and make
sure you are using the proper swerve module configurations for your hardware.

### Setting up module offsets

Now that we have code running on the robot, we can set up our module steering offsets. In order to do this we must have
our encoder values displayed to the dashboard.

> Before setting up module offsets ensure each offset is set to `-Math.toRadians(0.0)` and that code is deployed to the
> robot. This must be done each time offsets are determined.

1. Turn the robot on its side, so it is easy to move each module by hand.

> By default, module sensor information is displayed in the `Drivetrain` tab on ShuffleBoard.

2. Rotate each module so the bevel gear on the sides of each wheel are pointing to the robot's left.
> When aligning the wheels they must be as straight as possible. It is recommended to use a long straight edge such as
> a piece of 2x1 in order to make the wheels straight.

3. Record the angles of each module using the reading displayed on the dashboard.

4. Set the values of the `*_MODULE_STEER_OFFSET` constants in `Constants` to `-Math.toRadians(<the angle you recorded>)`
5. Re-deploy and try to drive the robot forwards. All wheels should stay parallel to each other.
6. Make sure all the wheels are spinning in the correct direction. If not, add 180 degrees to the offset of each wheel 
that is spinning in the incorrect direction. (I.e. `-Math.toRadians(<angle> + 180.0))`)
