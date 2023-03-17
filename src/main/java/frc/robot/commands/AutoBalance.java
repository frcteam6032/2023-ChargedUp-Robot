package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {
    private long startingTime;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

 

    public AutoBalance(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
 

        addRequirements(drivetrainSubsystem);
    }





@Override
public void initialize() {
     startingTime = System.currentTimeMillis();

}

private int MaxTime = 9000;  //Force robot to exit this command after this much time (milliseconds)
   
@Override
    public void execute() {   
        long elapsedTime = System.currentTimeMillis() - startingTime;
        
        // Intended Pseudo-Code
        // IF within first three seconds and still level THEN move backwards
        // ELSE IF angled too far forward or back THEN move in the correct direction
        // ELSE IF angle is level THEN stop moving
        // ELSE Stop Moving.
                
        final double AngleThreshold = 3;  // Threshold (in degrees) to be considered level
        
        if ((elapsedTime < 3000) && (Math.abs(m_drivetrainSubsystem.getPitch()) <= 1.5*AngleThreshold)) {
            // Uses 1.5*AngleThreshold to ensure that it doesn't stop immediately at Threshold then call itself level.
            // (i.e. Driving backward right until it tips to 3.0 degrees, then stopping)
            m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.9, 0.0, 0.0));
        }
         else if (m_drivetrainSubsystem.getPitch() > AngleThreshold) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.9, 0.0, 0.0));
         }
         else if (m_drivetrainSubsystem.getPitch() < -1 * AngleThreshold) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.9, .0, 0.0));
         }
         else if (Math.abs(m_drivetrainSubsystem.getPitch()) <= AngleThreshold) {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, .0, 0.0));
           // MaxTime = 0;
         }
         else {
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0, .0, 0.0));
         }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }


       
  

    @Override
    public boolean isFinished() {
        long elapsedTime2 = System.currentTimeMillis() - startingTime;

       return elapsedTime2 > MaxTime ? true:false; 
    }
        
}
