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

private int MaxTime = 9000;
   
@Override
    // If the yaw is greater than 1
    public void execute() {   
        long elapsedTime = System.currentTimeMillis() - startingTime;
        final double AngleThreshold = 3;
        if (elapsedTime < 3000) {
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
            MaxTime = 0;
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
