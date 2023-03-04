package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class AutoDrive extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

 

    public AutoDrive(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
 

        addRequirements(drivetrainSubsystem);
    }

  

    @Override
    public void execute() {
        
       
            
        
        
     m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.5, .0, 0.0));

                
                
        
        
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

        
}
