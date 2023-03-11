package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenixpro.signals.System_StateValue;

public class AutoDrive extends CommandBase {
    private long startingTime;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

 

    public AutoDrive(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
 

        addRequirements(drivetrainSubsystem);
    }





@Override
public void initialize() {
     startingTime = System.currentTimeMillis();

}


    @Override
    public void execute() {    
     m_drivetrainSubsystem.drive(new ChassisSpeeds(-0.6, .0, 0.0));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }


       
  
    private final int MaxTime = 5000;

    @Override
    public boolean isFinished() {
         long elapsedTime =  System.currentTimeMillis() - startingTime;
       return elapsedTime > MaxTime ? true:false; 
    }
        
}
