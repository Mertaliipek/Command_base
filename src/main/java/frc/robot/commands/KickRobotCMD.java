package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class KickRobotCMD extends CommandBase{
  private final DriveTrain driveSubsystem;
  private final double slowSpeed;
  private final double moreSpeed;
  
  public KickRobotCMD(DriveTrain driveSubsystem, double slowSpeed,double moreSpeed ) {
    this.driveSubsystem = driveSubsystem;
    this.slowSpeed = slowSpeed;
    this.moreSpeed = moreSpeed;
    addRequirements(driveSubsystem);
  } 
         // Called when the command is initially scheduled.
  
  
  
  @Override
  public void initialize() {
    System.out.print("Kick robot  started");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.letsKickrobot(slowSpeed,moreSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Kick robot ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
