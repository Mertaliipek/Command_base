package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Enco_gyroCMD extends CommandBase {
    private final double kP;
    private final DriveTrain driveTrain;

    public Enco_gyroCMD(double kP,DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.kP = kP;
        addRequirements(driveTrain);
    }  
    @Override
    public void initialize() {
      System.out.print("Encoder and Gyro started");
      
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      driveTrain.Encoders(kP);
      driveTrain.Gyro();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      System.out.println("Encoder and Gyro Ended");
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;

    }
}
