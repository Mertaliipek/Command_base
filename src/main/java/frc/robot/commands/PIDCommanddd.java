package frc.robot.commands;

import frc.robot.subsystems.PIDSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class PIDCommanddd extends CommandBase{
    private final PIDSubsystem pidsubsytem;
    private final double iLimit;
    private final double kP;
    private final double kI;
    private final double kD;
    private final int setpoint;

    public PIDCommanddd(PIDSubsystem pidsubsytem,double iLimit,double kP,double kI,double kD,int setpoint) {
        this.pidsubsytem = pidsubsytem;
        this.iLimit = iLimit;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setpoint = setpoint;
    }


    // Called when the command is initially scheduled.
@Override
public void initialize() {
    System.out.println("PID Started");
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    pidsubsytem.ErrorSum(iLimit);
    pidsubsytem.OutputSpeed(kP, kI, kD);
    pidsubsytem.setPoint(setpoint);
    

}


// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    pidsubsytem.ErrorSum(0);
    pidsubsytem.OutputSpeed(0, 0, 0);
    pidsubsytem.setPoint(0);
    System.out.println("PID ended");
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return false;
}
    

}
