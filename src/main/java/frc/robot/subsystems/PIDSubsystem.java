package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDSubsystem extends SubsystemBase{
    private final Victor victorLeft1 = new Victor(0);
  private final Victor victorLeft2 = new Victor(1);
  private final Victor victorRight1 = new Victor(2);
  private final Victor victorRight2 = new Victor(3);
    private final Encoder encoder = new Encoder(0,1);
    private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;
    private final MotorControllerGroup left_m = new MotorControllerGroup(victorLeft1, victorLeft2);
    private final MotorControllerGroup right_m = new MotorControllerGroup(victorRight1, victorRight2);
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
  
    double sensorPosition = encoder.get() * kDriveTick2Feet;
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoder.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    }
    public void ErrorSum (double iLimit) {
        if (Math.abs(error) < iLimit) {
          errorSum += error * dt;
        }
      }
      double errorRate = (error - lastError) / dt;
          
      public void OutputSpeed (double kP,double kI,double kD) {
        double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
        left_m.set(outputSpeed);
        right_m.set(-outputSpeed);
      }
      public void setPoint(int setPoint) {
        setpoint = setPoint; 
      }











      
      @Override
      public void simulationPeriodic() {
        // debug
        SmartDashboard.putNumber("Encoder Value",encoder.get());
    
        // This method will be called once per scheduler run during simulation   
      }
}
