package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public CANSparkMax elevatorLeft = new CANSparkMax(57, CANSparkMaxLowLevel.MotorType.kBrushless);
  public CANSparkMax elevatorRight = new CANSparkMax(59, CANSparkMaxLowLevel.MotorType.kBrushless);

  public Elevator() {
    this.elevatorLeft.restoreFactoryDefaults();
    this.elevatorRight.restoreFactoryDefaults();
    this.elevatorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.elevatorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }

  public void setSpeed(double targetSpeed)
  {
    this.elevatorLeft.set(targetSpeed);
    this.elevatorRight.set(targetSpeed);
  }
}