package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    public CANSparkMax liftOne = new CANSparkMax(57, CANSparkMaxLowLevel.MotorType.kBrushless);
    public CANSparkMax liftTwo = new CANSparkMax(59, CANSparkMaxLowLevel.MotorType.kBrushless);
    public Elevator()
    {
        this.liftOne.restoreFactoryDefaults();
        this.liftOne.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.liftTwo.restoreFactoryDefaults();
        this.liftTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {
      
    }
  
    @Override
    public void simulationPeriodic() {
      
    }

    public void setMotorSpeed(float targetSpeed){
      this.liftOne.set(targetSpeed);
      this.liftTwo.set(targetSpeed);
    }
}
