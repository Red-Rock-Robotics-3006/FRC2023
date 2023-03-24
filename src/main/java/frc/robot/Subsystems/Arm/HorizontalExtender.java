package frc.robot.Subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HorizontalExtender extends SubsystemBase {
    public CANSparkMax extensionMotor = new CANSparkMax(49, CANSparkMaxLowLevel.MotorType.kBrushed);
    
    public HorizontalExtender() {
        this.extensionMotor.restoreFactoryDefaults();
        this.extensionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public void setSpeed(double targetSpeed)
    {
        this.extensionMotor.set(targetSpeed);
    }
}
