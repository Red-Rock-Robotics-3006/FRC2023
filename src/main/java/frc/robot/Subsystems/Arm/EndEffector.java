package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class EndEffector extends SubsystemBase {

    Solenoid coneFlipSolenoid = new Solenoid(33,PneumaticsModuleType.CTREPCM, 1);

    int intakem = 61;
    int wristm = 62;
    CANSparkMax intakemotor = new CANSparkMax(intakem, CANSparkMaxLowLevel.MotorType.kBrushless); //random int deviceId
    CANSparkMax wristmotor = new CANSparkMax(wristm, CANSparkMaxLowLevel.MotorType.kBrushless); //random int deviceId
    private final CANCoder k_cCoder;
    private double targetAngle;

    public EndEffector(int cCoderChannel){
        this.intakemotor.restoreFactoryDefaults();
        this.intakemotor.setInverted(false);
        this.intakemotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.wristmotor.restoreFactoryDefaults();
        this.wristmotor.setInverted(false);
        this.wristmotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.k_cCoder = new CANCoder(cCoderChannel);

    }

    @Override
    public void periodic(){
        double motorpower = 0;
        double angleDifference = this.getCurrentAngle() - targetAngle;
        double kP = 0;

        //add PID loops to slow down arm as this.getCurrentAngle() nears targetAngle
        
        if (this.getCurrentAngle() != targetAngle) {
            motorpower = angleDifference * kP;
            wristmotor.set(motorpower);
        }
        else { //(this.getCurrentAngle() == targetAngle)
            wristmotor.set(0);
        }
    }
    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }
    public void intake(Mode mode) {
        if(mode.equals(Mode.CUBE)) {
            
        }

        else if(mode.equals(Mode.UPRIGHT_CONE)) {
            coneFlipSolenoid.set(true);
        }

        else if(mode.equals(Mode.TIPPED_CONE)) {
            coneFlipSolenoid.set(true);
        }
    }
    public void expel(Mode mode) {
        if(mode.equals(Mode.CUBE)) {
            
        }

        else if(mode.equals(Mode.UPRIGHT_CONE)) {
            coneFlipSolenoid.set(true);
        }

        else if(mode.equals(Mode.TIPPED_CONE)) {
            coneFlipSolenoid.set(true);
        }
    }
    public Translation2d getTargetCenterOffset() { return new Translation2d(); }
    public double getMeasuredAngle() { return 0; }

    public void solenoidToggle(boolean state) {
        coneFlipSolenoid.set(state);
    }

    public double getCurrentAngle() {
        return k_cCoder.getAbsolutePosition();
    }
    
    public enum Mode {
        CUBE,
        UPRIGHT_CONE,
        TIPPED_CONE
    }
}
