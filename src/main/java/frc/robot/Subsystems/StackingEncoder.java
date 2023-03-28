package frc.robot.Subsystems;
// CANCoder
import java.lang.Math;
import com.ctre.phoenix.sensors.CANCoder;

public class StackingEncoder {

    private final CANCoder m_cCoder = new CANCoder(0);

    private double currentPos = m_cCoder.getAbsolutePosition();
    private double pastPos;
    private double rotations = 0.0;
    private double actualPos;

    public double getActualCANCoder(){
        return actualPos;
    }

    public void periodic() {
        pastPos = currentPos;
        currentPos = m_cCoder.getAbsolutePosition();
        actualPos = currentPos + (rotations * 360.0);

        if(Math.abs(currentPos - pastPos)>180){
            double currentSign = Math.signum(pastPos - currentPos);
            rotations += currentSign;
        }
    }
}