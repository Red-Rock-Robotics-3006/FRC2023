package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;


/*TODO
 * Get max values of rotation and position for x and y
 * Get info from encoders
 * Make dynamic motor control
 * Get target from (limelight?)
 * Probably something else I'm forgetting
 */



public class ArmActuator extends SubsystemBase {

    public CANSparkMax yMotor = new CANSparkMax(61, CANSparkMaxLowLevel.MotorType.kBrushed);
    public CANSparkMax xMotor = new CANSparkMax(62, CANSparkMaxLowLevel.MotorType.kBrushed);


    public ArmActuator() {

        this.yMotor.restoreFactoryDefaults();
        this.yMotor.setInverted(false);
        this.yMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.xMotor.restoreFactoryDefaults();
        this.xMotor.setInverted(false);
        this.xMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }


    private Translation2d target;

    /* Replace with min and max values of the absolute encoders on x & y axis */
    private final double yRotMin = 0.0;
    private final double yRotMax = 360.0;
    private final double xRotMin = 0.0;
    private final double xRotMax = 360.0;

    /* Replace with min and max values of the x & y positions of the arm */
    private final double yMin = 0.0;
    private final double yMax = 100.0;
    private final double xMin = 0.0;
    private final double xMax = 100.0;

    /*Get from absolue encoders */
    double yRot = 0;
    double xRot = 0;

    /*Takes rotation from encoders and ranges of rotation and value and makes the position in the axis */
    double yPos = yMax * (yRot - yRotMin) / yRotMax + yMin;
    double xPos = xMax * (xRot - xRotMin) / xRotMax + xMin;

    Translation2d targetPos;
    double targetX;
    double targetY;


    public void setTargetPos(Translation2d pos) 
    {
        targetPos = pos;
        targetX = targetPos.getX();
        targetY = targetPos.getY();

        
        double xDiff = xPos - targetX;
        double yDiff = yPos - targetY;

        moveToPos(yDiff, xDiff);
        
    }
    public void setTargetPosSafeTravel(Translation2d pos, double arcRadius) {}
    public Translation2d getMeasuredPos() { return new Translation2d(); }

    /* Needs to be run repeatedly to make the arm slow to a stop */
    public void moveToPos(double vDiff, double hDiff){
        
        
        double yMotorPercent = (Math.cbrt(vDiff) / Math.cbrt(yMax - yMin));
        double xMotorPercent = (Math.cbrt(hDiff) / Math.cbrt(xMax - xMin));

        
        this.yMotor.set(yMotorPercent);
        this.xMotor.set(xMotorPercent);
        
    }

    public void periodic(){
        setTargetPos(target);
    }

}
