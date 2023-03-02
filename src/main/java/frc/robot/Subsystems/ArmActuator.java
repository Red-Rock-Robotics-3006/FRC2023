package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*TODO
 * Get max values of rotation and position for x and y
 * Get the motor control function
 * Fix the dynamic motor control while loop
 * Probably something else I'm forgetting
 */



public class ArmActuator extends SubsystemBase {

    /*Whatever the max motor speed is, used for telling the motor how fast to rotate */
    double maxMotorSpeed;



    /* Replace with min and max values of the absolute encoders on x & y axis */
    double yRotMin = 0.0;
    double yRotMax = 360.0;
    double xRotMin = 0.0;
    double xRotMax = 360.0;

    /* Replace with min and max values of the x & y positions of the arm 
     * DO NOT CHANGE MIN VALUES FROM 0 IT WILL PROBABLY BREAK SOMETHING
    */
    double yMin = 0.0;
    double yMax = 100.0;
    double xMin = 0.0;
    double xMax = 100.0;

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

        
        /* Fix while loop to stop once the arm is in position */
        while(xDiff!=0||yDiff!=0){
            double xDiff = xPos - targetX;
            double yDiff = yPos - targetY;

            setMotorSpeed(yDiff, xDiff);
        }
    }
    public void setTargetPosSafeTravel(Translation2d pos, double arcRadius) {}
    public Translation2d getMeasuredPos() { return new Translation2d(); }


    public double setMotorSpeed(double vDiff, double hDiff){
        
        
        double yMotorPercent = (Math.pow(vDiff, 0.333) / Math.pow(yMax, 0.333));
        double xMotorPercent = (Math.pow(hDiff, 0.333) / Math.pow(xMax, 0.333));

        
        moveYMotor(yMotorPercent);
        moveXMotor(xMotorPercent);
        
    }
}
