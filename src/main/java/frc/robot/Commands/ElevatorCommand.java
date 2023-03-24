package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm.Elevator;

public class ElevatorCommand extends CommandBase{
    private Elevator elevator = new Elevator();
    private double power;

    public ElevatorCommand(double targetSpeed){this.power = targetSpeed;}

    @Override
    public void initialize() {this.elevator.setSpeed(this.power);}

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){this.elevator.setSpeed(0.0);}

    @Override
    public boolean isFinished(){return false;}

}
