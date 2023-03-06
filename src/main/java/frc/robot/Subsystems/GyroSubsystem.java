package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

public class GyroSubsystem {
    private static Pigeon2 instance;

    public static Pigeon2 getPigeonInstance() {
        if(instance==null) instance = new Pigeon2(45);
        return instance;
    }
}
