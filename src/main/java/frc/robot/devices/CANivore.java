package frc.robot.devices;

public class CANivore {
    private final String CANIVORE_BUS_NAME = "Swerve";  // Name of the CANivore bus
    
    public CANivore() {}
    
    public String getBusName() {
        return CANIVORE_BUS_NAME;
    }
}