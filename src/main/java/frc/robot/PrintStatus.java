package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PrintStatus extends InstantCommand {

    public PrintStatus() {
        // No requirements needed as this command does not interact with subsystems
    }

    @Override
    public void initialize() {
        // Print a message to the Driver Station console
            DriverStation.reportWarning("it didnt work", false);

    }
}