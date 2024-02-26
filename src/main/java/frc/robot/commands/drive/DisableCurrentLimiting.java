package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DisableCurrentLimiting extends Command {
    private DriveSubsystem drive;

    public DisableCurrentLimiting(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.setCurrentLimit(false);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setCurrentLimit(true);
    }

}
