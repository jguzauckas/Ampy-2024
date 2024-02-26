package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends Command{
    DriveSubsystem drive;
    Timer elapsedTime;
    double seconds;
    double speed;
    double curvature;
    public AutoDriveCommand(double speed, double curvature, DriveSubsystem drive) {
        addRequirements(drive);
        this.drive = drive;
        this.speed = speed;
        this.curvature = curvature;
    }
    
    @Override
    public void execute() {
        drive.drive(speed, curvature);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

}
