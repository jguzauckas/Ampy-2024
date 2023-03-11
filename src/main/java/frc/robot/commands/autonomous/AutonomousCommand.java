
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

    public AutonomousCommand(DriveSubsystem drive) {
        addCommands(
            new AutoDriveCommand(2, 0, 0.5, drive) //Change speed and time
        );
    }
}
