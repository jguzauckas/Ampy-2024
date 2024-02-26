package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * Enable the conveyor within Ampy to rotate forward (in the direction into the intake)
 */
public class MoveConveyorCommand extends Command {
    ConveyorSubsystem conveyor;
    private final double targetSpeed;

    public MoveConveyorCommand(double targetSpeed, ConveyorSubsystem conveyor) {
        // Positive values move pieces towards the back
        this.targetSpeed = -targetSpeed;
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setSpeed(targetSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
    }

}
