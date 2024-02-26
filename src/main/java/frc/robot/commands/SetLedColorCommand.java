package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class SetLedColorCommand extends Command {
    LEDSubsystem led;
    private int red;
    private int green;
    private int blue;

    public SetLedColorCommand(LEDSubsystem led, int red, int green, int blue) {
        this.led = led;
        this.red = red;
        this.green = green;
        this.blue = blue;

        addRequirements(led);
    }

    @Override
    public void execute() {
        led.setLedColor(red, green, blue);
    }

    public void end() {
        led.setLedColor(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
