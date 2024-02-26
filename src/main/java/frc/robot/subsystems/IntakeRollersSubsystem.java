
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeRollersSubsystem extends SubsystemBase {

    private CANSparkMax intakeRollerMotor;
    public static final int CAN_ID = 6; // CAN ID should be 6, it may be different as a result of testing

    public IntakeRollersSubsystem() {
        intakeRollerMotor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
        intakeRollerMotor.restoreFactoryDefaults();
        intakeRollerMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Set the intake roller's speed
     * @param speed The speed in [-1.0, 1.0]
     */
    public void setSpeed(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void stop() {
        intakeRollerMotor.stopMotor();
    }

}
