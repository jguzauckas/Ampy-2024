package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {

    private CANSparkMax conveyorMotor;
    private static final int CONVEYOR_MOTOR_CAN_ID = 5; // CAN ID should be 5, it may be different as a result of testing
    private static final int INTAKE_SENSOR_CHANNEL = 0;
    private final DigitalInput intakeSensor;
    private static final int CONVEYOR_SENSOR_CHANNEL = 2;
    private final DigitalInput conveyorSensor;

    public ConveyorSubsystem() {
        intakeSensor = new DigitalInput(INTAKE_SENSOR_CHANNEL);
        conveyorSensor = new DigitalInput(CONVEYOR_SENSOR_CHANNEL);
        conveyorMotor = new CANSparkMax(CONVEYOR_MOTOR_CAN_ID, MotorType.kBrushless);
        conveyorMotor.restoreFactoryDefaults();
        initializeSmartDashboard();
    }

    private void initializeSmartDashboard() {
        SmartDashboard.putBoolean("isConveyorEmpty", isIntakeSensor());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isConveyorEmpty", isIntakeSensor());
    }

    /**
     * Set the conveyor's speed
     * @param speed the speed in [-1.0, 1.0]
     */
    public void setSpeed(double speed) {
        conveyorMotor.set(speed);
    }

    public void stop() {
        conveyorMotor.stopMotor();
    }

    /**
     * Get the intake sensor status
     * @return whether something is in the intake
     */
    public boolean isIntakeSensor() {
        return intakeSensor.get();
    }

    /**
     * Get the conveyor sensor status
     * @return whether something is on the conveyor
     */
    public boolean isConveyorSensor() {
        return conveyorSensor.get();
    }
}
