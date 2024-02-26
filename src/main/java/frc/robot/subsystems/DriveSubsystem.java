package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units.GearRatio;
import frc.robot.Units.Meters;
import frc.robot.Units.Radians;
import frc.robot.Units.Rotations;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
    // CAN Bus Information
    private static final String CAN_BUS_NAME = "rio";
    private static final int LEFT_MAIN_TALONFX = 1;
    private static final int LEFT_FOLLOWER_TALONFX = 2;
    private static final int RIGHT_MAIN_TALONFX = 3;
    private static final int RIGHT_FOLLOWER_TALONFX = 4;

    // 4 Motor Controllers for Drivetrain
    private final TalonFX leftMain = new TalonFX(LEFT_MAIN_TALONFX, CAN_BUS_NAME);
    private final TalonFX leftFollower = new TalonFX(LEFT_FOLLOWER_TALONFX, CAN_BUS_NAME);
    private final TalonFX rightMain = new TalonFX(RIGHT_MAIN_TALONFX, CAN_BUS_NAME);
    private final TalonFX rightFollower = new TalonFX(RIGHT_FOLLOWER_TALONFX, CAN_BUS_NAME);

    private final boolean isCoastMode = false;

    private final GearRatio gearRatio = new GearRatio(8.45d);
    private final Meters wheelDiameter = new Meters(0.1524);
    private final Radians wheelCircumference = new Radians(Math.PI * wheelDiameter.asDouble());

    // Differential Drive object to build Drivetrain with
    private DifferentialDrive drive;

    public DriveSubsystem() {
        initializeTalonFX(leftMain.getConfigurator(), "left");
        initializeTalonFX(leftFollower.getConfigurator(), "left");
        initializeTalonFX(rightMain.getConfigurator(), "right");
        initializeTalonFX(rightFollower.getConfigurator(), "right");

        leftFollower.setControl(new Follower(leftMain.getDeviceID(), false));
        rightFollower.setControl(new Follower(rightMain.getDeviceID(), false));

        drive = new DifferentialDrive(leftMain, rightMain);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Position/Left Main (m)", -rotationsToMetersAsDouble(new Rotations(leftFollower.getPosition().getValueAsDouble())));
        SmartDashboard.putNumber("Position/Left Follower (m)", -rotationsToMetersAsDouble(new Rotations(leftFollower.getPosition().getValueAsDouble())));
        SmartDashboard.putNumber("Position/Right Main (m)", -rotationsToMetersAsDouble(new Rotations(rightMain.getPosition().getValueAsDouble())));
        SmartDashboard.putNumber("Position/Right Follower (m)", -rotationsToMetersAsDouble(new Rotations(rightFollower.getPosition().getValueAsDouble())));

        SmartDashboard.putNumber("Velocity/Left Main (m-s)", -rotationsToMetersAsDouble(new Rotations(leftMain.getVelocity().getValueAsDouble())));
        SmartDashboard.putNumber("Velocity/Left Follower (m-s)", -rotationsToMetersAsDouble(new Rotations(leftFollower.getVelocity().getValueAsDouble())));
        SmartDashboard.putNumber("Velocity/Right Main (m-s)", -rotationsToMetersAsDouble(new Rotations(rightMain.getVelocity().getValueAsDouble())));
        SmartDashboard.putNumber("Velocity/Right Follower (m-s)", -rotationsToMetersAsDouble(new Rotations(rightFollower.getVelocity().getValueAsDouble())));
        
        SmartDashboard.putNumber("Acceleration/Left Main (m-s-s)", -rotationsToMetersAsDouble(new Rotations(leftMain.getAcceleration().getValueAsDouble())));
        SmartDashboard.putNumber("Acceleration/Left Follower (m-s-s)", -rotationsToMetersAsDouble(new Rotations(leftFollower.getAcceleration().getValueAsDouble())));
        SmartDashboard.putNumber("Acceleration/Right Main (m-s-s)", -rotationsToMetersAsDouble(new Rotations(rightMain.getAcceleration().getValueAsDouble())));
        SmartDashboard.putNumber("Acceleration/Right Follower (m-s-s)", -rotationsToMetersAsDouble(new Rotations(rightFollower.getAcceleration().getValueAsDouble())));

        SmartDashboard.putNumber("MotorTemperature/Left Main (C)", Math.round(leftMain.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Left Follower (C)", Math.round(leftFollower.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Right Main (C)", Math.round(rightMain.getDeviceTemp().getValueAsDouble()));
        SmartDashboard.putNumber("MotorTemperature/Right Follower (C)", Math.round(rightFollower.getDeviceTemp().getValueAsDouble()));

        SmartDashboard.putNumber("MotorCurrent/Left Main", leftMain.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Left Follower", leftFollower.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Right Main", rightMain.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("MotorCurrent/Right Follower", rightFollower.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Coast Mode Enabled", isCoastMode);
    }

    private void initializeTalonFX(TalonFXConfigurator cfg, String side) {
        TalonFXConfiguration toApply = new TalonFXConfiguration();

        if (side.equals("left")) {
            toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        
        if (isCoastMode) {
            toApply.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        } else {
            toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
        toApply.CurrentLimits.StatorCurrentLimitEnable = true;
        toApply.CurrentLimits.StatorCurrentLimit = 90;
        cfg.apply(toApply);
        cfg.setPosition(0);
    }

    public void currentLimitToggleTalonFX(TalonFXConfigurator cfg, boolean shouldCurrentLimitEnable) {
        TalonFXConfiguration toApply = new TalonFXConfiguration();
        toApply.CurrentLimits.StatorCurrentLimitEnable = shouldCurrentLimitEnable;
        cfg.apply(toApply);
    }

    public void drive(double speed, double turn) {
        drive.curvatureDrive(speed, turn, true);
    }
    
    private double rotationsToMetersAsDouble(Rotations rotations) {
        return
            rotations.asMeters(gearRatio, wheelCircumference)
                     .asDouble();
    }

    public void setCurrentLimit(boolean shouldCurrentLimitEnable) {
        currentLimitToggleTalonFX(leftMain.getConfigurator(), shouldCurrentLimitEnable);
        currentLimitToggleTalonFX(leftFollower.getConfigurator(), shouldCurrentLimitEnable);
        currentLimitToggleTalonFX(rightMain.getConfigurator(), shouldCurrentLimitEnable);
        currentLimitToggleTalonFX(rightFollower.getConfigurator(), shouldCurrentLimitEnable);
    }
    
    public void stop() {
        leftMain.set(0);
        rightMain.set(0);
    }
}                                                 