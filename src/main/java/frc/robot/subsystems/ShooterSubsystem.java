package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;

public class ShooterSubsystem extends SubsystemBase {
    
    private SparkMax shooter;
    private RelativeEncoder relativeEncoder;
    private SparkClosedLoopController pidController;
    private SparkMaxConfig config = new SparkMaxConfig();

    public ShooterSubsystem() {
        shooter = new SparkMax(0, MotorType.kBrushless);

        config.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    
        relativeEncoder = shooter.getEncoder();
        
        pidController = shooter.getClosedLoopController();
        
        config.closedLoop.p(.03).i(0).d(0);
        
        pidController.setReference(0, ControlType.kVelocity);
    }

    public void setPower(double power) {
        shooter.set(power);
    }

    public void setVelocity(double velocity) {
        pidController.setReference(velocity, ControlType.kVelocity);
    }

    public Command getSetPowerCommand(double power) {
        return this.startEnd(() -> {
            shooter.set(power);
        }, () -> {
            setPower(0);
        });
    }

    public Command getSetPowerCommand(DoubleSupplier powSupplier) {
        return this.runEnd(() -> {
            shooter.set(powSupplier.getAsDouble());
        }, () -> {
            setPower(0);
        });
    }

    public Command getSetVelocityCommand(double velocity) {
        return this.runOnce(() -> {
            this.setVelocity(
                velocity
            );
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", relativeEncoder.getVelocity());
    }
}