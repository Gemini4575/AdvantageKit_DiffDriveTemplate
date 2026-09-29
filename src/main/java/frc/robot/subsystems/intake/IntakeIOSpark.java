package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax intakeMotor =
      new SparkMax(kIntakeMotorCANID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(0.0, 0.0); // TODO: Set the correct feedforward gains

  public IntakeIOSpark() {
    // Create config
    var config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) currentLimit)
        .voltageCompensation(12.0);
    config.closedLoop.pid(realKp, 0.0, realKd);
    config
        .encoder
        .positionConversionFactor(2 * Math.PI / motorReduction) // Rotor Rotations -> Wheel Radians
        .velocityConversionFactor(
            (2 * Math.PI) / 60.0 / motorReduction) // Rotor RPM -> Wheel Rad/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    // Apply config to leaders
    config.inverted(Inverted);
    tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    ifOk(intakeMotor, intakeEncoder::getPosition, (value) -> inputs.positionRad = value);
    ifOk(intakeMotor, intakeEncoder::getVelocity, (value) -> inputs.velocityRotPerSec = value);
    ifOk(intakeMotor, intakeMotor::getAppliedOutput, (value) -> inputs.appliedVolts = value);
    ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRotPerSec) {
    intakeMotor.setVoltage(feedforward.calculate(velocityRotPerSec));
  }
}
