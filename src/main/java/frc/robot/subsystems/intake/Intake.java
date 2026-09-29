package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Controls the intake motor. */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Intake(IntakeIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage voltage) -> runOpenLoop(voltage.in(edu.wpi.first.units.Units.Volts)),
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Runs the intake motor open loop at the supplied voltage. */
  public void runOpenLoop(double volts) {
    io.setVoltage(volts);
  }

  /** Runs the intake motor at the requested velocity. */
  public void runVelocity(double velocityRotPerSec) {
    io.setVelocity(velocityRotPerSec);
  }

  /** Stops the intake motor. */
  public void stop() {
    runOpenLoop(0.0);
  }

  /** Returns the motor velocity used by feedforward characterization. */
  @AutoLogOutput
  public double getCharacterizationVelocity() {
    return inputs.velocityRotPerSec;
  }

  /** Returns a command to run the intake SysId quasistatic test. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run the intake SysId dynamic test. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
