package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

/** Factory methods for intake commands. */
public final class IntakeCommands {
  private static final double FF_RAMP_RATE_VOLTS_PER_SEC = 0.1;

  private IntakeCommands() {}

  /** Measures the intake motor's static and velocity feedforward constants. */
  public static Command feedforwardCharacterization(Intake intake) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE_VOLTS_PER_SEC;
                  intake.runOpenLoop(voltage);
                  velocitySamples.add(intake.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                intake)
            .finallyDo(
                () -> {
                  intake.stop();
                  if (velocitySamples.size() < 2) {
                    return;
                  }

                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    double velocity = velocitySamples.get(i);
                    double voltage = voltageSamples.get(i);
                    sumX += velocity;
                    sumY += voltage;
                    sumXY += velocity * voltage;
                    sumX2 += velocity * velocity;
                  }

                  double denominator = n * sumX2 - sumX * sumX;
                  if (Math.abs(denominator) < 1e-9) {
                    System.out.println(
                        "Intake FF characterization failed: insufficient velocity change.");
                    return;
                  }

                  double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
                  double kV = (n * sumXY - sumX * sumY) / denominator;
                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Intake FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
