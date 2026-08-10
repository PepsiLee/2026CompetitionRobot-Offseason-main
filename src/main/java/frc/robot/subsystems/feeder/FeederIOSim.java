package frc.robot.subsystems.feeder;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {

    private static final double LOOP_PERIOD_SECONDS = 0.02;

    private static final double GEAR_RATIO = 1.0;

    // Feeder rotor inertia
    private static final double MOMENT_OF_INERTIA = 0.001;

    private final DCMotor motor = DCMotor.getKrakenX60(1);

    private final DCMotorSim motorSim =
            new DCMotorSim(
                    createDCMotorSystem(
                            motor,
                            GEAR_RATIO,
                            MOMENT_OF_INERTIA),
                    motor);

    /*
     * Simulated equivalent of the TalonFX VelocityVoltage controller.
     */
    private final PIDController velocityPID =
            new PIDController(
                    1.0,    // kP
                    0.0,    // kI
                    0.0);   // kD

    private double targetRPM = 0.0;
    private double targetVoltage = 0.0;

    public FeederIOSim() {
        velocityPID.setTolerance(50.0);
    }

    @Override
    public void updateInputs(Inputs inputs) {

        /*
         * Current motor speed
         */
        double currentRPM =
                motorSim.getAngularVelocityRPM();

        /*
         * Velocity control
         */
        if (Math.abs(targetRPM) > 0.001) {

            double feedbackVoltage =
                    velocityPID.calculate(
                            currentRPM,
                            targetRPM);

            /*
             * Feedforward:
             *
             * 12 V at Kraken free speed.
             */
            double feedforwardVoltage =
                    12.0
                    * targetRPM
                    / motor.freeSpeedRadPerSec*(60.0 / (2.0 * Math.PI));

            targetVoltage =
                    feedbackVoltage
                    + feedforwardVoltage;

        } else {
            targetVoltage = 0.0;
            velocityPID.reset();
        }

        /*
         * Clamp to battery voltage.
         */
        targetVoltage =
                Math.max(
                        -12.0,
                        Math.min(12.0, targetVoltage));

        /*
         * Apply voltage to simulated motor.
         */
        motorSim.setInputVoltage(targetVoltage);

        /*
         * Advance simulation by 20 ms.
         */
        motorSim.update(LOOP_PERIOD_SECONDS);

        /*
         * Update IO inputs.
         */
        inputs.connected = true;

        inputs.appliedVolts =
                targetVoltage;

        inputs.supplyCurrentAmps =
                motorSim.getCurrentDrawAmps();

        inputs.statorCurrentAmps =
                motorSim.getCurrentDrawAmps();

        inputs.temperatureCelsius =
                25.0;
    }

    @Override
    public void setVoltage(double volts) {

        /*
         * Manual voltage mode.
         */
        targetRPM = 0.0;

        targetVoltage =
                Math.max(
                        -12.0,
                        Math.min(12.0, volts));

        velocityPID.reset();
    }

    @Override
    public void set(double rpm) {

        /*
         * Velocity mode.
         */
        targetRPM = rpm;
    }

    @Override
    public void stop() {

        targetRPM = 0.0;
        targetVoltage = 0.0;

        velocityPID.reset();

        motorSim.setInputVoltage(0.0);
    }
}