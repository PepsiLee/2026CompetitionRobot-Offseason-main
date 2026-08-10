package frc.robot.subsystems.feeder;

public interface FeederIO {
    class Inputs {
        public boolean connected;
        public double appliedVolts;
        public double supplyCurrentAmps;
        public double statorCurrentAmps;
        public double temperatureCelsius;
    }

    default void updateInputs(Inputs inputs) {
    }

    default void setVoltage(double volts) {
    }

    default void set(double rpm) {
    }

    default void stop() {
        setVoltage(0.0);
    }
}