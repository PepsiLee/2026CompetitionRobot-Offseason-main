package frc.robot.config;

/** CAN IDs and independent voltage outputs for the three shooter motors. */
public record ShooterConfiguration(
        String canBus,
        int shootOneCanId,
        int shootTwoCanId,
        double shootOneVolts,
        double shootTwoVolts,
        double supplyCurrentLimitAmps,
        double statorCurrentLimitAmps,
        double gearRatio) {
}
