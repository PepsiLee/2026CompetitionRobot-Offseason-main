package frc.robot.config;

/** CAN IDs and independent voltage outputs for the three shooter motors. */
public record ShooterConfiguration(
    String canBus,
    int shootOneCanId,
    int shootTwoCanId,
    int shootUpCanId,
    double shootOneVolts,
    double shootTwoVolts,
    double shootUpVolts,
    double shootUpDelaySeconds,
    double supplyCurrentLimitAmps,
    double statorCurrentLimitAmps) {}
