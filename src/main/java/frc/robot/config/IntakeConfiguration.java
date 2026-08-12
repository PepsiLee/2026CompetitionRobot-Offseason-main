package frc.robot.config;

/** Hardware and voltage outputs for the two gravity-feed intake motors. */
public record IntakeConfiguration(
        String canBus,
        int alwaysOnMotorCanId,
        int pivotRMotorCanId,
        int pivotLMotorCanId,
        double alwaysOnVolts,
        double circleMotorVolts,
        double supplyCurrentLimitAmps,
        double statorCurrentLimitAmps) {
}
