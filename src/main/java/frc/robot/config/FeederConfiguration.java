package frc.robot.config;

/**
 * Feeder 子系統的硬體與控制參數設定檔。
 *
 * @param canBus             驅動馬達所使用的 CAN 總線名稱 (例如 "rio" 或 "canivore")
 * @param feederMotorCanId   Feeder 馬達的 CAN ID
 * @param intakeVolts        正常進彈時的目標電壓 (單位: 伏特)
 * @param ejectVolts         反轉排彈時的目標電壓 (單位: 伏特)
 * @param feedToShooterVolts 射擊時推彈至 Shooter 的目標電壓 (單位: 伏特)
 * @param currentLimitAmps   防卡彈與馬達保護的電流上限 (單位: 安培)
 */
public record FeederConfiguration(
        String canBus,
        int feederMotorCanId,
        double intakeVolts,
        double ejectVolts,
        double feedToShooterVolts,
        double currentLimitAmps) {
}