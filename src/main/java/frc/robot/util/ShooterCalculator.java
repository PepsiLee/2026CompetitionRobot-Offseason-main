package frc.robot.util;

public class ShooterCalculator {
    // All Distance is meter
    private static final double TARGET_HEIGHT = 1.8288;
    private static final double ANGLE_DEG = 75.0;
    private static final double GRAVITY = 9.81;
    private static final double SHOOTER_HEIGHT = 0.47;
    private static final double WHEEL_DIAMETER = 0.1;
    private static final boolean IS_HOODED_SHOOTER = true;

    /**
     * 計算射球所需的 RPM
     * 
     * @param distanceMeters 機器人到目標的水平距離 (單位: 公尺)
     * @return 目標輪子轉速 (RPM)。若物理上無法達到目標高度，則回傳 -1
     */
    public static double calculateRPM(double distanceMeters) {
        // 1. 物理計算準備
        double angleRad = Math.toRadians(ANGLE_DEG);
        double yDisplacement = TARGET_HEIGHT - SHOOTER_HEIGHT;

        // 2. 檢查是否超出物理極限 (仰角下距離太遠，導致球達不到目標高度)
        double maxHeightTerm = distanceMeters * Math.tan(angleRad);
        if (maxHeightTerm <= yDisplacement) {
            return 3000; 
        }

        // 3. 計算球的初速度 (v) -> 單位: m/s
        double numerator = GRAVITY * Math.pow(distanceMeters, 2);
        double denominator = 2 * Math.pow(Math.cos(angleRad), 2) * (maxHeightTerm - yDisplacement);
        double ballVelocity = Math.sqrt(numerator / denominator);

        // 4. 計算輪子邊緣速度 (單輪機構需要 2 倍速度)
        double wheelSurfaceVelocity = IS_HOODED_SHOOTER ? (ballVelocity * 2) : ballVelocity;

        // 5. 轉換為 RPM
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;
        double rpm = (wheelSurfaceVelocity * 60) / wheelCircumference;

        // 加上一個摩擦力與動能損耗的補償係數 (視你們的機器狀況微調，預設為 1.0 不變)
        double tuningFactor = 1.1;
        
        // return ballVelocity * 1.07;

        return rpm * tuningFactor;
    }
}