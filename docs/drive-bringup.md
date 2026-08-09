# Drive bring-up

這個里程碑只建立 Drive 與 2910 式的責任分層：

- `RobotContainer`：建立物件、依 real/sim 選擇 IO、綁定控制器。
- `RobotState`：保存全機共用的 pose 與實測速度，其他 subsystem 不直接依賴 Drive 硬體。
- `RobotConfiguration` / `CompetitionRobotConfig`：集中 CAN ID、齒比、輪距、offset、inversion 與速度上限。
- `SuperStructure`：保留未來跨機構協調層；Drive 仍可獨立駕駛。
- `Drive`：處理駕駛輸入、座標轉換與 logging，不直接操作 TalonFX。
- `DriveIOReal`：CTRE TalonFX、CANcoder、Pigeon 2 與 CTRE odometry。
- `DriveIOSim`：與 2910 相同，內部依 `useMapleSim` 選擇 Maple-Sim 或 CTRE simulation。
- `SimulatedRobotState`：保存 Maple-Sim ground truth，並提供 Fuel 發射與場地模擬狀態。

## 控制器

- 左搖桿上下：field-relative 前後。
- 左搖桿左右：field-relative 左右平移。
- 右搖桿左右：旋轉；往左是逆時針正方向。
- Back：藍方設為 0 度，紅方設為 180 度，同時保留目前場上平移位置。

所有輸入都有 10% deadband 與平方曲線。紅方會自動翻轉平移方向。

## AdvantageScope

程式透過 AdvantageKit 的 NT4 publisher 發送資料。連線後可使用：

- `Drive/Pose`：拖進 2D Field，顯示 CTRE odometry 或 sim pose。
- `Drive/ModuleStates`：四個 module 的實際速度與角度。
- `Drive/ModuleTargets`：四個 module 的目標速度與角度。
- `Drive/GyroYaw`：Pigeon 或 sim yaw。
- `Drive/Viz/SimPose`：Maple-Sim ground-truth pose；場地碰撞顯示應使用這個值。
- `Drive/Viz/SimPose3d`：包含 BUMP 高度、pitch 與 roll 的 3D ground-truth pose。
- `Drive/Viz/IsOnBump`：自訂 BUMP 模型是否正在接管機器人的坡面運動。

## Maple-Sim 場地碰撞

`Constants.useMapleSim` 目前是 `true`。模擬啟動後會載入 Maple-Sim 0.4 beta 的
`Arena2026Rebuilt(false)`，將 bumper 外框加入 Dyn4j 物理世界，並保留場地邊界、Trench、Tower
及 HUB 碰撞。內建的 BUMP 實心矩形 collider 已關閉，改由 MIT 授權的 `RobotBumpSim` 追蹤四個
module 接觸點，模擬高度、pitch、roll、過坡速度以及速度不足時滑回。這是疊加在 Maple-Sim
2D 物理上的坡面近似模型，不是完整的 3D rigid-body solver。

## 第一次實機檢查

目前硬體常數以 2910 公開 ReBlitz 設定為起點。Team 11855 的實機在 enable 前必須逐項核對
`CompetitionRobotConfig`。

1. 架高車體並移除機構干涉；先確認 CANivore、Pigeon、8 顆 TalonFX、4 顆 CANcoder 都在線。
2. 依序核對 module 順序：front-left、front-right、back-left、back-right。
3. 手動讓每個 module 逆時針轉動；steer motor sensor 與 CANcoder 都必須增加。
4. 將四輪指向車頭；`Drive/ModuleStates` 的角度都應接近 0 度。若不符，只修改該輪 encoder offset。
5. 小幅向前命令；四輪速度都應為正且機器人朝 +X。若單輪相反，只修改該輪 drive inversion。
6. 小幅向左平移；四輪應接近 +90 度。
7. 小幅逆時針旋轉；Pigeon yaw 應增加，AdvantageScope 上車頭也應逆時針轉動。
8. 推行已停用的機器人並觀察 `Drive/Pose`；向前、向左、逆時針必須分別增加 X、Y、yaw。

模擬測試已固定上述 WPILib 座標規則；實機檢查負責確認 wiring、offset 與 inversion 和設定一致。
