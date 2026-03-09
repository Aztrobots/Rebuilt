# TASK — Publish NetworkTables keys for the dashboard

## Context
The React/FWC dashboard expects these SmartDashboard keys from the robot.
Add them to the appropriate subsystems.

## Required changes

### 1. ShooterSubsystem.java
Add `periodic()` publishing:
- `SmartDashboard.putNumber("ShooterRPM", lsMotor.getVelocity().getValueAsDouble() * 60)`
- `SmartDashboard.putNumber("TargetRPM", targetRPM)` — expose existing internal variable

### 2. IntakeSubsystem.java
Add `periodic()` publishing:
- `SmartDashboard.putNumber("Intake/EncoderPos", getPosition())`

### 3. ShooterSubsystem.java — read ShooterTable from NT
In the same `periodic()`, read 5 rows from the dashboard editable table
and update the `InterpolatingDoubleTreeMap itm`:
```java
for (int i = 1; i <= 5; i++) {
    double dist = SmartDashboard.getNumber("ShooterTable/Dist_" + i, i * 1.0);
    double rpm  = SmartDashboard.getNumber("ShooterTable/RPM_"  + i, i * 500.0);
    itm.put(dist, rpm);
}
```

### 4. Verify imports
Ensure `edu.wpi.first.wpilibj.smartdashboard.SmartDashboard`
is imported in both files.

## Do NOT touch
- TunerConstants.java
- RobotContainer.java
- Ports.java
- Any generated swerve file