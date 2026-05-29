# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

FRC (FIRST Robotics Competition) robot code for **Team 10333**, 2026 season. Java 17, command-based WPILib, built with GradleRIO 2026.2.1. The robot is a CTRE swerve-drive shooter robot (intake → hopper → indexer → kicker → shooter) with multi-camera PhotonVision pose estimation and PathPlanner autonomous.

## Commands

All commands use the Gradle wrapper from the repo root. There is no separate lint step; the Java compiler (with `-XDstringConcat=inline`) is the gate.

```bash
./gradlew build                 # Compile + run tests
./gradlew test                  # JUnit 5 tests only
./gradlew test --tests "ClassName"   # Run a single test class
./gradlew simulateJava          # Run in simulation (WPILib sim GUI + DriverStation auto-enabled)
./gradlew deploy                # Build and deploy to the roboRIO (team number from .wpilib/wpilib_preferences.json)
./gradlew riolog                # Stream console/riolog output from the robot
```

Deploy/simulate are the primary workflows — most "running" happens on the physical robot or in the WPILib simulator, not via unit tests.

## Architecture

Standard command-based layout under `src/main/java/frc/robot/`:

- **`Main.java`** — entry point; do not edit (boilerplate).
- **`Robot.java`** — extends `LoggedRobot` (AdvantageKit). `robotInit` starts the AdvantageKit `Logger` (WPILOG on the real robot, NT4 publisher always) and wires up `FuelSim`. Notably owns its own `Lights` instance and the auto/teleop **vision-fusion gating**: vision pose fusion is disabled at the start of autonomous to avoid pose "jumps," then re-enabled ~0.75s later; teleop seeds the drivetrain pose from the latest vision estimate.
- **`RobotContainer.java`** — the wiring hub. Instantiates all subsystems, registers PathPlanner `NamedCommands` (e.g. `shoot`, `Index`, `kick`, `intakeSpin`, `engageIntake`), configures `AutoBuilder` (`configureAutoBuilder()` with `PPHolonomicDriveController`), builds the `autoChooser` SendableChooser, and binds controllers in `configureBindings()`. Two controllers: **Driver** (port 0) and **Manipulator** (port 1). Much of `configureBindings()` is commented-out experimental bindings — active bindings are interleaved, so read carefully before assuming a button is free.
- **`Constants.java`** — robot-wide constants. Holds the four camera `Transform3d` mounting offsets and the vision standard-deviation matrices (`kSingleTagStdDevs` / `kMultiTagStdDevs`; theta std-dev is intentionally huge so heading is trusted only from the gyro, not vision).
- **`Telemetry.java`** — CTRE swerve telemetry publisher, registered via `drivetrain.registerTelemetry(...)`.

### Key directories

- **`subsystems/`** — `CommandSwerveDrivetrain` (CTRE-generated swerve base), `Vision`, `Shooter`, `Kicker`, `Indexer`, `Hopper`, `Intake`, `Climber`, `Lights`, plus `LockOnHub` and `ShooterMath` (shot math/lookup). `ExampleSubsystem`, `Deprecated`, and `lyle` are dead/scratch code — do not extend them.
- **`commands/`** — one command per file (`ShootBall`, `ShootSequence`, `Index`, `Kick`, `Hopp`, `PointAndRotate`, `runIntake`, `EngageIntake`, `MoveClimber`, etc.). `Autos.java` and several `*Climber` commands exist; many are referenced only from commented bindings.
- **`generated/`** — `TunerConstants.java` is produced by the CTRE Tuner X swerve generator. **Regenerate it from Tuner X rather than hand-editing**; it defines module configs, gains, and `createDrivetrain()`. `TunerConstantsOld.java` is a stale copy.
- **`tools/`** — utility/vendor helpers, mostly self-contained:
  - `FuelSim.java` — third-party game-piece physics simulation ([hammerheads5000/FuelSim](https://github.com/hammerheads5000/FuelSim)), driven from `Robot.simulationPeriodic`.
  - `Elastic.java` — Elastic-dashboard notification helper (vendored from Gold872/elastic-dashboard).
  - `QFRCLib.java` — QFRCDashboard integration over NetworkTables.
  - `PID`, `FieldUtil`, `DashboardSuite`, `CalcFromVision` (currently fully commented out).

### Vision

`Vision` runs **four PhotonVision cameras** (`frontCam`, `backCam`, `backRightCam`, `backLeftCam`) each with its own `PhotonPoseEstimator`, fusing AprilTag estimates into the swerve pose estimator. It supports `VisionSystemSim` for simulation and exposes hub-distance / rotation-error helpers consumed by the shooter and `LockOnHub`/`PointAndRotate`. Fusion can be toggled via `setVisionEnabled(...)` (see the gating logic in `Robot.java`).

### Autonomous (PathPlanner)

Paths and autos live in `src/main/deploy/pathplanner/` (`paths/*.path`, `autos/*.auto`). These are edited with the PathPlanner GUI and deployed as static files. Auto commands resolve through the `autoChooser`; `getAutoStartingPose()` pulls the starting pose from the selected `PathPlannerAuto`. Any command used inside a `.auto` must be registered via `NamedCommands.registerCommand(...)` in `RobotContainer` (name string must match exactly).

## Vendor dependencies

Defined in `vendordeps/*.json` (resolved from the local WPILib maven, see `settings.gradle`): WPILib New Commands, **CTRE Phoenix 6** (swerve + TalonFX), **PathPlannerLib**, **PhotonLib** (vision), and **AdvantageKit** (logging). Add new vendor libs via their vendordep JSON, not by editing `build.gradle` dependencies directly.

## Conventions / gotchas

- Hardware IDs (CAN IDs, ports) are hardcoded in the subsystem that owns the device (e.g. `Shooter` uses TalonFX 14). Confirm against the physical robot before changing.
- Lots of commented-out code and `SmartDashboard.putNumber(...)` tuning/DEBUG entries are intentional live-tuning scaffolding — don't assume they're cruft, but `getNumber` keys must match the `putNumber` keys (e.g. `Shooter.periodic` reads `"SetRPM"`).
- Commit history is informal; there is no branch/PR convention enforced here.
