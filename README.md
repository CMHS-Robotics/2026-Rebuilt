# 2026-Rebuilt — FRC Team 10333

Robot code for Team 10333's 2026 FRC season. Java, command-based WPILib, CTRE swerve drivetrain, PhotonVision pose estimation, and PathPlanner autonomous.

> **New to the repo?** Start here, then read [`CLAUDE.md`](CLAUDE.md) for a deeper map of how the code fits together.

## Getting set up

1. **Install WPILib 2026** — use the [official WPILib installer](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html). This installs the 2026 JDK, VS Code, and the Gradle/vendor tooling this project builds against.
2. **Open the project** in WPILib VS Code (`File → Open Folder`, pick this repo).
3. **Build it** to pull dependencies:
   ```bash
   ./gradlew build
   ```
   The first build downloads vendor libraries and can take a few minutes.

You don't need a globally-installed JDK or Gradle — the WPILib install and the `./gradlew` wrapper provide everything.

## Common tasks

Run these from the repo root (or use the WPILib VS Code command palette: `Ctrl/Cmd+Shift+P` → "WPILib").

| Task | Command |
| --- | --- |
| Build + run tests | `./gradlew build` |
| Run tests only | `./gradlew test` |
| Run in simulation | `./gradlew simulateJava` |
| Deploy to the robot | `./gradlew deploy` |
| View robot console output | `./gradlew riolog` |

**Simulation** opens the WPILib sim GUI with the DriverStation enabled, so you can drive the robot and watch the field/vision/FuelSim visualization without hardware.

**Deploy** sends code to the roboRIO. The team number (10333) comes from `.wpilib/wpilib_preferences.json`, so make sure you're on the robot's network first.

## How the robot works (high level)

The robot intakes game pieces and runs them through a feed chain to a flywheel shooter, aiming at the hub using vision:

```
Intake → Hopper → Indexer → Kicker → Shooter
```

- **Drivetrain** — CTRE swerve, configured by the Tuner-X-generated `TunerConstants`. Field-centric driving by default.
- **Vision** — four PhotonVision cameras fuse AprilTag readings into the robot's pose estimate, used for aiming and auto.
- **Autonomous** — built with PathPlanner; routines are chosen from the dashboard "Auto Chooser."

## Code layout

```
src/main/java/frc/robot/
├── Robot.java            # Robot lifecycle, logging, sim setup
├── RobotContainer.java   # Wires up subsystems, controls, and autos
├── Constants.java        # Tunable constants (camera mounts, vision trust, etc.)
├── subsystems/           # Hardware abstractions (Drivetrain, Vision, Shooter, ...)
├── commands/             # Robot behaviors (ShootSequence, Index, PointAndRotate, ...)
├── tools/                # Helpers (sim, dashboards, PID, field math)
└── generated/            # Tuner-X swerve config — regenerate, don't hand-edit

src/main/deploy/pathplanner/   # Auto paths & routines (edit in the PathPlanner app)
vendordeps/                    # Vendor library declarations
```

See [`CLAUDE.md`](CLAUDE.md) for what each subsystem/command does and the gotchas to watch for.

## Driver controls

Two controllers: **Driver** (port 0) and **Manipulator** (port 1). The active bindings live in `RobotContainer.configureBindings()` — many bindings there are commented out while features are being tuned, so treat that method as the source of truth and update this table when you change bindings.

Current Driver controls:

| Input | Action |
| --- | --- |
| Left stick | Translate (field-centric) |
| Right stick X | Rotate |
| Right trigger | Slow mode (~60% drive / 70% turn) |
| X | Lock on / aim at hub while driving |
| Y | Point-and-rotate to hub |
| A | Brake (X-lock wheels) |
| B | Point wheels at stick direction |
| POV Down | Re-zero field-centric heading |
| Back/Start + X/Y | SysId characterization routines |

## Contributing

- Keep one command per file in `commands/` and one subsystem per file in `subsystems/`.
- Hardware IDs (CAN IDs, ports) live in the owning subsystem — double-check them against the physical robot before changing.
- Don't hand-edit `generated/TunerConstants.java`; regenerate it from CTRE Tuner X.
- After changing controls, update the table above.
- Build before you push: `./gradlew build`.
