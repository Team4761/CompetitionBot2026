# New Programmer Codebase Guide

This project is a WPILib command-based robot with a CTRE swerve drivetrain, PhotonVision pose updates, hand-written autos, and PathPlanner autos.

For a new programmer, the important mental model is:

1. `Robot` owns robot lifecycle and dashboard setup.
2. `RobotContainer` wires together subsystems, controller bindings, telemetry, autos, and PathPlanner named commands.
3. Subsystems expose robot actions as methods.
4. Commands call subsystem methods.
5. Bindings decide when commands run.
6. Autos are mostly command groups that compose existing commands.

## What To Learn First

### 1. Lifecycle And Composition

- Start in `src/main/java/frc/robot/Robot.java`.
- `robotPeriodic()` runs the scheduler every loop.
- `autonomousInit()` chooses and schedules the selected auto.
- `teleopInit()` cancels the auto command when teleop starts.

Then move to `src/main/java/frc/robot/RobotContainer.java`.

- The subsystem singletons are created near the top of the class.
- `configureBindings()` maps controllers to commands.
- `configAutos()` registers hand-written autos for the dashboard chooser.
- `configPathPlanner()` registers named commands and PathPlanner autos.

If a student understands `Robot` plus `RobotContainer`, they understand how the whole robot is stitched together.

### 2. How Subsystems Are Built Here

Subsystems live under `src/main/java/frc/robot/subsystems/...` and usually follow this pattern:

1. Create a class that extends `SubsystemBase`.
2. Construct hardware in the subsystem constructor.
3. Read IDs, ratios, and limits from `Constants`.
4. Expose small methods like `set...`, `turn...`, `stop...`, `enableCoasting()`.
5. Let commands call those methods instead of touching motors directly.

Examples:

- `IntakeSubsystem` owns an extender motor and an intake motor.
- `TurretSubsystem` owns shooter, kicker, spindexer, and aiming motors.
- `VisionSubsystem` owns camera configuration and pose filtering.

This project also hides motor setup behind wrapper classes:

- `src/main/java/frc/robot/util/SmartKrakenMotor.java`
- `src/main/java/frc/robot/util/SmartVortexMotor.java`

That is a major pattern to learn. New hardware code is usually:

1. Add constants.
2. Configure the motor wrapper in a subsystem constructor.
3. Add subsystem methods for the behavior you want.
4. Write commands that call those subsystem methods.

### 3. How Commands Are Built Here

Most commands in this codebase are simple wrappers around a subsystem action:

- `initialize()` starts something.
- `execute()` is often empty.
- `isFinished()` is often `false` for hold-style commands.
- `end()` stops the subsystem safely.

Examples:

- `IntakeCommand` starts intake and stops it when interrupted.
- `OuttakeCommand` does the reverse.
- `SpindexSpinCommand` and `KickerSpinCommand` are the same structure with different motor calls.

The second command pattern is a timed state machine using a `Timer`.

Examples:

- `ExtendCommand`
- `ExtendCommandCAUGHT`
- `INTERMITENTShootCommand`
- `ShootCommandSTUTTER`

The third pattern is a `SequentialCommandGroup` that chains existing commands into a feature or auto step.

Examples:

- `ShootAtAngleCommand`
- `NeutralZoneAuto`
- `MiddleBackupAuto`

### 4. How Bindings Work

Bindings are all in `RobotContainer.configureBindings()`.

Driver controller:

- Default command drives the swerve continuously.
- `back()` reseeds field-centric heading.
- Triggers run intake and outtake.

Operator controller:

- Triggers and bumpers run shooting modes.
- `leftBumper()` acts like a manual-override modifier.
- Combined triggers like `leftBumper().and(controller_operator.b())` create alternate manual controls.

For new programmers, the main rule is:

- Build the command first.
- Test it from Shuffleboard or a temporary button.
- Then attach it in `configureBindings()`.

### 5. How Autos Work

There are two auto systems in this project.

Hand-written autos:

- Registered in `configAutos()`.
- Implemented as `SequentialCommandGroup`s in `src/main/java/frc/robot/autos`.
- Good for learning because they are easy to read.

PathPlanner autos:

- Named commands are registered in `configPathPlanner()`.
- Path files live in `src/main/deploy/pathplanner`.
- The dashboard chooser is populated from `AutoBuilder.getAllAutoNames()`.

When students add a new robot action that PathPlanner should trigger, they need to:

1. Build the command.
2. Register it in `NamedCommands.registerCommand(...)`.
3. Use the same exact name in PathPlanner.

### 6. Where Configuration Lives

- Hardware ports, PID values, and gear conversions are mostly in `src/main/java/frc/robot/Constants.java`.
- Field geometry is in `src/main/java/frc/robot/FieldConstants.java`.
- CTRE swerve generator output is in `src/main/java/frc/robot/generated/TunerConstants.java`.
- Driver dashboard strings and chooser setup are in `Robot.java`.

New programmers should expect most "why is this motor on CAN ID 45?" questions to be answered in `Constants.java`.

## Repeatable Chunks To Reuse In A Redesign

These are the largest repeated patterns in the current codebase.

### 1. Motor Wrapper Configuration

The subsystem constructors repeat the same builder flow:

- choose port
- set PID
- set output range
- set angle limits
- choose mode
- set gear ratio

This is a strong candidate for reusable hardware config objects or small factory helpers, especially for Kraken-based mechanisms.

### 2. Start-Hold-Stop Command Wrappers

Many commands are almost identical:

- start motor in `initialize()`
- do nothing in `execute()`
- return `false`
- stop motor in `end()`

Examples include:

- `IntakeCommand`
- `OuttakeCommand`
- `SpindexSpinCommand`
- `KickerSpinCommand`
- parts of the shooter commands

This should probably become either:

- command factories on the subsystem, or
- a small reusable "run motor while held" command helper.

### 3. Timed Mechanism State Machines

Several commands are handwritten timer/state machines with slightly different timing logic:

- `ExtendCommand`
- `ExtendCommandCAUGHT`
- `STUTTERExtendCommand`
- `ShootCommand`
- `ShootWithPowerCommand`
- `ShootCommandSTUTTER`
- `INTERMITENTShootCommand`

These variants suggest the robot has a few mechanism behaviors with multiple tuning profiles, but the code represents them as separate classes instead of parameterized recipes.

For redesign, this is one of the highest-value cleanup targets:

- one configurable intake deploy sequence
- one configurable shooter sequence
- optional profiles like `NORMAL`, `STUTTER`, `INTERMITTENT`, `CAUGHT`

### 4. Auto Composition

Autos are mostly repeated command chains:

- do nothing buffer
- seed heading
- maybe extend intake
- maybe shoot
- drive a few relative moves
- rotate
- shoot again

That pattern appears across multiple auto classes. A redesign could replace many one-off auto classes with:

- a small library of reusable auto steps
- a shared auto builder/helper
- heavier reliance on PathPlanner with clean named commands

### 5. Controller Binding Registration

`configureBindings()` is currently one large method that mixes:

- drivetrain defaults
- driver controls
- operator controls
- manual override behavior
- dashboard side effects

This should be split during redesign into smaller methods such as:

- `configureDriveBindings()`
- `configureOperatorBindings()`
- `configureManualOverrideBindings()`
- `configureDisabledBehavior()`

That makes it much easier for new students to edit without breaking unrelated controls.

### 6. Auto Registration And Naming

Auto registration is duplicated in two places:

- hand-written chooser entries in `configAutos()`
- PathPlanner named command and chooser setup in `configPathPlanner()`

There are also signs of name drift and typo compatibility, which means the code is carrying legacy aliases.

That should become:

- one clear auto registry
- one source of truth for named command strings
- constants or enums for dashboard-facing auto names when possible

### 7. Dashboard Keys And UI Integration

`Robot`, `VisionSubsystem`, and the Elastic dashboard commands directly write many SmartDashboard keys as raw strings.

That is another repeatable chunk worth consolidating:

- centralize dashboard keys
- centralize telemetry publishing
- keep UI-only commands separate from mechanism logic

## Practical Teaching Order For New Students

Teach new programmers in this order:

1. Read `Robot.java` and `RobotContainer.java`.
2. Read one simple subsystem like `IntakeSubsystem`.
3. Read one simple command like `IntakeCommand`.
4. Follow that command into a binding in `configureBindings()`.
5. Read one command group auto like `MiddleBackupAuto` or `NeutralZoneAuto`.
6. Read one advanced subsystem like `VisionSubsystem` only after they understand the command-based flow.

That order gives them a working mental model before they touch the more advanced math and vendor-specific code.

## Recommended Redesign Targets

If the goal is to make the 2026 redesign easier to maintain and easier to teach, the best targets are:

1. Replace duplicated command classes with parameterized command factories.
2. Move timed mechanism variants into reusable sequence profiles instead of separate classes.
3. Split `RobotContainer` into smaller binding and registration helpers.
4. Centralize named command strings, auto registration, and dashboard keys.
5. Keep subsystems focused on hardware ownership and keep orchestration in commands/command groups.

## Short Version

New programmers mainly need to learn four things in this codebase:

1. Subsystems own hardware.
2. Commands call subsystem methods.
3. `RobotContainer` decides what buttons run which commands.
4. Autos are just command groups or PathPlanner paths using those same commands.

The biggest repeatable redesign chunks are motor setup, hold-style commands, timed mechanism sequences, auto registration, and controller binding structure.
