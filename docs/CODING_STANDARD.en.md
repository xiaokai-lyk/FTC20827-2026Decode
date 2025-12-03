# FTC20827 - Coding Standards

Version: 2025.11

---

## 1. Naming Conventions (Mandatory)

- Package names: all lowercase, using reversed-domain style. The main package of this repository already uses `org.firstinspires.ftc.teamcode`, with subpackages organized by function (`subsystems`, `teleops`, `utils`, `external`).
- Class names: PascalCase, representing a single responsibility, e.g., `Hardwares`, `Shooter`, `MecanumDrive`.
- Interface names: PascalCase, typically describing capabilities or nouns.
- Method names: verb–noun camelCase, e.g., `initialize`, `updatePose`, `setShooter`.
- Fields/variables: camelCase, avoiding meaningless abbreviations (loop indices are exceptions).
- Constants: ALL_CAPS with underscores, e.g., `MAX_SPEED`.
- HardwareMap names: must match the strings used in `Hardwares` (e.g., `frontLeft`, `backRight`, `shooterFront`, `intake`). HardwareMap names and code constants/variables should map one-to-one for quick runtime identification.

Examples:

- Package: `org.firstinspires.ftc.teamcode.subsystems`
- Class: `Shooter` (file: `Shooter.java`)
- Hardware constants/variables: `mFrontLeft`, `shooterFront`, `preShooter`

---

## 2. Code Organization (Mandatory)

Following the existing structure (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode`):

- Organize by functional packages:

  - `subsystems/`: robot subsystems (Shooter, Intake, etc.)
  - `teleops/`: TeleOp OpModes and entry classes (e.g., `SingleTeleOp`)
  - `autos/`: autonomous OpModes
  - `utils/`: general utility classes (e.g., `MecanumDrive`, `ButtonEx`)
  - `external/`: third-party code (e.g., `PedroPathing`), kept in original structure for easier updates

- One class per file, filename equals class name (`Shooter.java` contains `public class Shooter`).
- Centralize hardware mapping in a single class (currently `Hardwares`). Do not repeat HardwareMap name strings across classes and avoid unnecessary hardware configuration.
- Avoid oversized classes: when a class exceeds ~400 lines or holds multiple responsibilities, split it into smaller classes or managers/controllers.
- Subsystems exposed to OpModes should provide clear control methods (e.g., `startIntake()`, `stopIntake()`, `setShooter()`), not direct access to hardware objects.

---

## 3. Special Requirements (Mandatory)

These apply to OpMode and hardware interaction, based on the existing design using FTCLib CommandScheduler, GamepadEx, and Hardwares:

- OpMode lifecycle: strictly follow `init`/`start`/`loop`/`stop` (or framework equivalents). Do not run long blocking tasks on the main thread.
- Hardware mapping: use the `Hardwares` class exclusively for HardwareMap access and initialization. Do not duplicate HardwareMap name strings in different OpModes.
- Hardware naming consistency: HardwareMap names (XML/phone configuration) must match the strings in `Hardwares` (e.g., `frontLeft`, `preShooter`). Any name change must be updated in `Hardwares` and documented in the PR.
- Telemetry and frequency: avoid excessively frequent telemetry updates; recommended normal status update rate is 10–20 Hz (every 50–100 ms). High-frequency telemetry should be used only for debugging and must be noted in the PR.
- Thread safety: all hardware access should be centralized and thread-safe. If hardware is used from multiple threads, document it clearly and use proper synchronization or message passing.
- Timeouts and failure handling: mechanical actions and peripheral operations must have reasonable timeouts and fallback behavior (e.g., sensor failures should produce defaults or safe-stop actions).
- Use of CommandScheduler: encapsulate behaviors as commands/subsystems and let `CommandScheduler` handle execution. OpMode entry classes should remain minimal.

---

## 4. PR Process (Recommended)

Goal: ensure all changes are reviewable, revertible, and pass baseline checks before merging.

- Commit messages:

  - Summary should begin with a type such as `fix` or `feature`
  - Summary should be ≤20 words describing the main change
  - Example: `[feature] Add Shooter subsystem with basic control`

- Pre-merge checklist (minimum requirements):

  - Relevant OpModes build locally and run on the robot.
  - Code follows the naming and organization rules in this document.
  - Key logic has comments or is explained in the PR, including risks and fallback plans.
  - Requires review from **two collaborators**, or **one collaborator plus GitHub Copilot**.
