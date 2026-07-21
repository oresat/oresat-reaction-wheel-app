# Controller

## 1. Introduction

The Controller is responsible for determining how the reaction wheel should operate to achieve the desired system behavior. Whereas the Hardware Abstraction Layer provides deterministic access to the physical hardware, the Controller interprets those measurements, evaluates the current operating state, and computes the electrical command required to achieve the requested motor response.

The Controller forms the central decision-making component of the firmware architecture. It continuously receives the measured state of the motor—including rotor position, velocity, current, and voltage—from the HAL and compares those measurements against the desired operating point. Any difference between the desired and measured states is used to compute the command that will be issued to the motor during the current control iteration.

The Controller is intentionally independent of both the underlying hardware and the selected commutation strategy. It neither accesses MCU peripherals nor generates PWM waveforms directly. Likewise, it does not implement trapezoidal commutation, sinusoidal commutation, or field-oriented control. Instead, it produces a normalized electrical command that is subsequently interpreted by the active commutation algorithm, allowing multiple commutation strategies to share the same high-level control architecture.

This separation of responsibilities is fundamental to the firmware design. The HAL is responsible for observing the physical system, the Controller determines the desired electrical response, and the Commutation module converts that response into three-phase inverter commands. By isolating these responsibilities, each subsystem can evolve independently while maintaining a consistent interface to the remainder of the firmware.

Conceptually, the Controller occupies the center of the motor-control pipeline.

```text
             Measured Motor State
                     ▲
                     │
                Hardware
            Abstraction Layer
                     ▲
                     │
Motor ◄──────── Controller ───────► Commutation
                     │
                     ▼
             Electrical Command
```

Throughout operation, the Controller executes once during every iteration of the 16 kHz control loop. Each execution evaluates the current operating state, determines the appropriate control action, and produces a new electrical command for the commutation subsystem. Because this sequence is repeated at a fixed rate using synchronized measurements supplied by the HAL, the Controller provides deterministic closed-loop regulation while remaining independent of the hardware implementation and the selected commutation method.

## 2. Design Objectives

The Controller was designed to provide deterministic closed-loop regulation while remaining independent of both the underlying hardware and the selected commutation strategy. Rather than implementing hardware-specific behavior, its primary responsibility is to determine the electrical response required to achieve the commanded motor state using the measurements supplied by the Hardware Abstraction Layer.

Several engineering objectives guided the design of the controller architecture.

### 2.1 Deterministic Operation

The Controller executes as part of the fixed-rate 16 kHz control loop established by the firmware architecture. Every execution follows the same sequence of operations, processes a single synchronized set of measurements, and produces exactly one electrical command before returning control to the scheduler.

Maintaining deterministic execution ensures that the controller behaves consistently across all operating conditions. Since every commutation strategy is evaluated using the same execution frequency and measurement timing, experimental differences reflect the behavior of the control algorithms rather than variations in controller timing.

### 2.2 Modular Architecture

The Controller is responsible only for making control decisions. It neither interacts directly with hardware nor performs commutation.

This separation divides the firmware into three independent responsibilities:

- The Hardware Abstraction Layer measures the physical system.
- The Controller determines the desired electrical response.
- The Commutation module converts that response into three-phase inverter actuation.

Because each subsystem performs a distinct function, modifications to one layer generally do not require changes to the others. Improvements to the controller can therefore be developed without modifying the hardware interface, while new commutation algorithms can be introduced without altering the controller architecture.

### 2.3 Hardware Independence

The Controller operates exclusively on physical quantities expressed in engineering units. Rotor position, velocity, phase currents, phase voltages, and DC bus voltage are supplied by the HAL as calibrated measurements, allowing the Controller to remain independent of peripheral registers, ADC resolution, timer configuration, and board-specific implementation details.

Similarly, the Controller produces normalized electrical commands rather than hardware-specific PWM outputs. The conversion of these commands into switching signals is delegated entirely to the Commutation and HAL subsystems.

This abstraction allows the same control architecture to operate across multiple hardware revisions while minimizing software changes outside the HAL.

### 2.4 Commutation Independence

One of the primary objectives of this project is the comparison of multiple commutation strategies using identical hardware and experimental conditions. To support this objective, the Controller was intentionally designed so that its operation does not depend upon the selected commutation algorithm.

Regardless of whether the motor is operating using trapezoidal commutation, sinusoidal commutation, or field-oriented control, the Controller performs the same sequence of operations:

- Acquire the current motor state.
- Evaluate the control objective.
- Compute the required electrical command.
- Pass that command to the active commutation module.

The commutation algorithm determines how the electrical command is applied to the motor, but the decision-making process remains unchanged. This architectural separation ensures that experimental comparisons evaluate differences in commutation strategy rather than differences in controller implementation.

### 2.5 Maintainability

The Controller was also designed to support long-term maintainability and future expansion. Individual control functions are organized into well-defined modules with clearly separated responsibilities, allowing new operating modes, control strategies, or experimental features to be incorporated without restructuring the existing control architecture.

This modular approach proved particularly valuable throughout the development of the reaction wheel firmware. As additional commutation strategies and experimental capabilities were introduced, the Controller continued to provide the same high-level interface, allowing new functionality to be integrated while preserving the existing software structure.

### 2.6 Design Philosophy

Taken together, these objectives establish a controller that is deterministic, modular, and largely independent of both hardware implementation and commutation strategy. Rather than controlling individual peripherals or implementing motor-specific electrical algorithms, the Controller focuses exclusively on determining the electrical action required to achieve the desired operating behavior.

The following section examines how these design objectives are realized through the overall control architecture and the flow of information through the firmware during each iteration of the control loop.

## 3. Control Architecture

The Controller is organized as a sequence of independent processing stages that collectively determine the electrical action required to achieve the commanded motor behavior. Rather than operating as a single monolithic algorithm, the controller divides the decision-making process into a series of well-defined functional blocks, each responsible for a specific aspect of closed-loop control.

During every iteration of the 16 kHz control loop, information flows through these blocks in a fixed order. Measurements acquired by the HAL enter the controller, are interpreted according to the current operating state, and are transformed into an electrical command that is subsequently executed by the active commutation algorithm.

Conceptually, the control pipeline can be represented as

```text
        Measured Motor State
                 │
                 ▼
          System State
           Evaluation
                 │
                 ▼
       Reference Generation
                 │
                 ▼
        Closed-Loop Controller
                 │
                 ▼
       Electrical Command
                 │
                 ▼
      Active Commutation Method
                 │
                 ▼
           Motor Response
                 │
                 └─────────────┐
                               │
                               ▼
                     Updated Measurements
```

Each stage performs a single well-defined task before passing its output to the next stage. This organization simplifies both the firmware implementation and the overall control architecture by ensuring that individual stages remain responsible for only one aspect of the decision-making process.

### 3.1 State Evaluation

The first responsibility of the Controller is determining the current operating state of the system. Before any control calculations are performed, the controller evaluates whether the motor is idle, calibrating, starting, operating in closed-loop control, or responding to a fault condition.

The current operating state determines which portions of the controller execute during the present control iteration. For example, while the controller may regulate motor speed during normal operation, it intentionally suppresses actuator commands while the system is idle or responding to a fault. Separating operational behavior into distinct states prevents incompatible control actions from executing simultaneously and provides a predictable framework for managing transitions between operating modes.

### 3.2 Reference Generation

Once the operating state has been established, the Controller determines the desired operating point for the motor. Depending upon the active operating mode, this reference may represent a target speed, torque-producing current, voltage command, or another controlled quantity.

The controller intentionally separates reference generation from feedback regulation. Higher-level software specifies *what* the motor should achieve, while the feedback controller determines *how* the motor should respond to achieve that objective.

This distinction improves modularity by allowing operating modes to change independently of the underlying control algorithm.

### 3.3 Closed-Loop Regulation

The measured motor state is continuously compared with the desired operating point to determine the control error. The Controller processes this error to compute the electrical command required to reduce the difference between the desired and measured motor behavior.

Importantly, the Controller does not directly determine how this electrical command is applied to the motor. Instead, it produces an intermediate control output representing the desired electrical response. The selected commutation algorithm is then responsible for converting this command into the corresponding three-phase inverter actuation.

Separating regulation from commutation allows multiple commutation strategies to share the same high-level control architecture while differing only in the method used to generate the motor phase voltages.

### 3.4 Electrical Command Interface

The output produced by the Controller represents the interface between the decision-making logic and the electrical motor-control algorithms.

From the Controller's perspective, this output is simply the electrical action required to reduce the control error. The Controller neither generates PWM signals nor determines individual phase voltages. Those responsibilities belong exclusively to the Commutation module.

Consequently, the Controller remains largely independent of the electrical implementation used to drive the motor. Whether the firmware employs trapezoidal commutation, sinusoidal commutation, or field-oriented control, the Controller continues to produce the same logical output while the commutation subsystem determines how that command should be realized electrically.

### 3.5 Closed-Loop Information Flow

One complete iteration of the controller may therefore be summarized as

```text
Motor State
     │
     ▼
Evaluate Operating State
     │
     ▼
Generate Reference
     │
     ▼
Compute Control Error
     │
     ▼
Determine Electrical Command
     │
     ▼
Commutation Module
     │
     ▼
Motor
     │
     └──────────────► Repeat
```

This information flow remains unchanged regardless of the selected operating mode or commutation strategy. Every iteration of the control loop follows the same architectural sequence, allowing the Controller to provide deterministic decision-making while remaining independent of both the hardware implementation and the electrical algorithms responsible for driving the motor.

The following section examines the first stage of this architecture in greater detail by introducing the controller state machine and its role in coordinating the various operating modes of the reaction wheel firmware.

## 4. System State Machine

The controller is not continuously permitted to regulate the motor. During startup, calibration, fault conditions, and other transitional operating modes, applying closed-loop control would either produce invalid results or energize the motor under undefined conditions. The controller therefore employs a finite state machine to ensure that each stage of operation occurs in a well-defined and predictable sequence.

Rather than allowing every control function to execute simultaneously, the state machine determines which portions of the controller are active during each iteration of the control loop. This prevents incompatible operations from occurring at the same time and ensures that the motor progresses through a controlled sequence of operating modes from power-on through normal operation and, if necessary, into a safe shutdown.

Conceptually, the controller transitions between operating modes as shown below.

```text
            Power-On
                │
                ▼
             Idle State
                │
        Start Requested
                │
                ▼
          Calibration
                │
                ▼
            Startup
                │
                ▼
         Closed-Loop Control
                │
      ┌─────────┴─────────┐
      │                   │
      ▼                   ▼
 Command Stop         Fault Detected
      │                   │
      ▼                   ▼
   Idle State         Fault State
                          │
                          ▼
                   Recovery / Reset
```

Each state represents a distinct phase of operation with clearly defined responsibilities. While the controller architecture remains unchanged, the behavior of the controller differs according to the current operating state.

### 4.1 Idle State

The Idle state represents the default operating condition following initialization or after motor operation has been intentionally stopped. During this state, the controller does not attempt to regulate the motor and no electrical commands are issued to the commutation subsystem.

Although motor actuation is disabled, the remainder of the firmware continues operating normally. Measurements continue to be acquired, telemetry remains available, and the controller continues evaluating requests to transition into other operating states.

Separating the idle state from active control ensures that the motor remains electrically inactive until all prerequisites for normal operation have been satisfied.

### 4.2 Calibration

Before closed-loop control can begin, the controller must establish the calibration information required for accurate operation. This includes current sensor offset estimation, encoder alignment, electrical angle calibration, or verification of previously stored calibration parameters.

The controller intentionally prevents normal motor operation while calibration is active. This guarantees that every subsequent control calculation operates using valid measurement references established before torque-producing operation begins.

The calibration algorithms themselves are described in the Calibration chapter and are therefore not discussed further here.

### 4.3 Startup

Once calibration has completed successfully, the controller transitions into the startup state. During this phase, the controller prepares the motor for normal closed-loop operation by establishing the initial operating conditions required by the selected commutation strategy.

Unlike steady-state regulation, startup represents a controlled transition between an inactive motor and continuous closed-loop operation. By treating startup as an independent operating state, the controller avoids abrupt changes in electrical actuation while ensuring that all control variables begin from well-defined initial conditions.

The specific startup behavior depends upon the active commutation strategy and is therefore discussed in the Commutation chapter.

### 4.4 Closed-Loop Operation

Closed-loop operation represents the normal operating state of the reaction wheel. During every iteration of the control loop, the controller acquires the current motor state, compares it with the commanded operating point, computes the required electrical response, and issues a new command to the commutation subsystem.

This state contains the primary control algorithms responsible for regulating motor behavior and occupies the majority of the controller execution time during normal operation. The closed-loop regulation algorithms are introduced in the following sections of this chapter.

### 4.5 Fault State

If an abnormal operating condition is detected, the controller immediately transitions into the fault state. In this operating mode, normal control calculations are suspended and the controller requests that the Hardware Abstraction Layer place the inverter into its predefined safe state.

Remaining in a dedicated fault state prevents the controller from repeatedly attempting to resume motor operation while the underlying fault condition remains unresolved. Recovery from a fault therefore requires an explicit transition initiated by higher-level firmware once safe operation has been restored.

The mechanisms used to detect and classify individual fault conditions are implementation-specific and fall outside the scope of the controller architecture. From the controller's perspective, the fault state simply represents an operating mode in which normal motor regulation is intentionally disabled.

### 4.6 State Management

Although each operating state performs a different function, they all execute within the same deterministic control loop. The state machine therefore acts as the coordinator of the controller architecture, determining which control functions are permitted to execute during each control iteration while preserving a consistent execution structure throughout the firmware.

By separating controller behavior into well-defined operating states, the firmware achieves predictable transitions between initialization, normal operation, and fault handling while preventing incompatible control actions from executing simultaneously.

The following section examines the primary function performed during normal operation: the closed-loop regulation of motor speed.

## 5. Closed-Loop Speed Control

The primary objective of the controller during normal operation is to regulate the rotational speed of the reaction wheel. Rather than commanding a fixed electrical output, the controller continuously adjusts the motor actuation in response to changes in the measured rotor speed, allowing the reaction wheel to maintain the desired operating point despite disturbances such as bearing friction, aerodynamic drag, manufacturing tolerances, or variations in supply voltage.

This process is known as **closed-loop control** because every control decision is based upon continuous feedback from the motor itself. Instead of assuming that a particular electrical input will always produce the desired speed, the controller repeatedly measures the motor response, compares it with the commanded operating point, and adjusts the electrical command accordingly.

Conceptually, the feedback process forms a continuous loop.

```text
      Commanded Speed
             │
             ▼
        Controller
             │
             ▼
     Electrical Command
             │
             ▼
      Commutation System
             │
             ▼
           Motor
             │
             ▼
      Measured Speed
             │
             └──────────────► Controller
```

Because this loop executes every 62.5 μs as part of the 16 kHz control cycle, the controller continuously responds to changes in motor behavior while maintaining deterministic execution timing.

### 5.1 Why Feedback Is Required

If the motor always behaved identically under every operating condition, closed-loop control would be unnecessary. A fixed electrical command could simply be associated with a desired rotational speed, and the controller would never need to measure the motor once operation had begun.

Real motors, however, do not behave this way.

The electrical torque required to maintain a given rotational speed varies continuously throughout operation. Bearing friction changes with temperature, supply voltage fluctuates, mechanical loading varies, and the motor itself exhibits manufacturing tolerances that influence its electrical characteristics. Consequently, the same electrical command does not always produce the same rotor speed.

Consider a controller that applies a constant voltage to achieve a desired speed. If the mechanical load increases, the motor slows because the electrical input is no longer sufficient to balance the opposing torque. Likewise, if the load decreases, the motor accelerates beyond the commanded operating point.

Without feedback, the controller has no mechanism for detecting these changes and therefore no means of correcting them.

Closed-loop control solves this problem by continuously observing the measured motor speed and adjusting the electrical command whenever the measured speed deviates from the desired operating point.

The controller therefore regulates **error** rather than electrical output.

### 5.2 Speed Error

At the beginning of each control iteration, the controller compares the desired rotor speed with the measured rotor speed supplied by the Hardware Abstraction Layer.

The resulting speed error is

$$
e_{\omega}
=
\omega_{\mathrm{ref}}
-
\omega,
$$

where

- $\omega_{\mathrm{ref}}$ is the commanded rotor speed,
- $\omega$ is the measured rotor speed, and
- $e_{\omega}$ represents the instantaneous control error.

The sign of the error immediately indicates how the controller should respond.

- If the measured speed is below the commanded speed, the error is positive and additional motor torque is required.
- If the measured speed is above the commanded speed, the error is negative and the electrical command must be reduced.
- If the measured and commanded speeds are equal, the error approaches zero and only sufficient electrical effort is required to maintain steady-state operation.

The remainder of the controller is therefore responsible for determining the electrical response required to reduce this error while maintaining stable motor operation.

The following section introduces the proportional-integral controller used to convert the speed error into the electrical command supplied to the commutation subsystem.

### 5.3 Proportional Control

The simplest method of reducing the speed error is to make the electrical command directly proportional to the measured error. If the motor is rotating more slowly than commanded, the controller increases the electrical command. Conversely, if the motor is rotating faster than desired, the electrical command is reduced.

This relationship is expressed as

$$
u
=
K_p e_{\omega},
$$

where

- $u$ is the controller output,
- $e_{\omega}$ is the measured speed error, and
- $K_p$ is the proportional gain.

The proportional gain determines how aggressively the controller responds to changes in rotor speed. Larger values produce stronger corrective action for a given speed error, while smaller values result in a more gradual response.

Conceptually, proportional control behaves much like a spring. The farther the measured speed moves away from the commanded operating point, the greater the restoring effort produced by the controller.

```text
Large Negative Error ─────► Large Negative Command

        Small Error ─────► Small Command

Large Positive Error ─────► Large Positive Command
```

Because the controller output is directly proportional to the measured error, the motor naturally accelerates whenever it rotates too slowly and decelerates whenever it rotates too quickly. As the measured speed approaches the commanded speed, the error decreases and the controller output is reduced accordingly.

This simple feedback mechanism is sufficient to stabilize many systems and provides the foundation upon which more advanced controllers are built.

### 5.4 Limitations of Proportional Control

Although proportional control significantly improves motor regulation, it cannot completely eliminate steady-state speed error.

To understand why, consider the reaction wheel operating at a constant commanded speed. Even though the speed is no longer changing, the motor must still generate torque to overcome bearing friction, aerodynamic drag, and other mechanical losses. Producing this torque requires a non-zero electrical command.

Under proportional control, however, the electrical command is generated solely from the speed error.

As the measured speed approaches the commanded speed, the speed error becomes progressively smaller. Since the controller output is proportional to this error, the electrical command also decreases.

Eventually, the controller reaches an equilibrium where the remaining speed error is just large enough to generate the electrical effort required to balance the mechanical losses acting on the motor.

```text
Commanded Speed
        │
        ▼
Measured Speed
        │
        ▼
 Small Speed Error
        │
        ▼
 Small Electrical Command
        │
        ▼
 Torque balances losses
```

The consequence is that a proportional controller naturally settles with a small but non-zero speed error whenever continuous torque is required. Increasing the proportional gain reduces this error, but it cannot eliminate it entirely. Excessively large proportional gains also reduce controller stability and may introduce oscillatory behavior.

The controller therefore requires an additional mechanism capable of generating a sustained electrical command even after the speed error has become very small.

The following section introduces the integral component of the controller, which accumulates the remaining speed error over time and eliminates the steady-state offset inherent to proportional control alone.

### 5.5 Integral Control

The steady-state speed error produced by proportional control arises because the controller can only generate an electrical command while an error exists. Once the motor reaches a nearly constant speed, the remaining error becomes very small, limiting the controller's ability to produce the torque required to balance the continuous mechanical losses acting on the system.

To eliminate this residual error, the controller must retain information about how long the motor has remained away from the commanded operating point rather than considering only the instantaneous speed error.

This is accomplished using **integral control**.

Rather than responding only to the current error, the integral component continuously accumulates the error over time. Errors that persist for many control iterations therefore produce an increasingly large correction, while brief transient errors contribute very little to the accumulated value.

Mathematically, the integral term is expressed as

$$
I(t)
=
\int_0^t e_{\omega}(\tau)\,d\tau,
$$

where

- $e_{\omega}$ is the instantaneous speed error,
- $t$ is time, and
- $I(t)$ represents the accumulated error.

Within the firmware, this integration is performed numerically during each iteration of the 16 kHz control loop. Rather than evaluating a continuous integral, the controller repeatedly adds a small contribution proportional to the current speed error and the control period.

Conceptually,

```text
Control Loop

Error
 │
 ▼
+0.003
 │
 ▼
Integrator = 0.126

Next Loop

Error
 │
 ▼
+0.003
 │
 ▼
Integrator = 0.129

Next Loop

Error
 │
 ▼
+0.003
 │
 ▼
Integrator = 0.132
```

As long as a small speed error continues to exist, the accumulated integral value continues to grow. This increasing correction produces additional electrical effort until the remaining steady-state error disappears.

Conversely, once the measured speed matches the commanded speed, the accumulated error ceases to increase. The integral term therefore provides exactly the electrical effort required to maintain the desired operating point without requiring a persistent speed error.

Unlike the proportional term, which reacts immediately to changes in rotor speed, the integral term responds gradually. Its purpose is not to provide rapid correction, but to compensate for long-term disturbances that would otherwise produce a steady-state offset.

### 5.6 Proportional-Integral Controller

Neither proportional nor integral control alone provides the desired controller behavior.

A proportional controller reacts quickly to changes in speed but cannot eliminate steady-state error. An integral controller removes steady-state error but responds too slowly to rapidly changing operating conditions.

The firmware therefore combines both mechanisms into a proportional-integral (PI) controller.

The controller output is given by

$$
u
=
K_p e_{\omega}
+
K_i
\int_0^t e_{\omega}(\tau)\,d\tau,
$$

where

- $K_p$ is the proportional gain,
- $K_i$ is the integral gain,
- $e_{\omega}$ is the measured speed error, and
- $u$ is the electrical command supplied to the commutation subsystem.

The proportional component provides the immediate response necessary to correct rapid speed deviations, while the integral component gradually compensates for persistent disturbances and eliminates steady-state error.

The complementary behavior of these two terms can be summarized as

| Component | Primary Function | Response Characteristics |
|-----------|------------------|--------------------------|
| Proportional | Correct instantaneous speed error | Fast response |
| Integral | Remove steady-state error | Slow accumulated response |

Together, these two mechanisms provide stable and accurate speed regulation over the full operating range of the reaction wheel.

The resulting controller continuously adjusts its electrical command during every 16 kHz control iteration, allowing the motor to respond rapidly to changing operating conditions while maintaining zero steady-state speed error under normal operation.

In practice, however, the electrical command produced by the PI controller cannot increase without bound. The motor drive is ultimately limited by the available DC bus voltage and inverter capability. The controller must therefore constrain its output to remain within the achievable operating range while preserving stable behavior.

The following section examines these output limits and the additional considerations they introduce.

### 5.7 Output Limiting

The proportional-integral controller computes the electrical command required to reduce the measured speed error. In theory, this command may assume any magnitude necessary to achieve the desired operating point.

The motor drive hardware, however, cannot generate arbitrary voltages.

The inverter is powered from a finite DC bus, and the pulse-width modulation hardware can only command duty cycles within the range permitted by the available supply voltage. Consequently, every electrical command produced by the controller must ultimately remain within the physical capabilities of the motor drive.

This relationship is illustrated conceptually below.

```text
        PI Controller
             │
             ▼
   Requested Electrical Command
             │
             ▼
       Output Limiter
             │
      ┌──────┴──────┐
      │             │
Within Limits   Exceeds Limits
      │             │
      ▼             ▼
 Pass Through   Clamp to Limit
      │             │
      └──────┬──────┘
             ▼
      Commutation Module
```

The firmware therefore constrains the controller output before it is passed to the commutation subsystem. If the requested electrical command lies within the achievable operating range, it is transmitted unchanged. If the requested command exceeds the available voltage capability, the controller limits the output to the maximum value that the inverter can physically produce.

Mathematically, this operation may be expressed as

$$
u_{\mathrm{lim}}
=
\operatorname{clip}
\left(
u,\,
u_{\min},\,
u_{\max}
\right),
$$

where

- $u$ is the unconstrained controller output,
- $u_{\mathrm{lim}}$ is the limited output supplied to the commutation module,
- $u_{\min}$ and $u_{\max}$ represent the allowable electrical limits of the system.

This limiting operation guarantees that every command produced by the controller is physically realizable by the inverter and motor.

Output limiting becomes particularly important during large speed transients. For example, immediately after a significant increase in commanded speed, the speed error may be sufficiently large that the unconstrained PI controller requests substantially more electrical effort than the inverter is capable of delivering. Rather than attempting to generate an impossible command, the controller simply applies the maximum achievable output until the motor accelerates and the required control effort falls back within the available operating range.

Although output limiting protects the hardware and preserves physically realizable commands, it introduces an additional challenge for controllers that include an integral component. While the controller output is clamped at its maximum value, the integral term continues accumulating speed error internally, even though this accumulated correction can no longer influence the motor.

This phenomenon is known as **integrator windup**.

### 5.8 Integrator Windup

Consider a large positive speed command applied to a stationary motor.

Initially, the speed error is large, causing the proportional component to request a substantial electrical command. The combined PI controller rapidly reaches the maximum output permitted by the inverter, and the limiter constrains the command accordingly.

Although the applied electrical command can increase no further, the motor has not yet reached the desired speed. The speed error therefore remains positive, causing the integral term to continue accumulating error during every iteration of the control loop.

Conceptually, the situation appears as

```text
Large Speed Error
        │
        ▼
 PI Controller
        │
        ▼
 Requested Output
        │
        ▼
 Output Limiter
        │
        ▼
Maximum Achievable Voltage

Meanwhile...

Speed Error
      │
      ▼
Integrator
continues increasing
```

Because the controller output is already saturated, this additional accumulated integral value has no immediate effect on the motor. Instead, it becomes stored within the controller itself.

Once the motor eventually approaches the commanded speed, the proportional error begins to decrease. The integrator, however, may have accumulated a much larger correction than is actually required for steady-state operation.

The controller therefore continues commanding excessive electrical effort even after the speed error has nearly disappeared. The motor may overshoot the commanded operating point before the accumulated integral term gradually returns to an appropriate value.

This excessive accumulation of the integral state while the controller output is saturated is referred to as **integrator windup**.

Windup does not arise because the controller is mathematically incorrect. Rather, it occurs because the mathematical controller assumes unlimited actuator authority, whereas the physical motor drive is constrained by finite voltage and current limits.

To preserve stable closed-loop behavior, the firmware must therefore prevent the integrator from accumulating error whenever the controller output is unable to increase further.

The following section describes the anti-windup mechanism used by the firmware to achieve this behavior.

### 5.9 Anti-Windup

To prevent integrator windup, the firmware modifies the behavior of the integral component whenever the controller output reaches its allowable operating limits.

The underlying principle is straightforward. The integral term should continue accumulating error only while additional controller output can still influence the motor. Once the controller has reached the maximum electrical command that the inverter is capable of producing, further accumulation serves no useful purpose because the requested correction cannot be applied.

The firmware therefore temporarily suspends integration whenever the controller output is saturated **and** continued integration would drive the controller further into saturation.

Conceptually, the decision process can be represented as

```text
Controller Output
        │
        ▼
 Is Output Saturated?
        │
   ┌────┴────┐
   │         │
 No         Yes
   │         │
   ▼         ▼
Integrate  Would Integration
 Error      Increase Saturation?
                 │
            ┌────┴────┐
            │         │
           No        Yes
            │         │
            ▼         ▼
      Continue     Freeze
      Integrating  Integrator
```

This approach allows the integral term to resume normal operation as soon as the controller once again has authority to influence the motor.

For example, consider the motor accelerating toward a new commanded speed. Initially, the controller output may remain saturated because the requested electrical effort exceeds the available DC bus voltage. During this period, the integral state is prevented from growing unnecessarily.

As the motor accelerates, the speed error gradually decreases until the requested controller output falls below the saturation limit. At this point, the controller regains full authority over the motor, and the integral term resumes accumulating any remaining steady-state error.

The integrator therefore contributes only when it can actively improve controller performance rather than accumulating error that cannot be acted upon.

This simple modification significantly improves transient response by reducing overshoot, shortening recovery time following large command changes, and preventing unnecessarily long settling periods after saturation.

### 5.10 Controller Summary

The complete speed controller may now be viewed as a sequence of processing stages executed during every iteration of the control loop.

```text
          Commanded Speed
                 │
                 ▼
        Measured Speed
                 │
                 ▼
          Compute Error
                 │
                 ▼
        Proportional Term
                 │
                 ├──────────────┐
                 ▼              │
        Integral Term           │
                 │              │
                 └──────┬───────┘
                        ▼
                  PI Controller
                        │
                        ▼
                 Output Limiter
                        │
                        ▼
                 Anti-Windup Logic
                        │
                        ▼
              Electrical Command
                        │
                        ▼
               Commutation Module
```

Every execution of the controller follows this same sequence. Beginning with the measured motor state supplied by the Hardware Abstraction Layer, the controller computes the speed error, determines the required electrical correction using proportional and integral control, constrains the resulting command to the capabilities of the motor drive, and prevents unnecessary accumulation within the integral state whenever saturation occurs.

The resulting electrical command is then passed to the active commutation algorithm, which converts the controller output into the corresponding three-phase inverter actuation.

Although the mathematical operations performed by the controller are relatively straightforward, executing them deterministically within every 62.5 μs control period provides the reaction wheel with stable and repeatable closed-loop speed regulation across its entire operating range.

The following section examines how these controller operations are organized within the firmware during each iteration of the 16 kHz control loop.

## 6. Runtime Operation

The mathematical controller described in the previous sections represents only one aspect of the overall firmware. In practice, these calculations are embedded within a deterministic real-time control loop that executes continuously at a frequency of 16 kHz.

Each control iteration performs the complete sequence of measurement, decision-making, and command generation before returning control to the scheduler. By repeating this process every 62.5 μs, the firmware continuously updates the electrical command supplied to the motor while maintaining a fixed execution period.

Although the individual control calculations are relatively simple, their deterministic execution is fundamental to achieving stable and repeatable motor behavior.

### 6.1 Control Loop Execution

Every iteration of the controller follows the same sequence of operations.

```text
        Start Control Loop
                │
                ▼
      Acquire Motor Measurements
                │
                ▼
      Evaluate Controller State
                │
                ▼
     Determine Reference Command
                │
                ▼
        Compute Speed Error
                │
                ▼
      Execute PI Controller
                │
                ▼
      Apply Output Limiting
                │
                ▼
      Update Integral State
                │
                ▼
Generate Electrical Command
                │
                ▼
   Pass Command to Commutation
                │
                ▼
      End Control Loop
```

Because this sequence is executed identically during every control period, the controller exhibits highly predictable timing and behavior. No stage is skipped during normal operation, and each stage performs a single, clearly defined task before passing control to the next.

### 6.2 Measurement Processing

At the beginning of each control iteration, the Controller receives the latest calibrated measurements from the Hardware Abstraction Layer.

These measurements include the quantities required for closed-loop regulation, including rotor position, rotational speed, phase currents, phase voltages, DC bus voltage, and other system status information.

The Controller assumes that all measurements supplied by the HAL are already expressed in engineering units. Consequently, the Controller performs no ADC conversion, scaling, calibration, or hardware-specific processing. Its responsibility begins only after valid physical measurements have been produced.

This separation ensures that the Controller operates exclusively on physical system behavior rather than peripheral implementation details.

### 6.3 State Evaluation

Before executing any control calculations, the Controller evaluates its current operating state.

Depending upon the active state, the controller may

- remain idle,
- perform calibration,
- execute startup procedures,
- regulate motor speed, or
- enter fault handling.

Only the control functions associated with the active state are permitted to execute during the current control iteration.

This evaluation occurs before any closed-loop calculations are performed, preventing invalid control actions from being applied while the system is initializing or responding to abnormal operating conditions.

### 6.4 Closed-Loop Regulation

When the controller is operating in its normal closed-loop state, the measured rotor speed is compared with the commanded operating point to determine the instantaneous speed error.

The controller then executes the proportional-integral regulator described in the previous section, producing the electrical command required to reduce the measured error while respecting the physical limits of the inverter.

Internally, this process includes

- proportional correction,
- integral accumulation,
- output limiting, and
- anti-windup protection.

Collectively, these operations produce a single electrical command representing the desired motor actuation for the current control iteration.

### 6.5 Command Transfer

Once the controller has completed its calculations, the resulting electrical command is passed to the active commutation module.

At this point, responsibility transfers from the Controller to the Commutation subsystem.

The Controller does not determine phase voltages, PWM duty cycles, switching sequences, or inverter timing. Instead, it simply communicates the required electrical effort. The selected commutation algorithm—whether trapezoidal, sinusoidal, or field-oriented control—determines how this command is converted into three-phase inverter actuation.

This interface forms the architectural boundary between high-level decision-making and low-level motor drive implementation.

### 6.6 Continuous Operation

After the electrical command has been transferred to the Commutation subsystem, the current control iteration is complete.

The firmware then waits until the beginning of the next 16 kHz control period, at which point the entire process repeats using the latest motor measurements.

This repetitive execution forms a continuous feedback loop between the physical motor and the controller. Every control decision is therefore based upon the most recent observed motor behavior, allowing the reaction wheel to respond continuously to changing operating conditions while maintaining deterministic real-time execution.

The following section examines how these controller functions are organized within the firmware source code.

## 7. Source Code Organization

The Controller is implemented as a collection of modular software components, each responsible for a distinct aspect of the overall control architecture. Rather than concentrating all control functionality within a single source file, the firmware separates measurement handling, operating state management, closed-loop regulation, and commutation into independent modules with clearly defined interfaces.

This organization reflects the architectural principles described throughout this chapter: each subsystem owns a specific responsibility, communicates through well-defined interfaces, and remains largely independent of the implementation details of the surrounding firmware.

At a high level, the Controller occupies the central layer of the firmware architecture.

```text
          Application Layer
                 │
                 ▼
        Controller Module
                 │
      ┌──────────┴──────────┐
      ▼                     ▼
Hardware Abstraction     Commutation
       Layer              Algorithms
```

Within this architecture:

- The **Hardware Abstraction Layer** acquires and calibrates physical measurements from the motor and inverter hardware.
- The **Controller** interprets these measurements, evaluates the current operating state, and determines the electrical response required to achieve the commanded operating point.
- The **Commutation** subsystem converts the controller output into the corresponding three-phase inverter actuation.

This separation minimizes coupling between software components and allows each subsystem to evolve independently. For example, modifications to current sensing or encoder hardware affect only the Hardware Abstraction Layer, while improvements to field-oriented control remain confined to the Commutation subsystem. The Controller itself continues to operate using the same high-level interfaces.

During execution, the Controller receives calibrated measurements from the HAL, performs the closed-loop calculations described in this chapter, and forwards the resulting electrical command to the active commutation algorithm. This simple flow of information defines the primary responsibility of the Controller within the overall firmware architecture.

## 8. Engineering Decisions

Several architectural decisions guided the implementation of the Controller.

### Deterministic Execution

The Controller executes at a fixed frequency of 16 kHz with an identical sequence of operations performed during every control iteration. Maintaining deterministic execution simplifies controller analysis, improves repeatability during experimental testing, and ensures consistent interaction with the Hardware Abstraction Layer and Commutation subsystems.

### Separation of Responsibilities

The firmware intentionally separates measurement acquisition, decision-making, and electrical actuation into independent subsystems.

The Hardware Abstraction Layer measures the physical system.

The Controller determines the required electrical response.

The Commutation subsystem applies that response to the motor.

This separation reduces software complexity while allowing each subsystem to be developed, tested, and maintained independently.

### Commutation Independence

A primary objective of this project is the comparison of multiple commutation strategies under identical operating conditions. The Controller therefore remains independent of the underlying electrical drive algorithm.

Regardless of whether trapezoidal commutation, sinusoidal commutation, or field-oriented control is active, the Controller performs the same feedback calculations and produces the same abstract electrical command. Only the interpretation of that command changes within the Commutation subsystem.

This architectural boundary enables meaningful comparisons between commutation methods by ensuring that all strategies operate under the same closed-loop controller.

### Modularity and Maintainability

The modular organization of the Controller simplifies future development by isolating individual software responsibilities. New operating modes, additional control strategies, or future hardware revisions can be incorporated with minimal impact on the existing architecture, provided that the interfaces between subsystems remain unchanged.

This approach also improves software verification by allowing each subsystem to be validated independently before integration into the complete firmware.

## Chapter Summary

The Controller forms the decision-making core of the reaction wheel firmware. Using calibrated measurements supplied by the Hardware Abstraction Layer, it evaluates the current operating state, computes the required electrical response through deterministic closed-loop control, and supplies this command to the active commutation algorithm.

Throughout this chapter, the Controller has been presented as an abstraction independent of the electrical method used to drive the motor. It determines *how much* electrical effort is required, but not *how* that effort is physically applied.

The next chapter examines the Commutation subsystem, where these abstract electrical commands are transformed into the three-phase voltage waveforms that ultimately produce torque within the brushless DC motor.