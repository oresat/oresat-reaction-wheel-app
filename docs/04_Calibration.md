# Calibration

## 1. Introduction

Accurate motor control depends upon the availability of reliable electrical and mechanical system parameters. Even when identical hardware is used, manufacturing tolerances, sensor offsets, component variation, and mechanical assembly differences introduce small discrepancies that degrade control performance if left uncompensated.

Rather than relying exclusively on nominal design values, the reaction wheel firmware performs an automated calibration procedure during commissioning to identify the characteristics of the assembled system. The resulting parameters are subsequently used throughout normal operation to improve measurement accuracy, establish the electrical relationship between the encoder and motor phases, and provide realistic motor parameters for the control algorithms.

Unlike many calibration routines that consist of isolated measurement procedures executed independently of the main control software, the calibration system implemented for this work is integrated directly into the normal firmware architecture. Calibration executes as a deterministic state machine within the same fixed 16 kHz control loop used during normal motor operation, ensuring identical timing behavior and eliminating the need for dedicated calibration firmware or special execution modes.

Successful completion of the calibration procedure produces a validated calibration snapshot containing the measured current sensor offsets, encoder electrical offset, motor phase resistance, motor phase inductance, and supporting system information. This snapshot becomes the authoritative source of calibration data used throughout subsequent controller operation. Once validated, the calibration data are stored in non-volatile memory and automatically restored during future system startup, eliminating the need to repeat the calibration procedure unless hardware changes or stored data become invalid.

The following sections describe the objectives, implementation, and execution of the calibration subsystem developed for the reaction wheel controller.

## 2. Design Objectives

The calibration subsystem was developed to satisfy several functional and architectural requirements while remaining compatible with the deterministic execution model established throughout the firmware.

First, the calibration process must execute entirely within the existing 16 kHz control loop. Every calibration step therefore performs a bounded amount of work during each control iteration, preserving deterministic execution time and avoiding lengthy blocking operations.

Second, all calibration results must be validated before they are accepted for controller use. Measurements that fall outside predefined electrical, mechanical, or statistical limits are rejected to prevent erroneous calibration data from degrading subsequent motor control. Configuration parameters defining acceptable operating ranges, timeout limits, current thresholds, encoder consistency limits, and motor parameter bounds are maintained separately from the calibration algorithm itself.

Third, calibration must remain independent of the Controller and the individual commutation strategies. Rather than directly manipulating PWM hardware or controller state, the calibration engine generates drive requests through the same commutation interface used during normal operation. This approach allows the calibration procedure to reuse the existing commutation infrastructure while maintaining clear separation between subsystem responsibilities.

Finally, successful calibration results must persist across power cycles. Once a complete and validated calibration has been obtained, the resulting parameter set is stored in non-volatile memory together with integrity information that allows the firmware to verify the stored record before restoring it during subsequent system startup.

Collectively, these design objectives produce a calibration subsystem that is deterministic, repeatable, fault tolerant, and tightly integrated with the overall reaction wheel firmware architecture while remaining logically independent of the controller and hardware abstraction layers.

## 3. Calibration Architecture

The calibration subsystem is implemented as a deterministic state machine that executes entirely within the normal 16 kHz control loop. Rather than performing calibration as a separate initialization routine or blocking procedure, each control iteration advances the calibration state machine by one bounded execution step before returning control to the remainder of the firmware. This execution model preserves the deterministic timing characteristics established throughout the software architecture while allowing complex calibration procedures to be performed over many thousands of control cycles.

Conceptually, the calibration process progresses through the sequence illustrated below.

```text
        Start Calibration
               │
               ▼
     Current Offset Calibration
               │
               ▼
         Rotor Lock Check
               │
               ▼
 Multi-Point Encoder Calibration
               │
               ▼
 Optional Phase Sweep Validation
               │
               ▼
 Motor Parameter Identification
               │
               ▼
     Calibration Validation
               │
               ▼
 Persistent Storage
               │
               ▼
      Controller Operation
```

Each stage performs a specific measurement while simultaneously validating that the measured quantities remain within predefined operating limits. Should any measurement violate these acceptance criteria, the calibration procedure immediately terminates and reports the corresponding failure condition rather than allowing potentially invalid parameters to propagate into the controller. The calibration subsystem therefore functions as both a parameter identification routine and a comprehensive validation process.

Unlike many calibration implementations that directly manipulate hardware peripherals, the calibration engine interacts with the remainder of the firmware exclusively through the existing software interfaces. Sensor measurements are supplied by the Controller and Hardware Abstraction Layer, while calibration drive commands are produced using the same commutation interface employed during normal motor operation. Consequently, calibration remains independent of the controller implementation, hardware drivers, and individual commutation algorithms while still leveraging the existing control infrastructure.

This separation of responsibilities substantially simplifies software maintenance. The calibration subsystem is responsible only for determining accurate system parameters, while flash management, controller operation, hardware access, and commutation remain the responsibility of their respective software modules.

## 4. Calibration Procedure

The calibration procedure consists of a sequence of measurement stages that progressively identify the electrical and mechanical characteristics required for accurate motor control.

Each stage depends upon the successful completion of the previous stage, ensuring that measurements are performed only after the necessary prerequisites have been established. For example, encoder calibration is not attempted until current sensor offsets have been removed and stable rotor lock has been achieved, while motor parameter identification is performed only after the encoder alignment has been validated.

This sequential organization minimizes measurement uncertainty while allowing each stage to verify the assumptions required by the following stage.

The complete calibration procedure performed by the reaction wheel controller consists of:

1. Current sensor offset calibration
2. Rotor lock verification
3. Multi-point encoder electrical offset calibration
4. Optional moving phase-sweep validation
5. Phase resistance and inductance identification
6. Validation and persistent storage

The following subsections describe each stage in detail.

### 4.1 Current Sensor Offset Calibration

The first stage of the calibration procedure determines the zero-current offsets associated with the phase current measurement system.

Although the current sensing hardware is designed to produce zero output when no current flows through the motor windings, practical analog circuits exhibit small offset voltages resulting from amplifier input offsets, component tolerances, and analog-to-digital converter bias. If left uncompensated, these offsets appear as false motor currents and introduce steady-state errors into the Field-Oriented Control current regulators.

To eliminate these systematic measurement errors, the calibration procedure begins with the inverter disabled and the motor electrically unexcited. During this interval, multiple samples of the measured phase currents are collected over successive control iterations.

Rather than storing offsets directly within the three-phase ABC reference frame, the measured phase currents are first transformed into the stationary αβ reference frame using the Clarke Transform. The resulting α-axis and β-axis currents are then accumulated and averaged to determine the corresponding current measurement offsets.

Once the required number of samples has been collected, the average offsets are computed and compared against predefined acceptance limits. Measurements that exceed the allowable offset magnitude indicate abnormal sensor behavior or hardware faults and immediately terminate the calibration procedure.

Successful completion of this stage produces calibrated α-axis and β-axis current offsets that are subsequently subtracted from all measured currents before they are used by the controller. Removing these offsets at the beginning of the signal-processing chain improves measurement accuracy for every commutation strategy while ensuring that subsequent calibration stages operate using corrected current measurements.

### 4.2 Rotor Lock Verification

Following current offset calibration, the firmware verifies that the rotor can be reliably aligned to a known electrical position before attempting encoder calibration.

The objective of this stage is not merely to hold the rotor stationary, but to establish a repeatable electrical reference from which the encoder offset can subsequently be measured. Any movement or instability during this process would directly reduce the accuracy of the encoder calibration and propagate errors into all future commutation calculations.

To achieve rotor alignment, the calibration engine commands a fixed d-axis excitation while maintaining zero q-axis command. This produces a stationary magnetic field within the stator that attracts the permanent-magnet rotor toward a known electrical angle without generating continuous rotational torque. The applied excitation is introduced gradually through a short voltage ramp, reducing transient current spikes while allowing the rotor to settle smoothly into its equilibrium position.

Once the settling period has elapsed, the firmware continuously monitors the measured α-axis and β-axis currents over multiple control iterations. Rather than relying upon a single measurement, the average current vector is calculated to reduce the influence of measurement noise and transient disturbances.

Several validation checks are then performed before calibration is permitted to continue.

- The encoder must report a valid rotor position.
- The measured lock current must lie within predefined minimum and maximum limits.
- The measured β-axis current must remain within an acceptable tolerance, indicating that the rotor has aligned with the commanded electrical axis.
- Sufficient measurement samples must have been collected to ensure statistical confidence.

Each of these conditions verifies a different aspect of the locking process. Insufficient current may indicate inadequate excitation or an electrical fault, excessive current may indicate abnormal loading or hardware failure, while a significant β-axis component suggests that the rotor has not settled to the intended electrical orientation.

Only after all validation criteria have been satisfied does the calibration procedure advance to encoder calibration. Any failure immediately terminates the calibration process and reports the corresponding failure condition, preventing inaccurate alignment measurements from being accepted by the controller.

### 4.3 Multi-Point Encoder Electrical Offset Calibration

Once stable rotor lock has been verified, the firmware determines the electrical offset between the encoder reference frame and the motor electrical reference frame.

Although the encoder provides an accurate measurement of the mechanical rotor position, its zero position is determined entirely by the physical installation of the encoder and therefore has no inherent relationship to the electrical phase orientation of the motor windings. Accurate commutation requires this relationship to be identified so that the measured mechanical angle can be converted into the correct electrical angle during normal operation.

Rather than estimating this relationship from a single measurement, the reaction wheel firmware performs a multi-point calibration across several equally spaced electrical positions. At each commanded electrical angle, the rotor is allowed to settle before multiple encoder samples are collected and averaged. The resulting electrical offset for that position is determined from the difference between the commanded electrical angle and the measured encoder angle.

Repeating this process at multiple electrical positions provides two important advantages.

First, averaging measurements obtained from several independent lock positions reduces the influence of encoder quantization, electrical noise, and small mechanical disturbances that may affect any individual measurement.

Second, the variation between the measured offsets provides a direct indication of calibration quality. Rather than assuming that every measurement is equally valid, the firmware evaluates the consistency of all measured offsets before accepting the final result.

After measurements have been collected at every calibration position, the individual offsets are combined using circular averaging to produce a single electrical offset representative of the complete data set. Circular averaging is particularly well suited to angular measurements because it correctly accounts for the periodic nature of rotational quantities and avoids discontinuities near ±π radians.

The firmware then computes the maximum deviation of every measured offset from the final averaged value. If this deviation exceeds the allowable tolerance, the encoder calibration is rejected and the calibration procedure terminates with a lock stability failure. Only measurements demonstrating sufficient repeatability are accepted for controller use.

Upon successful completion of this stage, the calculated electrical offset is immediately supplied to the encoder subsystem. All subsequent commutation algorithms therefore operate using an encoder reference frame that is correctly aligned with the electrical orientation of the motor.

### 4.4 Moving Phase-Sweep Validation

Following completion of the static encoder calibration, the firmware optionally performs a moving phase-sweep diagnostic to independently verify the calculated electrical offset.

Unlike the preceding calibration stages, this procedure does not generate the calibration value ultimately stored by the controller. Instead, it serves as an additional validation mechanism that compares the static encoder calibration against measurements obtained while the commanded electrical field rotates continuously.

During the phase sweep, the commanded electrical angle is gradually advanced through the full electrical revolution while the encoder simultaneously measures the resulting rotor position. Depending upon the selected configuration, this rotating magnetic field may be generated using either open-loop voltage excitation or closed-loop Field-Oriented Control current regulation. The latter provides improved current regulation throughout the sweep while exercising the same control infrastructure employed during normal operation.

To improve measurement quality, the sweep is performed in both the forward and reverse directions. Measurements are accepted only after the commanded electrical velocity has reached its intended operating value and the measured rotor velocity agrees with the commanded velocity within predefined limits. This gating process prevents acceleration transients from influencing the calculated electrical offset and ensures that only steady-state measurements contribute to the final diagnostic result.

Separate electrical offsets are determined from the forward and reverse sweeps before being combined through circular averaging. The agreement between these independently measured offsets provides an indication of the repeatability of the encoder calibration under dynamic operating conditions. Any significant disagreement between the two measurements or excessive deviation from the previously determined static calibration may indicate encoder mounting errors, mechanical compliance, or other dynamic effects that are not apparent during static rotor locking.

Although this diagnostic provides valuable engineering insight during firmware development and system validation, it does not replace the primary calibration procedure. The firmware continues to use the static multi-point encoder calibration as the authoritative electrical offset while the moving phase sweep serves solely as an independent verification of that result.

### 4.5 Motor Parameter Identification

Following encoder calibration, the firmware identifies the motor phase resistance and phase inductance required by the Field-Oriented Control algorithm.

Although nominal motor parameters may be supplied by the manufacturer, the electrical characteristics of an assembled system vary due to manufacturing tolerances, wiring resistance, connector losses, temperature, and measurement uncertainty. Directly identifying these parameters from the assembled reaction wheel therefore provides values that more accurately represent the electrical behavior of the complete system.

The identification procedure begins by allowing any residual winding current remaining from the previous calibration stages to decay naturally before controlled electrical excitation is applied. This ensures that resistance and inductance measurements begin from a well-defined initial condition and are not influenced by stored magnetic energy within the motor windings.

The firmware then applies a controlled excitation while monitoring the measured phase current response. From the resulting current behavior, estimates of the motor phase resistance and phase inductance are determined and compared against predefined acceptance limits configured within the firmware. Measurements falling outside these limits are rejected, preventing physically unrealistic parameters from being accepted for subsequent controller operation.

Successful completion of this stage produces calibrated estimates of the motor phase resistance and phase inductance that are subsequently supplied to the current controller and associated feedforward compensation algorithms. Because these parameters are identified directly from the assembled reaction wheel rather than assumed from nominal design values, they more accurately represent the electrical characteristics of the hardware used throughout the experimental evaluation presented in this thesis.

### 4.6 Calibration Validation

The final stage of the calibration procedure combines the results obtained throughout the previous stages into a single calibration snapshot representing the electrical characteristics of the reaction wheel system.

Before this snapshot is accepted, each measured quantity must satisfy the validation criteria established by the calibration subsystem. Current sensor offsets must remain within acceptable limits, the rotor lock must demonstrate stable alignment, the encoder calibration must exhibit sufficient consistency across all measurement positions, and the identified motor parameters must lie within physically reasonable operating ranges. In addition, calibration is continuously monitored for timeout conditions and acceptable DC bus voltage throughout the entire procedure.

Only when every stage has completed successfully is the calibration marked as valid. The resulting calibration snapshot contains the measured current offsets, encoder electrical offset, motor phase resistance, motor phase inductance, and supporting system information required during normal controller operation. This consolidated data structure forms the authoritative calibration record used by the remainder of the firmware.

By validating every measurement before it is accepted, the calibration subsystem minimizes the possibility of incorrect electrical parameters being supplied to the controller. This conservative approach improves overall system robustness while ensuring that subsequent motor control algorithms operate using a consistent and verified set of calibration data.

## 5. Persistent Calibration Storage

The calibration parameters identified during the calibration procedure are intended to remain valid across multiple power cycles. Repeating the complete calibration sequence every time the controller starts would unnecessarily increase startup time while providing little benefit unless the hardware configuration has changed.

To avoid this repeated initialization, the firmware stores the validated calibration snapshot in non-volatile memory after successful calibration. During subsequent startup, the stored calibration record is automatically restored, allowing the controller to begin normal operation immediately without repeating the complete calibration procedure.

Importantly, the calibration subsystem and the storage subsystem perform distinct responsibilities. The Calibration module is responsible for measuring electrical parameters, validating their physical correctness, and constructing the final calibration snapshot. The CalibrationStore module is responsible solely for persistent storage, record integrity, and retrieval. It deliberately performs no electrical or physical validation of the stored calibration values. Instead, these higher-level sanity checks remain the responsibility of the Calibration subsystem after the data have been loaded from memory.

This separation of responsibilities simplifies both software maintenance and future development. Changes to the calibration algorithm do not require modifications to the storage implementation, while improvements to flash management can be introduced without affecting the calibration procedure itself.

### 5.1 Calibration Record Format

Rather than storing individual calibration parameters independently, the firmware maintains a single calibration record containing all information required for subsequent controller operation.

This record includes the measured current sensor offsets, encoder electrical offset, identified motor parameters, calibration status, and supporting system information captured during the calibration procedure. Treating these values as a single logical record ensures that all parameters originate from the same successful calibration cycle and remain internally consistent.

Before a calibration record is written to non-volatile memory, additional metadata are incorporated to support integrity verification. Each stored record contains:

- a fixed magic identifier,
- a storage format version,
- the complete calibration data structure,
- a cyclic redundancy check (CRC).

These additional fields allow the firmware to distinguish valid calibration records from uninitialized flash contents, detect incompatible storage formats, and identify accidental data corruption resulting from interrupted writes or flash memory errors.

By storing the calibration information as a single versioned record rather than a collection of independent variables, future revisions of the firmware can extend the calibration structure while maintaining compatibility with previously stored calibration data.

### 5.2 Startup Validation

Whenever the controller initializes, the firmware attempts to restore the most recently stored calibration record from non-volatile memory.

However, the existence of stored data alone is insufficient to guarantee that the calibration is suitable for controller operation. Consequently, every stored record undergoes a sequence of validation checks before it is accepted.

The storage subsystem first verifies the structural integrity of the record by checking the stored magic identifier, storage format version, calibration status, and cyclic redundancy check. Records failing any of these tests are immediately rejected, preventing incomplete or corrupted flash contents from being interpreted as valid calibration data.

After the record has passed these storage integrity checks, the Calibration subsystem performs an additional series of physical reasonableness tests. Motor phase resistance, phase inductance, and current sensor offsets must all lie within acceptable operating ranges before the calibration is accepted for normal controller operation. These checks provide protection against structurally valid records containing physically unrealistic values that could otherwise degrade motor control performance.

Only after both the storage integrity checks and the physical validation checks have completed successfully are the calibration parameters transferred to the controller and encoder subsystems. If any stage fails, the stored calibration is rejected and the firmware requires a new calibration procedure before normal operation can continue.

### 5.3 Separation of Responsibilities

The persistent calibration architecture follows the same modular design philosophy adopted throughout the remainder of the reaction wheel firmware.

The Calibration subsystem is responsible for determining the electrical characteristics of the motor, validating the measured parameters, and constructing the calibration snapshot used during controller operation. In contrast, the CalibrationStore subsystem is responsible solely for storing and retrieving that snapshot from non-volatile memory.

Consequently, the storage subsystem contains no knowledge of motor control, encoder alignment, current sensing, or controller behavior. Its responsibilities are limited to flash management, record serialization, version compatibility, and integrity verification. Likewise, the Calibration subsystem contains no knowledge of flash layout or non-volatile storage implementation beyond requesting that a validated calibration snapshot be saved or restored.

This separation significantly reduces coupling between the two modules while improving maintainability. Modifications to the calibration algorithm can therefore be implemented independently of the storage mechanism, and future changes to the underlying flash storage implementation do not require modification of the calibration procedure itself. This modular organization is consistent with the broader firmware architecture presented throughout this thesis, where individual subsystems communicate through well-defined interfaces while remaining responsible only for their own functionality.

## 6. Runtime Operation

Unlike many embedded motor control systems, calibration within the reaction wheel controller is not implemented as a separate execution mode or dedicated initialization routine. Instead, the calibration engine operates as part of the normal controller execution path and is updated once during every iteration of the deterministic 16 kHz control loop.

During each control cycle, the Controller supplies the calibration subsystem with the most recent sensor measurements, including phase currents, DC bus voltage, and encoder state. The calibration state machine then performs the bounded amount of computation associated with its current operating state before generating the appropriate drive command through the existing commutation interface.

Conceptually, the execution sequence is illustrated below.

```text
          Controller Update
                 │
                 ▼
      Acquire Sensor Measurements
                 │
                 ▼
     Calibration_Update16kHz()
                 │
                 ▼
     Advance Calibration State
                 │
                 ▼
   Generate Drive Command
                 │
                 ▼
      Selected Commutation
                 │
                 ▼
        PWM Generation
                 │
                 ▼
           Motor Response
```

Because each calibration state performs only a limited amount of work during each control iteration, the computational load remains predictable regardless of which calibration stage is currently executing. Long-duration operations, such as current averaging, encoder sampling, and parameter identification, are therefore naturally distributed across many control cycles rather than being performed as blocking computations.

Once calibration has completed successfully, the state machine terminates and returns control to the normal operating controller. If any validation step fails, calibration immediately reports the corresponding failure condition and ceases issuing calibration commands, preventing invalid parameters from propagating into subsequent motor operation.

## 7. Source Code Organization

The calibration subsystem is organized into several independent software modules, each responsible for a clearly defined aspect of the overall calibration process. This modular organization follows the same architectural principles employed throughout the remainder of the reaction wheel firmware.

The primary calibration algorithm is implemented within the Calibration module. This module contains the deterministic calibration state machine, performs all electrical measurements, validates the resulting parameters, and maintains the active calibration snapshot used by the controller. It also provides the public interface through which calibration may be started, monitored, and queried by the remainder of the firmware.

Persistent storage is implemented separately within the CalibrationStore module. Rather than participating in the calibration algorithm itself, this module is responsible for serializing calibration records, interacting with Zephyr's Non-Volatile Storage (NVS) subsystem, and verifying record integrity through version and cyclic redundancy check validation. This separation ensures that flash management remains independent of the calibration algorithm while providing a reusable storage interface for future firmware revisions.

Calibration configuration parameters are maintained independently within the central firmware configuration file. Sampling durations, validation thresholds, operating limits, timeout values, and optional diagnostic features are therefore defined separately from the calibration implementation itself, allowing calibration behavior to be modified without altering the underlying algorithm.

Collectively, these modules provide a calibration subsystem that is both highly cohesive and loosely coupled. Each module performs a single well-defined function while interacting with the remainder of the firmware through a minimal public interface, improving readability, maintainability, and long-term extensibility.

## 8. Engineering Decisions

Several engineering decisions influenced the implementation of the calibration subsystem, each intended to improve measurement reliability while preserving the deterministic architecture of the reaction wheel controller.

The first design decision was to execute calibration entirely within the normal 16 kHz control loop. Rather than creating a dedicated calibration scheduler or introducing blocking measurement routines, every calibration stage advances incrementally during successive control iterations. This approach preserves deterministic execution timing and allows calibration to reuse the existing controller, commutation, and hardware abstraction infrastructure.

A second design decision was to validate every measured parameter before it is accepted for controller use. Rather than assuming successful measurements, the firmware verifies current sensor offsets, rotor alignment, encoder consistency, motor parameter ranges, supply voltage, and execution time throughout the calibration process. Rejecting invalid measurements before they reach the controller substantially improves system robustness and reduces the likelihood of difficult-to-diagnose control faults.

The encoder calibration procedure was similarly designed to prioritize repeatability over simplicity. Instead of determining the electrical offset from a single rotor lock position, multiple electrical lock positions are sampled and combined through circular averaging while simultaneously evaluating the consistency of the measured offsets. This additional processing increases confidence that the final encoder offset accurately represents the electrical alignment of the assembled reaction wheel.

Finally, persistent storage was intentionally separated from parameter identification. By dividing the responsibilities of parameter measurement and flash management into independent software modules, the firmware maintains clear subsystem boundaries while simplifying future modifications to either the calibration algorithm or the storage implementation. This separation is consistent with the broader architectural philosophy adopted throughout the reaction wheel controller, where individual software modules communicate through well-defined interfaces while remaining responsible only for their own functionality.

## 9. Chapter Summary

This chapter presented the calibration subsystem developed for the reaction wheel controller. Unlike conventional initialization routines, calibration is implemented as a deterministic state machine that executes entirely within the normal 16 kHz control loop while remaining integrated with the existing controller, commutation, and hardware abstraction architecture.

The calibration procedure determines the electrical characteristics required for accurate motor control, including current sensor offsets, encoder electrical alignment, and motor electrical parameters. Each stage incorporates validation criteria to ensure that only physically reasonable measurements are accepted, improving the reliability of the resulting controller parameters. Successfully identified calibration data are consolidated into a validated calibration snapshot and preserved in non-volatile memory, allowing future controller startups to restore previously measured parameters without repeating the complete calibration procedure.

The modular separation between calibration, persistent storage, hardware abstraction, and controller operation further improves software maintainability while preserving the deterministic execution model established throughout the firmware.

With an accurate and validated set of electrical parameters established, the controller is able to perform reliable motor control and provide meaningful experimental measurements. The following chapter describes the telemetry subsystem developed to communicate these measurements to the host computer for real-time monitoring, data logging, and post-processing throughout the experimental evaluation.