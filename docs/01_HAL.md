# Hardware Abstraction Layer (HAL)

## 1. Introduction

The Hardware Abstraction Layer (HAL) provides the interface between the motor-control firmware and the physical hardware of the reaction wheel electronics. It owns every peripheral directly responsible for motor actuation and measurement, including the PWM generators, ADC subsystem, quadrature encoder interface, gate-driver interface, timing infrastructure, and board-specific initialization. Higher-level firmware operates exclusively on physical quantities such as phase current, DC bus voltage, rotor position, and PWM duty cycle, while the HAL is responsible for converting those quantities to and from the underlying microcontroller peripherals.

The primary objective of the HAL is deterministic hardware interaction. Every commutation strategy implemented by the firmware—trapezoidal, sinusoidal, and field-oriented control (FOC)—must execute using identical measurements, identical timing, and identical actuator interfaces so that any observed differences in experimental performance originate from the control algorithm rather than variations in hardware access. The HAL therefore establishes a common measurement and actuation layer shared by the entire control stack.

Unlike a traditional Board Support Package (BSP), the HAL is not intended to provide a generic abstraction over the microcontroller. Instead, it exposes only the hardware functionality required by the reaction wheel controller while preserving precise control over timing and execution order. This approach allows peripheral behavior to remain explicit within the firmware while isolating device-specific implementation details from the controller and commutation modules.

Figure X illustrates the architectural position of the HAL within the firmware.

```text
                High-Level Firmware
 ┌────────────────────────────────────────────┐
 │             Experiment Framework           │
 │               Telemetry                    │
 │               Calibration                  │
 │               Controller                   │
 │            Commutation Algorithms          │
 └────────────────────────────────────────────┘
                     ▲
                     │ Physical Measurements /
                     │ Actuation Commands
                     ▼
 ┌────────────────────────────────────────────┐
 │        Hardware Abstraction Layer          │
 │                                            │
 │ PWM │ ADC │ Encoder │ Timers │ GPIO │ SPI  │
 └────────────────────────────────────────────┘
                     ▲
                     │
                     ▼
 ┌────────────────────────────────────────────┐
 │       MCXN947 + Power Electronics          │
 │                                            │
 │ LMG2100 │ Shunts │ Encoder │ Sensors │ MCU │
 └────────────────────────────────────────────┘
```

Rather than implementing control algorithms, the HAL guarantees that every measurement and every PWM update occur at deterministic points within the switching cycle. This timing guarantee forms the foundation of the current measurement accuracy, control-loop stability, and experimental repeatability required throughout the project.

The HAL intentionally owns only hardware interaction. It does not implement current regulation, velocity control, coordinate transforms, observers, calibration algorithms, telemetry formatting, or experiment sequencing. Those responsibilities belong to higher firmware layers that consume the measurements and actuator interfaces provided by the HAL.

## 2. Design Objectives

The HAL was designed to provide a deterministic and hardware-independent interface between the control algorithms and the reaction wheel electronics. Rather than allowing individual control modules to interact directly with MCU peripherals, all hardware access is centralized within the HAL, ensuring that every commutation strategy executes under identical electrical and timing conditions. This isolation allows the controller and commutation implementations to evolve independently of the underlying hardware while maintaining a consistent execution environment.

The primary design requirement is deterministic execution. The controller executes at a fixed 16 kHz update rate, corresponding to a control period of 62.5 μs. During each control cycle, the HAL must acquire all required measurements, update the inverter outputs, and provide the controller with a complete and self-consistent snapshot of the motor state before the next iteration begins. Variable execution latency or asynchronous hardware access would introduce sampling jitter, degrade current regulation, and reduce the repeatability of experimental measurements.

The HAL therefore adopts a strictly synchronous execution model. All measurements are acquired at fixed points within the PWM cycle, and all actuator updates occur at deterministic instants relative to those measurements. No blocking operations, dynamic memory allocation, operating system services, or communication interfaces participate in the control path. Every execution of the control loop performs the same sequence of operations in the same order, allowing the execution time to remain bounded and highly repeatable.

Another major design objective is measurement consistency across commutation strategies. Trapezoidal commutation, sinusoidal commutation, and field-oriented control all operate on the same measured phase currents, DC bus voltage, rotor position, and rotor velocity. The HAL intentionally provides these quantities without embedding any control-specific behavior so that each commutation algorithm receives an identical representation of the physical motor state. Consequently, differences observed during experimental characterization reflect differences in the control algorithms rather than differences in sensing or hardware interaction.

Hardware ownership is similarly centralized. The HAL owns the initialization, configuration, and runtime operation of every peripheral directly involved in motor control. Higher-level firmware never configures ADC conversions, modifies PWM timing, accesses encoder registers, or manipulates gate-driver hardware directly. Restricting hardware ownership to a single subsystem significantly reduces coupling between modules and simplifies future hardware revisions by confining board-specific changes to a well-defined portion of the firmware.

The HAL is intentionally narrow in scope. It provides measurements and actuator interfaces but performs no interpretation of those measurements beyond the processing required to convert peripheral data into meaningful physical quantities. Current regulation, velocity control, coordinate transformations, observers, calibration procedures, telemetry generation, and experiment sequencing remain the responsibility of higher-level modules. Maintaining this separation of responsibilities keeps the hardware interface stable while allowing the control algorithms to evolve independently.

Finally, the HAL was designed with hardware evolution in mind. The current V1 hardware provides inline phase-current sensing, DC bus voltage measurement, quadrature encoder feedback, and limited temperature monitoring. The interface, however, is structured such that additional sensing capabilities planned for future hardware revisions—including independent phase-voltage measurement and per-phase temperature sensing—can be incorporated with minimal impact on the controller or commutation modules. This separation allows improvements to the measurement hardware without altering the higher-level control architecture.

## 3. Hardware Ownership

The HAL owns every hardware resource required to measure the motor state and actuate the three-phase inverter. It provides the only interface through which higher-level firmware interacts with the physical system, ensuring that peripheral configuration, timing, and hardware-specific behavior remain isolated from the controller and commutation algorithms. This ownership model allows the remainder of the firmware to operate entirely in terms of physical quantities rather than peripheral registers or MCU-specific driver interfaces.

The V1 reaction wheel hardware is centered around the NXP MCXN947 microcontroller, which executes the complete control stack at a fixed 16 kHz update rate. Three half-bridge gate drivers generate the inverter switching signals required to energize the brushless DC motor, while inline current sensing, DC bus voltage measurement, and a quadrature encoder provide the electrical and mechanical feedback required for closed-loop control. Figure X summarizes the hardware resources managed by the HAL.

| Hardware | Firmware Responsibility |
|----------|-------------------------|
| PWM timers | Three-phase inverter switching |
| ADC subsystem | Phase current and DC bus voltage acquisition |
| Quadrature encoder | Rotor position measurement |
| GPIO | Gate enable, status signals, LEDs |
| Timing infrastructure | Deterministic control-loop timing |
| SPI / peripheral interfaces | External hardware communication where required |

The HAL owns both the initialization and runtime operation of these peripherals. During system startup it configures each peripheral into a known operating state before enabling motor operation. Once initialization is complete, the HAL becomes responsible for maintaining deterministic interaction with the hardware throughout every control cycle. Higher-level modules never modify peripheral configuration or access hardware registers directly.

An important design principle is that ownership extends beyond configuration to include hardware timing. For example, the controller does not request an ADC conversion or manually update PWM outputs. Instead, the HAL ensures that conversions occur at predetermined points within the PWM cycle and that new duty cycles are applied at deterministic update events. This guarantees that every controller iteration operates on measurements acquired under identical electrical conditions.

The HAL intentionally exposes hardware through physical quantities rather than peripheral abstractions. Instead of returning ADC counts, timer capture values, or encoder register contents, it provides measurements expressed in engineering units that can be consumed directly by the remainder of the firmware. The controller therefore operates on quantities such as amperes, volts, radians, radians per second, and normalized duty cycles without requiring knowledge of the underlying hardware implementation.

Equally important is defining what the HAL does **not** own. It does not determine desired motor torque, regulate current, estimate velocity, perform coordinate transformations, execute commutation algorithms, or manage experiment sequencing. Those responsibilities belong to higher-level firmware modules that consume the measurements produced by the HAL. This separation prevents hardware-specific implementation details from propagating into the control algorithms and significantly reduces coupling between firmware subsystems.

The hardware ownership model also simplifies future hardware revisions. Should the sensing topology, gate driver, encoder technology, or microcontroller change, only the HAL is expected to require significant modification. As long as it continues to provide the same physical interface to the remainder of the firmware, the controller and commutation algorithms remain largely unaffected. This abstraction was an explicit architectural decision made to support the planned evolution from the V1 hardware platform to future revisions incorporating additional sensing capabilities.

## 4. Firmware Architecture

The HAL is organized as a thin abstraction layer between the physical hardware and the remainder of the firmware. Its primary responsibility is to acquire measurements from the reaction wheel hardware, convert those measurements into consistent engineering quantities, and apply actuator commands generated by the higher-level control algorithms. It intentionally contains no control policy or decision-making logic; instead, it serves as a deterministic translation layer between hardware peripherals and the control stack.

From the perspective of the controller, the HAL behaves as a hardware service provider. During each control cycle it updates the measured motor state, after which the controller computes the desired electrical actuation. The resulting phase commands are then returned to the HAL, which converts those commands into the PWM outputs required by the inverter. This bidirectional data flow forms the foundation of every control iteration.

```text
                 Measurements
         ┌─────────────────────────┐
         │                         │
         ▼                         │
+----------------+         +------------------+
|      HAL       |────────▶|    Controller    |
|                |         +------------------+
| ADC            |                  │
| Encoder        |                  │
| PWM            |                  ▼
| Timers         |         +------------------+
+----------------+         |   Commutation    |
         ▲                 | (Trap/Sine/FOC)  |
         │                 +------------------+
         │                         │
         └─────────────────────────┘
              Duty-cycle Commands
```

Although presented as a single subsystem, the HAL internally consists of several specialized components that collectively implement the hardware interface. These components remain tightly coupled because they must operate with deterministic timing, yet each owns a distinct portion of the measurement or actuation pipeline.

| Component | Primary Responsibility |
|-----------|------------------------|
| PWM interface | Three-phase inverter actuation |
| ADC interface | Synchronized current and voltage acquisition |
| Encoder interface | Rotor position measurement |
| Timing infrastructure | Deterministic control-loop scheduling |
| GPIO interface | Gate-driver control and board-specific digital I/O |
| Hardware initialization | Peripheral configuration and startup sequencing |

The runtime architecture follows a strict ownership model. Each peripheral is configured once during initialization and thereafter accessed exclusively through the HAL. Higher-level modules never manipulate timer registers, trigger ADC conversions, or access encoder hardware directly. This centralization ensures that peripheral configuration remains internally consistent and prevents timing assumptions made by one subsystem from inadvertently affecting another.

An equally important architectural decision is that the HAL communicates exclusively through physical quantities rather than peripheral representations. Internal register layouts, ADC conversion results, encoder counts, and timer compare values remain implementation details of the HAL. By the time information reaches the controller, it has been converted into meaningful engineering quantities with consistent units. Likewise, actuator commands received from the commutation algorithms are expressed as normalized phase commands rather than device-specific timer values. This separation allows higher-level firmware to remain independent of both the underlying microcontroller and future hardware revisions.

The HAL is therefore positioned as the boundary between two fundamentally different domains. Below the HAL, firmware interacts with peripherals, registers, interrupts, and electrical signals. Above the HAL, firmware operates entirely in terms of motor-state estimation, control algorithms, and physical quantities. Maintaining this separation significantly simplifies the implementation of the controller and commutation modules while confining hardware-specific complexity to a single subsystem.

The following sections examine each portion of this hardware interface in detail, beginning with the generation of the three-phase PWM signals that drive the inverter.

## 5. PWM Generation and Inverter Interface

The primary actuator managed by the HAL is the three-phase voltage source inverter that drives the brushless DC motor. Every commutation strategy implemented by the firmware ultimately produces the same output: a set of three normalized phase voltage commands. The responsibility of the HAL is to translate those commands into deterministic PWM waveforms suitable for the gate-driver hardware while guaranteeing predictable timing and safe operation.

The reaction wheel employs a conventional three-phase bridge consisting of six switching devices arranged as three half-bridges. By rapidly switching each half-bridge between the positive DC bus and ground, the inverter synthesizes the average phase voltages required to generate the desired stator magnetic field. Rather than controlling the switches directly, the controller specifies the desired phase voltages, while the HAL manages the low-level timing required to produce them.

### 5.1 Pulse Width Modulation

Pulse Width Modulation (PWM) controls the average voltage applied to each motor phase by varying the fraction of time that each switching device remains enabled during a fixed switching period. If the DC bus voltage is denoted by $V_{DC}$ and the normalized duty cycle by $D$, the average phase voltage is approximately

$$
V_{\text{phase}} = D \cdot V_{DC},
$$

where

$$
0 \le D \le 1.
$$

The controller therefore does not command absolute voltages directly. Instead, it generates normalized voltage commands that are converted by the HAL into timer compare values corresponding to the desired duty cycles. This normalization allows the higher-level control algorithms to remain independent of timer resolution and PWM frequency.

### 5.2 Center-Aligned PWM

The firmware employs center-aligned PWM rather than edge-aligned PWM. In a center-aligned timer, the counter increments from zero to the reload value before decrementing back to zero, producing a symmetric switching waveform about the midpoint of the PWM period.

```text
Counter

TOP        /\        /\
          /  \      /  \
         /    \    /    \
0_______/      \__/      \____ Time
```

Center-aligned PWM was selected for several engineering reasons.

- Symmetric switching reduces harmonic distortion in the synthesized phase voltages.
- The midpoint of the PWM period provides a stable interval during which phase currents have largely settled following switching transitions.
- ADC conversions can therefore be synchronized near the electrical center of the PWM period, reducing switching noise in the measured currents.
- Symmetric switching minimizes timing bias between rising and falling edges, improving current measurement consistency across all three phases.

These characteristics directly improve the quality of the current measurements used by the controller, making center-aligned PWM particularly well suited to high-performance current regulation and field-oriented control.

### 5.3 Duty Cycle Generation

Regardless of the selected commutation strategy, the HAL ultimately receives three normalized phase commands corresponding to phases A, B, and C. These values are constrained to the valid operating range before being converted into timer compare values.

Conceptually, the conversion performed by the HAL is

$$
\text{Compare} = D \cdot N_{\text{PWM}},
$$

where

- $D$ is the normalized duty cycle, and
- $N_{\text{PWM}}$ is the timer period expressed in timer counts.

The specific timer configuration is therefore hidden from the controller. Whether the underlying hardware uses a 12-bit timer, 16-bit timer, or an entirely different peripheral, the controller continues to operate using normalized commands while the HAL performs the hardware-specific conversion.

### 5.4 Safe Output Control

An important responsibility of the HAL is ensuring that the inverter never produces unintended phase voltages. During initialization, calibration, idle operation, and fault conditions, PWM outputs are forced into a known safe state before any control algorithm is permitted to command the motor. Similarly, when the controller transitions back to the idle state, the HAL immediately removes commanded actuation and returns the inverter to a non-driving condition.

Centralizing this functionality within the HAL prevents higher-level modules from manipulating gate-driver hardware directly and guarantees that every operating mode enters and exits motor operation using the same well-defined hardware sequence.

### 5.5 Separation from Commutation

The HAL intentionally performs no voltage synthesis or commutation calculations. It does not implement six-step commutation, sinusoidal modulation, Space Vector Modulation (SVM), or any field-oriented control mathematics. Those algorithms determine the desired normalized phase voltages, while the HAL is responsible solely for reproducing those commands accurately using the PWM hardware.

This separation is one of the key architectural decisions within the firmware. By isolating hardware-specific PWM generation from the commutation algorithms, every control strategy shares an identical actuation interface. Consequently, changes to the inverter hardware or timer implementation can be confined to the HAL without requiring modifications to the higher-level motor-control algorithms.

## 6. ADC Measurement System

Accurate electrical measurements form the foundation of every closed-loop control algorithm implemented within the reaction wheel firmware. The controller continuously regulates the motor using measurements acquired by the analog front-end, making the quality, timing, and consistency of those measurements directly responsible for the stability, efficiency, and repeatability of the entire control system. Consequently, the HAL treats the ADC subsystem as a deterministic measurement instrument rather than a peripheral that is sampled opportunistically.

Unlike many embedded applications where analog inputs may be acquired asynchronously, motor-control firmware requires measurements to be taken at precisely defined instants within the PWM cycle. The instantaneous phase currents and phase voltages vary continuously as the inverter switches between conduction states. Sampling immediately following a switching transition introduces measurement error due to switching transients, dead time, reverse recovery, and finite settling of both the power stage and analog front-end. The timing of the measurement is therefore as important as the measurement itself.

### 6.1 Measurement Architecture

The V2 reaction wheel hardware provides a complete electrical representation of the motor through synchronized measurement of the inverter and motor state. The HAL acquires six primary analog measurements during each control cycle:

| Measurement | Physical Quantity | Primary Consumer |
|------------|-------------------|------------------|
| Phase A Current | $I_A$ | Controller / Commutation |
| Phase B Current | $I_B$ | Controller / Commutation |
| Phase C Current | $I_C$ | Controller / Commutation |
| Phase A Voltage | $V_A$ | Controller / Commutation |
| Phase B Voltage | $V_B$ | Controller / Commutation |
| Phase C Voltage | $V_C$ | Controller / Commutation |
| DC Bus Voltage | $V_{BUS}$ | Controller |
| Phase Temperatures | Thermal monitoring | Protection / Telemetry |
| MCU Temperature | Board monitoring | Protection / Telemetry |

Together, these measurements provide complete visibility of both the electrical inputs applied to the motor and the resulting current response. While the present controller primarily utilizes the phase currents, rotor position, and DC bus voltage, the additional phase-voltage measurements significantly expand the capability of the firmware by supporting future observer development, improved motor parameter estimation, enhanced diagnostics, and more advanced control algorithms without requiring architectural changes to the measurement subsystem.

The HAL is responsible for transforming every raw ADC conversion into calibrated engineering quantities before exposing them to the remainder of the firmware. Higher-level modules never operate directly on ADC counts, reference voltages, or peripheral registers. Instead, the controller receives measurements expressed in physical units, allowing the remainder of the firmware to remain independent of ADC resolution, analog gain, and board-specific signal conditioning.

### 6.2 Analog-to-Digital Conversion

Each ADC conversion produces an integer representing the sampled analog input relative to the ADC reference voltage. For an $N$-bit converter,

$$
V_{\mathrm{ADC}}
=
\frac{C}{2^{N}-1}
V_{\mathrm{REF}},
$$

where $C$ is the ADC conversion result, $N$ is the ADC resolution, and $V_{\mathrm{REF}}$ is the reference voltage.

These voltages represent only the outputs of the analog conditioning circuitry and therefore require further conversion before they correspond to meaningful physical quantities. The HAL applies the appropriate scaling, offset compensation, and calibration coefficients for each measurement channel before publishing the resulting engineering values.

For the inline current sensors, the conversion follows

$$
I
=
\frac{V_{\mathrm{ADC}}-V_{\mathrm{offset}}}
{G\,R_{\mathrm{shunt}}},
$$

where $R_{\mathrm{shunt}}$ is the shunt resistance, $G$ is the amplifier gain, and $V_{\mathrm{offset}}$ represents the calibrated zero-current offset of the measurement chain.

Voltage measurements follow an analogous process using the resistor-divider ratio and ADC scaling constants associated with each phase-voltage and DC bus measurement channel. Although the calibration constants differ between measurement types, every analog quantity exposed by the HAL ultimately follows the same conversion pipeline: raw ADC counts are transformed into calibrated engineering units before becoming visible to higher firmware layers.

### 6.3 PWM-Synchronized Sampling

One of the HAL's most important responsibilities is ensuring that every analog measurement is acquired at a deterministic point within the PWM switching cycle.

```text
PWM Period

      ▲                ▲
      │                │
 _____│________________│______

      │<-- Sample -->│
          Center

Switching         Switching
Edge                Edge
```

The firmware synchronizes ADC sampling near the center of the PWM period, where the phase currents and voltages have largely settled following the previous switching event. Sampling during this interval minimizes switching noise and provides the controller with measurements that most accurately represent the instantaneous electrical state of the motor.

This deterministic sampling strategy is particularly important for field-oriented control, where current regulation depends directly upon the measured phase currents. Sampling at inconsistent locations within the PWM cycle introduces apparent current ripple and voltage variation unrelated to the motor itself, degrading controller performance and reducing the repeatability of experimental results. By fixing the sampling instant relative to the PWM waveform, every control iteration observes the motor under nearly identical electrical conditions.

Equally importantly, this synchronization ensures that all commutation strategies—trapezoidal, sinusoidal, and field-oriented control—operate using identical measurements acquired under identical electrical conditions. Experimental differences therefore reflect the behavior of the control algorithms rather than inconsistencies in the measurement process.

### 6.4 Hardware Abstraction

The controller never requests ADC conversions, waits for conversion completion, or accesses ADC registers directly. Those responsibilities remain entirely within the HAL. By the time execution reaches the controller, every measurement has already been converted into calibrated physical quantities with consistent units and synchronized to the current control cycle.

This abstraction isolates the remainder of the firmware from hardware-specific implementation details while allowing future modifications to the analog front-end, ADC configuration, sensing topology, or calibration methodology to remain localized within the HAL. As long as the HAL continues to provide the same physical interface, the controller and commutation algorithms remain independent of the underlying measurement hardware.

## 7. Rotor Position Measurement

Accurate rotor position information is fundamental to every commutation strategy implemented by the firmware. Regardless of whether the motor is operating using trapezoidal commutation, sinusoidal commutation, or field-oriented control, the controller must continuously determine the angular position of the rotor in order to energize the stator windings at the correct electrical angle. The HAL provides the physical measurement interface required to obtain this information while remaining independent of the higher-level algorithms that consume it.

The reaction wheel employs an incremental quadrature encoder mounted directly above the motor shaft. Unlike sensorless control techniques that infer rotor position from measured voltages and currents, the encoder provides an absolute measurement of shaft displacement with high repeatability over the entire operating speed range. This approach was selected to maximize controller performance during both low-speed operation and experimental characterization, where deterministic and repeatable position measurements are considerably more valuable than minimizing sensor count.

### 7.1 Mechanical and Electrical Position

The encoder directly measures the **mechanical rotor angle**, denoted by

$$
\theta_m,
$$

which represents the physical angular position of the rotor shaft. The controller, however, regulates the magnetic field of the motor, requiring knowledge of the **electrical angle**

$$
\theta_e.
$$

These quantities are related through the motor pole-pair count,

$$
\theta_e
=
p\theta_m,
$$

where

- $p$ is the number of pole pairs.

Since electrical angle repeats every electrical revolution rather than every mechanical revolution, the result is wrapped into the interval

$$
0 \le \theta_e < 2\pi.
$$

This conversion forms the interface between the physical motor geometry and the commutation algorithms. Every commutation strategy ultimately operates using electrical angle, regardless of how the rotor position was originally measured.

### 7.2 Encoder Resolution

An incremental quadrature encoder produces a sequence of digital transitions as the rotor rotates. By decoding both encoder channels and detecting each rising and falling edge, the MCU obtains a discrete position count

$$
N_c,
$$

which is converted into mechanical angle according to

$$
\theta_m
=
2\pi
\frac{N_c}{N_{\mathrm{rev}}},
$$

where

- $N_{\mathrm{rev}}$ is the number of encoder counts per mechanical revolution.

In practice, the firmware also applies an electrical offset determined during calibration so that the measured electrical angle aligns with the motor's magnetic reference frame. The calibration procedure establishing this offset is described in the Calibration chapter and is therefore not repeated here.

### 7.3 HAL Responsibilities

The HAL owns the hardware interface to the encoder peripheral, including peripheral initialization, counter configuration, and acquisition of the raw encoder count. It provides the remainder of the firmware with the measured rotor position while abstracting the underlying MCU peripheral implementation.

The HAL intentionally does **not** perform higher-level estimation or control-related processing. Position observers, velocity estimation, phase-locked loops, filtering, and any derived rotor-state quantities remain outside the scope of the hardware abstraction layer. Their implementation belongs to the Encoder subsystem, which transforms the raw position measurements supplied by the HAL into the continuous rotor-state estimates required by the controller.

This separation is deliberate. The encoder hardware represents a physical sensor, while estimation algorithms represent mathematical models operating on that sensor data. Keeping these responsibilities independent allows improvements to the observer implementation without modifying the hardware interface, and likewise permits future encoder hardware changes without affecting the estimation algorithms.

### 7.4 Deterministic Sampling

Rotor position is sampled synchronously with the control loop so that every controller iteration operates using a self-consistent snapshot of the motor state. Position, current, voltage, and other measured quantities therefore correspond to approximately the same instant in time, minimizing inconsistencies that could otherwise arise if different measurements were acquired asynchronously.

Maintaining this temporal alignment is particularly important for field-oriented control, where the measured phase currents are transformed into the rotating reference frame using the current electrical angle. Any significant delay or inconsistency between the position measurement and current acquisition introduces errors into the coordinate transformations and directly degrades current regulation performance.

By synchronizing encoder acquisition with the remainder of the measurement pipeline, the HAL provides the controller with a coherent representation of the motor state each time the 16 kHz control loop executes.

## 7.5 Velocity Estimation

While the encoder directly measures rotor position, it does not measure rotor velocity. Every commutation strategy implemented by the firmware, however, requires an estimate of the rotor speed. Trapezoidal commutation uses velocity for closed-loop speed regulation, while sinusoidal and field-oriented control additionally require a continuous estimate of the rotor state to produce smooth electrical excitation. The HAL therefore derives rotor velocity from the measured encoder position before exposing it to the remainder of the firmware.

A straightforward approach is to differentiate successive encoder measurements,

$$
\omega
=
\frac{\Delta\theta}{\Delta t},
$$

where $\Delta\theta$ is the change in measured position over the sampling interval $\Delta t$. Although mathematically correct, this approach performs poorly in practice.

Incremental encoders produce discrete position updates rather than continuous measurements. At low rotational speeds, several control iterations may occur before the encoder count changes, causing the estimated velocity to alternate between zero and large instantaneous values. At higher speeds, encoder quantization and measurement noise are amplified by differentiation, resulting in a noisy velocity estimate. Since differentiation inherently amplifies high-frequency noise, directly differentiating encoder position is generally unsuitable for high-performance motor control.

Rather than differentiating the encoder measurements directly, the firmware estimates rotor velocity using a digital Phase-Locked Loop (PLL) observer.

### 7.6 Phase-Locked Loop Observer

The PLL observer maintains a continuously updated mathematical model of the rotor position and velocity. During each control iteration, the observer predicts where the rotor should be based on its current velocity estimate. The measured encoder position is then compared against this prediction, and any resulting error is used to correct both the estimated position and estimated velocity.

Instead of treating each encoder measurement independently, the observer continuously tracks the rotor, producing a smooth estimate of its motion even though the encoder itself provides only discrete position updates.

The observer therefore behaves as a digital tracking system rather than a numerical differentiator.

### 7.7 Prediction Model

The observer maintains two internal state variables:

- Estimated rotor position, $\hat{\theta}$
- Estimated rotor velocity, $\hat{\omega}$

Assuming constant velocity over a single control period, the predicted rotor position for the next iteration is

$$
\hat{\theta}_{k+1}
=
\hat{\theta}_k
+
T_s\hat{\omega}_k,
$$

where

- $T_s$ is the control-loop sampling period,
- $\hat{\theta}_k$ is the current estimated position, and
- $\hat{\omega}_k$ is the current estimated velocity.

Physically, this prediction answers a simple question:

> *If the rotor continues rotating at its current estimated speed, where should it be one control period from now?*

This prediction forms the observer's internal model of the rotor motion.

### 7.8 Phase Error

After the next encoder measurement is acquired, the observer compares the measured rotor position with the predicted position. The resulting phase error is

$$
e_{\theta}
=
\theta
-
\hat{\theta},
$$

where

- $\theta$ is the measured encoder position, and
- $\hat{\theta}$ is the predicted position.

Since angular position wraps every revolution, the phase error is normalized to remain within the principal angular interval. This prevents discontinuities when the rotor crosses the $0^\circ$ or $360^\circ$ boundary, ensuring that the observer always computes the smallest angular difference between the measured and predicted positions.

### 7.9 PI Correction

The phase error is used to correct both observer states through a proportional-integral feedback structure.

The proportional correction immediately adjusts the estimated rotor position,

$$
\hat{\theta}
\leftarrow
\hat{\theta}
+
K_p e_{\theta},
$$

while the integral correction updates the estimated velocity,

$$
\hat{\omega}
\leftarrow
\hat{\omega}
+
K_i e_{\theta},
$$

where

- $K_p$ is the proportional gain, and
- $K_i$ is the integral gain.

These corrections allow the observer to continuously converge toward the measured encoder position. The proportional term provides an immediate correction whenever the prediction deviates from the measured position, while the integral term gradually adjusts the estimated velocity so that future predictions remain aligned with the rotor.

Over successive control iterations, the observer naturally converges to the true rotor motion while rejecting much of the quantization noise that would otherwise appear in a differentiated velocity estimate.

### 7.10 Locking Behavior

The name *Phase-Locked Loop* originates from the observer's ability to continuously synchronize its internal model with the measured rotor position.

If the predicted position begins to lag behind the measured position, the phase error increases. The proportional and integral corrections respond by increasing the estimated velocity, causing future predictions to advance more rapidly. Conversely, if the prediction moves ahead of the measured rotor position, the phase error reverses sign, reducing the estimated velocity until the prediction once again aligns with the measurement.

As the observer converges, the phase error approaches zero and the internal model remains locked to the physical rotor. Rather than repeatedly calculating velocity from individual encoder measurements, the firmware continuously tracks the rotor motion, producing smooth estimates of both position and velocity throughout operation.

```text
             Encoder Position
                    │
                    ▼
             Measured Angle
                    │
                    ▼
            Phase Difference
        (Measured - Estimated)
                    │
          ┌─────────┴─────────┐
          ▼                   ▼
     Proportional         Integral
       Correction         Correction
          │                   │
          ▼                   ▼
Estimated Position     Estimated Velocity
          ▲                   │
          └──────────┬────────┘
                     │
               Position Predictor
                     │
                     └──────► Next Sample
```

### 7.11 Engineering Considerations

The PLL observer was selected because it provides a deterministic and computationally efficient estimate of rotor velocity while avoiding the shortcomings of direct numerical differentiation. Its fixed computational cost makes it well suited to the 16 kHz real-time control loop, and its continuous tracking behavior produces smooth velocity estimates over the full operating speed range.

Compared with direct differentiation, the PLL significantly reduces quantization noise at low speeds, provides improved immunity to encoder measurement noise, and produces a continuously varying velocity estimate that is particularly beneficial for field-oriented control.

Like any observer, however, its performance depends upon appropriate gain selection. Excessive observer bandwidth increases sensitivity to measurement noise, while insufficient bandwidth introduces lag between the estimated and actual rotor motion. Proper tuning therefore represents a tradeoff between responsiveness and noise rejection, allowing the observer to provide accurate rotor-state estimates while maintaining stable closed-loop operation.

## 8. Runtime Operation

Following initialization, the HAL operates continuously as part of the deterministic 16 kHz control loop. Each iteration of the loop acquires a complete snapshot of the motor state, exposes the resulting measurements to the controller, accepts new actuator commands generated by the selected commutation algorithm, and updates the inverter outputs in preparation for the next control period. Every execution follows the same sequence of operations, ensuring deterministic execution time and repeatable control behavior.

Conceptually, a single control iteration follows the sequence illustrated below.

```text
                  Control Cycle (62.5 μs)

      ┌─────────────────────────────────────────────┐
      │                                             │
      │  Acquire Measurements                       │
      │      │                                      │
      │      ▼                                      │
      │  Convert to Engineering Units               │
      │      │                                      │
      │      ▼                                      │
      │  Update Rotor Measurements                  │
      │      │                                      │
      │      ▼                                      │
      │  Controller Executes                        │
      │      │                                      │
      │      ▼                                      │
      │  Commutation Algorithm                      │
      │      │                                      │
      │      ▼                                      │
      │  Apply New PWM Outputs                      │
      │                                             │
      └─────────────────────────────────────────────┘
```

Although individual implementation details differ between hardware peripherals, the execution order remains fixed throughout runtime. This deterministic sequencing ensures that every controller iteration operates on a coherent set of measurements acquired from approximately the same instant in time before producing the corresponding inverter commands.

### 8.1 Measurement Pipeline

At the beginning of each control period, the HAL acquires the latest synchronized measurements from the analog front-end and encoder hardware. Raw peripheral values are immediately converted into calibrated engineering quantities before becoming visible to higher firmware layers. This guarantees that every consumer of the HAL receives measurements expressed in consistent physical units rather than device-specific register values.

Because every measurement originates from the same control period, the controller operates on a temporally consistent representation of the motor state. Phase currents, phase voltages, DC bus voltage, rotor position, and temperature measurements therefore correspond to essentially the same electrical operating point, eliminating inconsistencies that could arise if different quantities were sampled independently.

### 8.2 Control Interaction

Once the measurement pipeline has completed, the controller consumes the updated motor state and computes the desired electrical actuation for the current operating mode. The HAL itself performs no interpretation of these measurements and does not participate in the control calculations. Its role is limited to supplying accurate physical measurements and reproducing the actuator commands generated by the controller and commutation modules.

The controller therefore remains entirely independent of the underlying MCU peripherals. Whether measurements originate from one ADC, multiple synchronized ADCs, or a future hardware revision employing different sensing hardware, the controller continues to operate on the same physical interface presented by the HAL.

### 8.3 Output Update

Following completion of the control calculations, the commutation subsystem generates normalized phase commands representing the desired inverter output. The HAL converts these commands into hardware-specific PWM compare values and updates the inverter at the appropriate timer synchronization point.

Updating the PWM outputs at deterministic instants prevents partially updated switching states and ensures that every control iteration produces exactly one coherent inverter update. This behavior is particularly important for high-bandwidth current regulation, where inconsistent output timing can introduce additional phase delay into the control loop.

### 8.4 Deterministic Execution

A fundamental design objective of the firmware is that every control iteration executes with predictable timing. Consequently, the HAL avoids any operation capable of introducing unbounded execution latency within the real-time control path.

The runtime implementation therefore avoids:

- Dynamic memory allocation
- Blocking peripheral transactions
- Operating system synchronization primitives
- UART communication
- File-system access
- Non-deterministic background processing

Similarly, computationally expensive operations that are not directly required for motor control—such as telemetry transmission, experiment management, and host communication—are intentionally executed outside the real-time control path. This separation ensures that the execution time of the control loop remains effectively independent of communication activity and experimental data logging.

Maintaining deterministic execution is essential for the repeatability of the experimental results presented throughout this project. Because every commutation strategy executes under identical timing conditions, measured differences in efficiency, ripple, dynamic response, and thermal performance can be attributed to the control algorithms themselves rather than variations in firmware execution.

### 8.5 Interaction with Higher-Level Modules

The HAL occupies the lowest layer of the motor-control software stack and therefore serves as the common interface for every higher-level subsystem. The controller, commutation algorithms, calibration routines, telemetry system, and experimental framework all ultimately depend on the physical measurements and actuator interfaces provided by the HAL.

Despite this central role, the HAL remains intentionally passive. It neither schedules control algorithms nor determines operating modes. Instead, it continuously provides deterministic hardware services while higher-level modules determine how those services are used. This strict separation of responsibilities significantly reduces coupling between firmware components and allows the hardware interface to remain stable as the controller and commutation algorithms continue to evolve.

## 9. Hardware Initialization

Before closed-loop motor control can begin, every hardware peripheral must be configured into a known and deterministic operating state. Initialization establishes the communication between the MCU and the external hardware, configures the measurement and actuation peripherals, applies hardware-specific calibration parameters, and ensures that the inverter remains in a safe state until the controller is ready to assume operation.

Unlike general embedded applications where peripherals may be initialized as needed, the reaction wheel firmware performs hardware initialization in a carefully defined sequence. Many peripherals depend upon others already being configured, and incorrect initialization ordering can lead to invalid measurements, unstable control, or unintended inverter outputs. The HAL therefore centralizes the entire initialization process to guarantee that every subsystem begins operation from a consistent and repeatable state.

### 9.1 Initialization Sequence

Conceptually, the HAL performs initialization in the following order:

```text
Power-On Reset
       │
       ▼
Initialize MCU Hardware
       │
       ▼
Configure GPIO
       │
       ▼
Initialize Timers
       │
       ▼
Initialize PWM
       │
       ▼
Initialize ADC
       │
       ▼
Initialize Encoder
       │
       ▼
Load Calibration Data
       │
       ▼
Enable Measurement Pipeline
       │
       ▼
HAL Ready
```

The exact implementation differs between hardware peripherals, but the logical ordering remains consistent. Each stage establishes the prerequisites required by the following stage while ensuring that the motor remains electrically inactive until initialization has completed successfully.

### 9.2 GPIO Configuration

The initialization process begins by configuring the MCU pins for their intended hardware functions. Depending upon the peripheral, pins may be configured for analog inputs, timer outputs, encoder interfaces, communication peripherals, or general-purpose digital I/O.

Performing this configuration first ensures that all subsequent peripherals communicate with the external hardware through correctly configured electrical interfaces before any active measurements or switching occur.

### 9.3 Timer and PWM Configuration

The timer subsystem is initialized before the inverter outputs are enabled. This establishes the PWM frequency, operating mode, synchronization behavior, and update timing required by the control loop.

During this stage, the PWM outputs remain disabled or forced into a safe state. Although the timer hardware begins operating internally, no switching signals are presented to the gate driver until the controller explicitly commands motor operation. This prevents unintended energization of the motor during startup.

### 9.4 ADC Configuration

Once the PWM timing has been established, the analog measurement subsystem is configured. This includes initialization of the ADC peripherals, configuration of the measurement channels, and synchronization of the conversion timing with the PWM cycle.

Establishing the PWM timing before configuring ADC synchronization ensures that all subsequent current and voltage measurements are acquired at deterministic locations within each switching period.

### 9.5 Encoder Initialization

The encoder interface is initialized after the measurement hardware has been configured. During this stage, the quadrature decoder is configured, the position counter is reset to a known state, and the encoder peripheral begins tracking rotor motion.

Although valid position measurements become immediately available, the electrical angle used by the controller is not fully established until the calibration constants have been applied.

### 9.6 Calibration Loading

Following peripheral initialization, the HAL loads the calibration parameters required to convert raw hardware measurements into engineering units.

These calibration parameters include measurement offsets, scaling coefficients, electrical angle alignment, and other hardware-specific constants determined during system calibration. Applying these parameters before normal operation ensures that every measurement exposed to the controller represents a calibrated physical quantity rather than an uncorrected hardware measurement.

The calibration methodology itself is described in the Calibration chapter and is therefore not repeated here.

### 9.7 Transition to Runtime Operation

Once all peripherals have been initialized and the calibration parameters have been applied, the HAL enters its normal runtime state. At this point, synchronized measurements become available to the controller, while the inverter remains in a safe, non-driving condition until commanded by the system state machine.

Separating hardware initialization from controller execution ensures that the controller never operates on incomplete or uninitialized hardware data. Every control iteration therefore begins with a fully configured measurement and actuation subsystem, providing the deterministic execution environment required for reliable motor control and repeatable experimental evaluation.

## 10. Fault Handling and Safe States

Motor-control firmware must assume that abnormal operating conditions can occur at any point during execution. Hardware failures, invalid sensor measurements, communication errors, or controller faults all have the potential to produce incorrect inverter commands. Since the inverter is capable of rapidly delivering significant electrical power to the motor, the firmware must ensure that any detected fault immediately places the hardware into a predictable and electrically safe operating state.

Within the software architecture, the HAL serves as the final authority over the physical hardware. Regardless of which higher-level module detects a fault, the HAL is responsible for ensuring that the inverter transitions into a non-driving condition and that no further actuator commands are applied until normal operation has been explicitly restored.

### 10.1 Safe-State Philosophy

The primary objective of the HAL during fault conditions is not to recover the controller or diagnose the failure. Instead, its responsibility is considerably simpler: remove electrical actuation from the motor while preserving sufficient hardware functionality to support fault reporting, diagnostics, and recovery.

Conceptually, a fault transitions the hardware through the following sequence.

```text
Fault Detected
       │
       ▼
Reject New Commands
       │
       ▼
Disable PWM Outputs
       │
       ▼
Motor No Longer Driven
       │
       ▼
Remain in Safe State
```

By centralizing this behavior within the HAL, every subsystem interacts with the inverter through a common safety mechanism. The controller, telemetry system, calibration routines, and experimental framework therefore do not require independent implementations of hardware shutdown procedures.

### 10.2 PWM Shutdown

The most important action performed during a fault is the immediate removal of inverter actuation. The HAL forces the PWM outputs into their predefined safe state, preventing further switching activity regardless of the previously commanded duty cycles.

Disabling the PWM outputs prevents additional electrical energy from being delivered to the motor while allowing the remainder of the firmware to continue executing. Diagnostic information can therefore continue to be collected and reported without risking unintended motor operation.

### 10.3 Measurement Availability

Although inverter actuation is removed during a fault, the measurement subsystem may continue operating. Encoder measurements, voltage measurements, current measurements, and temperature monitoring remain valuable for determining the cause of the fault and for evaluating the state of the system after shutdown.

Separating measurement availability from motor actuation allows the firmware to retain visibility of the hardware while ensuring that no additional torque is generated.

### 10.4 Controlled Recovery

Returning to normal operation requires an explicit transition out of the fault state. The HAL does not automatically resume inverter operation simply because a fault condition disappears. Instead, higher-level firmware is responsible for determining when recovery is appropriate and for issuing the commands required to restart the controller.

This approach prevents repeated fault-recovery cycles that could otherwise occur if an intermittent hardware problem repeatedly triggered and cleared within a short period of time.

### 10.5 Architectural Separation

The HAL intentionally does not determine whether a fault has occurred or how individual faults should be handled. Those responsibilities belong to the higher-level controller, protection logic, and system state machine.

Instead, the HAL provides the hardware mechanisms required to safely execute the requested transition. Once instructed to enter a safe state, it guarantees that the inverter hardware is placed into a known, non-driving configuration while continuing to provide the hardware services necessary for diagnostics and subsequent recovery.

Maintaining this separation of responsibilities significantly simplifies the overall firmware architecture. Fault detection remains a system-level responsibility, while safe manipulation of the physical hardware remains exclusively within the HAL.

## 11. Source Code Organization

The HAL is organized as a collection of cohesive modules, each responsible for managing a specific portion of the underlying hardware. Rather than concentrating all peripheral access within a single source file, the implementation separates functionality according to the hardware resources being abstracted. This organization reduces coupling between unrelated peripherals, simplifies maintenance, and allows individual hardware interfaces to evolve independently while preserving a consistent interface to the remainder of the firmware.

Each HAL module owns the complete interaction with its associated peripheral, including initialization, configuration, runtime access, and hardware-specific implementation details. Higher-level firmware therefore interacts exclusively through the interfaces exposed by the HAL without requiring knowledge of the underlying MCU registers or peripheral configuration.

### 11.1 Module Responsibilities

Conceptually, the HAL can be divided into several functional components.

| Module | Primary Responsibility |
|---------|------------------------|
| GPIO | Pin configuration and digital I/O |
| PWM | Three-phase inverter output generation |
| ADC | Acquisition of current, voltage, and temperature measurements |
| Encoder | Rotor position and velocity acquisition |
| Calibration Interface | Application of hardware calibration constants |
| Hardware Utilities | Device-specific support functions |

Although implemented as separate software components, these modules cooperate to provide a single hardware abstraction layer to the remainder of the firmware.

### 11.2 Layered Architecture

The HAL occupies the lowest software layer that is directly visible to the remainder of the firmware.

```text
Application
      │
      ▼
Controller
      │
      ▼
Commutation
      │
      ▼
HAL
      │
      ▼
MCU Peripherals
      │
      ▼
Physical Hardware
```

This layered architecture establishes a clear separation of responsibilities. Control algorithms operate entirely on physical measurements and normalized actuator commands, while the HAL manages the details of peripheral configuration, register access, synchronization, and hardware timing.

As a result, modifications to the underlying hardware rarely propagate beyond the HAL, provided that the external interface remains unchanged.

### 11.3 Information Flow

During runtime, information flows through the HAL in two distinct directions.

Measurements propagate upward from the physical hardware toward the controller,

```text
Sensors
    │
    ▼
HAL
    │
    ▼
Controller
```

while actuator commands propagate downward from the controller toward the inverter,

```text
Controller
     │
     ▼
HAL
     │
     ▼
PWM Hardware
     │
     ▼
Motor
```

This bidirectional structure clearly separates sensing from actuation while ensuring that every interaction with the physical hardware passes through a common abstraction layer.

### 11.4 Dependency Management

The HAL intentionally minimizes dependencies on higher-level firmware modules. It neither performs control calculations nor manages application state. Instead, it provides deterministic hardware services that are consumed by the controller, commutation algorithms, telemetry system, calibration routines, and experimental framework.

Conversely, higher-level modules remain independent of MCU-specific implementation details. They neither access peripheral registers nor manipulate hardware directly. This separation significantly reduces software coupling and allows each module to evolve independently while preserving a stable interface between hardware and control software.

### 11.5 Maintainability

Organizing the HAL into dedicated hardware modules simplifies long-term maintenance of the firmware. New sensors, revised hardware interfaces, or future MCU platforms can be incorporated by modifying the corresponding HAL implementation without requiring widespread changes throughout the control software.

This modular organization also supports the experimental objectives of the project. Because every commutation strategy interacts with the same hardware interface, experimental comparisons remain independent of the underlying peripheral implementation. Improvements to the hardware abstraction therefore benefit every control algorithm simultaneously while preserving identical operating conditions across all experimental evaluations.

## 12. Engineering Decisions

The architecture of the Hardware Abstraction Layer was driven by two primary objectives: providing a deterministic execution environment for high-performance motor control while maintaining a clean separation between hardware-specific implementation details and the control algorithms themselves. Every significant design decision within the HAL was evaluated against these objectives.

### 12.1 Deterministic Execution

Real-time motor control requires predictable execution timing. Variations in sampling, computation, or output timing directly influence controller performance and can introduce differences between experimental results that are unrelated to the control algorithms being evaluated.

For this reason, the HAL was designed so that every control iteration performs the same sequence of operations using the same hardware interfaces and synchronization points. Measurements are acquired at deterministic locations within the PWM cycle, converted into engineering units before becoming visible to higher-level software, and used to produce a single synchronized inverter update each control period.

Maintaining deterministic execution was particularly important for this project because three independent commutation strategies were evaluated on identical hardware. Ensuring that each controller operated under the same measurement and timing conditions allowed differences in experimental performance to be attributed to the commutation algorithms rather than inconsistencies within the firmware infrastructure.

### 12.2 Hardware Independence

The controller and commutation algorithms operate exclusively on physical quantities such as current, voltage, position, and velocity rather than peripheral registers or raw ADC values. Consequently, higher-level firmware remains largely independent of the underlying hardware implementation.

This abstraction significantly simplifies future hardware revisions. Changes to ADC resolution, timer peripherals, encoder interfaces, or analog signal conditioning can generally be implemented within the HAL while preserving the interfaces consumed by the remainder of the firmware.

The transition from the original hardware platform to the V2 reaction wheel hardware illustrates this design philosophy. Although the newer hardware introduced additional sensing capability—including independent phase-voltage measurements and expanded thermal monitoring—the interfaces presented to the controller remained fundamentally unchanged. The controller therefore continued to operate on the same physical quantities while the HAL absorbed the hardware-specific implementation differences.

### 12.3 Single Ownership of Hardware

Each hardware peripheral is owned exclusively by the HAL. Higher-level modules neither configure peripherals nor access hardware registers directly. This ownership model establishes a single authoritative interface for every physical resource within the system.

Restricting direct hardware access provides several benefits:

- Peripheral configuration remains consistent throughout the firmware.
- Hardware modifications are localized to a single software layer.
- Timing behavior remains predictable.
- Debugging is simplified by eliminating competing hardware access paths.
- Hardware safety mechanisms remain centralized.

This philosophy also reduces coupling between software modules, allowing each subsystem to focus exclusively on its intended responsibility.

### 12.4 Consistent Measurement Interface

One of the most important responsibilities of the HAL is presenting every measurement using consistent engineering units. The controller never operates on ADC counts, timer values, encoder counts, or hardware-specific scaling factors. Instead, every quantity is expressed using meaningful physical units before it becomes visible outside the HAL.

This greatly improves readability throughout the remainder of the firmware while reducing the likelihood of scaling errors. More importantly, it establishes a stable interface that remains valid even if the underlying hardware implementation changes.

### 12.5 Separation of Measurement and Control

The HAL intentionally performs no control calculations. It neither determines operating modes nor computes actuator commands. Instead, it acquires measurements from the hardware, converts them into calibrated physical quantities, and reproduces the commands generated by the higher-level controller.

Maintaining this separation provides a clear architectural boundary between hardware interaction and control logic. Improvements to control algorithms therefore do not require modifications to the hardware abstraction, while hardware revisions can generally be accommodated without altering the controller implementation.

This separation was particularly valuable during development of the multiple commutation strategies evaluated throughout this project. Trapezoidal commutation, sinusoidal commutation, and field-oriented control all operated using the same measurement pipeline and actuator interface, allowing meaningful experimental comparisons while minimizing duplicated firmware.

### 12.6 Chapter Summary

The Hardware Abstraction Layer forms the foundation upon which the remainder of the reaction wheel firmware is constructed. By encapsulating every hardware-specific operation behind a deterministic and well-defined interface, it provides the controller with synchronized measurements, calibrated engineering quantities, and reliable actuator control while isolating higher-level software from the details of the underlying hardware implementation.

The following chapter builds upon this foundation by examining the control architecture itself. Whereas the HAL is responsible for observing and actuating the physical system, the Controller determines how those measurements are interpreted to regulate the motor and achieve the desired operating behavior.