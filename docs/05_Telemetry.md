# Telemetry

## 1. Introduction

The telemetry subsystem provides the communication interface between the embedded reaction wheel controller and the external host computer used throughout system development, experimental testing, and performance evaluation.

Although all control algorithms execute entirely on the embedded controller, experimental characterization requires continuous observation of the controller state, motor operating conditions, and experiment progress. The telemetry subsystem was therefore developed to export real-time measurements without compromising the deterministic execution of the 16 kHz control loop. Rather than functioning as a debugging utility, telemetry forms an integral component of the overall experimental framework by providing a stable communication interface between the embedded firmware and the host-side data acquisition software.

Unlike conventional serial debugging implementations that periodically print variables over a UART interface, the telemetry subsystem was designed specifically for repeatable automated experimentation. Measurements are collected directly from the Hardware Abstraction Layer, combined with controller and experiment metadata, assembled into a fixed binary packet, and transmitted asynchronously to the host computer. This architecture provides a deterministic communication path while minimizing bandwidth requirements and simplifying host-side parsing.

A key design objective of the telemetry subsystem is the complete separation of measurement acquisition from physical communication. The deterministic control loop is responsible only for constructing telemetry packets at fixed intervals, while UART transmission is performed independently through an asynchronous driver and packet queue. By separating these responsibilities, communication latency cannot delay controller execution, preserving the deterministic timing required for reliable motor control and repeatable experimental measurements.

Beyond simple measurement transmission, the telemetry subsystem also provides experiment synchronization, firmware status reporting, measurement validity information, packet integrity verification, and compatibility between multiple hardware revisions. These capabilities allow the host software to coordinate automated experiments while maintaining a consistent communication interface across firmware revisions and hardware generations.

## 2. Design Objectives

The telemetry subsystem was designed to satisfy two competing requirements. On one hand, the experimental evaluation required continuous access to motor measurements, controller state, and experiment progress. On the other hand, the communication system could not be permitted to influence the deterministic execution of the real-time controller.

Several design objectives therefore guided the development of the telemetry architecture.

The first objective was to preserve deterministic controller execution. The reaction wheel controller operates at a fixed update rate of 16 kHz, leaving only a small and predictable computation budget during each control cycle. Consequently, telemetry generation was designed to perform only bounded operations within the real-time loop while avoiding any blocking communication or variable execution latency.

The second objective was to establish a stable communication contract between the embedded firmware and the host-side data acquisition software. Rather than transmitting human-readable text, telemetry is encoded within a fixed binary packet containing version information, measurement data, experiment metadata, validity flags, and checksum protection. Maintaining a stable packet structure simplifies host-side software while allowing future firmware revisions to extend the protocol without disrupting existing analysis tools.

Another important objective was to support automated experimentation. The telemetry subsystem therefore includes experiment lifecycle information, fault reporting, and trigger synchronization in addition to conventional measurement data. These metadata allow the host computer to coordinate firmware-controlled experiments without requiring knowledge of the controller's internal implementation.

Finally, the telemetry protocol was designed to support multiple hardware revisions through explicit measurement validity flags. Rather than changing the packet format as additional sensors become available, every packet retains the same overall structure while indicating which measurements are valid for the current hardware configuration. This approach maintains compatibility between hardware revisions while avoiding unnecessary complexity within the host-side software.

## 3. Telemetry Architecture

The telemetry subsystem is positioned between the embedded controller and the external host computer, providing a deterministic communication pathway through which controller measurements are exported for monitoring, data acquisition, and post-processing. Rather than interacting directly with the control algorithms, telemetry observes the current system state, packages the required measurements into a standardized binary format, and transfers those packets asynchronously to the host computer. This separation allows the controller and telemetry subsystems to evolve independently while maintaining a stable communication interface.

At the highest level, telemetry performs four primary functions.

1. Acquire measurements from the Hardware Abstraction Layer.
2. Combine measurements with controller and experiment metadata.
3. Assemble a fixed binary telemetry packet.
4. Queue the completed packet for asynchronous UART transmission.

Conceptually, the telemetry data flow is illustrated below.

```text
                 Hardware Abstraction Layer
            (Sensors and Controller Measurements)
                           │
                           ▼
                 Telemetry_Update16kHz()
                           │
          ┌────────────────┴────────────────┐
          │                                 │
          ▼                                 ▼
  Read Latest Measurements        Read Experiment Metadata
          │                                 │
          └────────────────┬────────────────┘
                           ▼
                Build Telemetry Packet
                           │
                           ▼
              Packet Transmission Queue
                           │
                           ▼
            Asynchronous UART Driver
                           │
                           ▼
                  Host Computer (Python)
                           │
                           ▼
            Logging, Analysis, and Graphing
```

Measurements used to construct each telemetry packet are obtained directly from the Hardware Abstraction Layer. These include electrical quantities such as DC bus voltage and phase currents, mechanical quantities including rotor velocity, and available thermal measurements. The telemetry subsystem does not perform any sensor conversion or signal processing itself; instead, it simply retrieves the latest validated measurements already maintained by the Hardware Abstraction Layer.

In addition to physical measurements, each packet incorporates metadata describing the current operating state of the firmware. Information such as the commanded speed, selected commutation strategy, active experiment state, fault status, and measurement validity flags provides the host computer with the contextual information required to correctly interpret the accompanying sensor data. By packaging both measurements and controller state within a single packet, every transmitted sample becomes self-describing, eliminating the need for the host software to infer controller operation from external timing assumptions.

A defining characteristic of the telemetry architecture is the separation between packet construction and packet transmission. Packet generation occurs deterministically within the 16 kHz control loop using only bounded operations, while UART communication is performed asynchronously using an independent transmission queue and callback-driven driver. Consequently, the execution time of the controller remains independent of UART bandwidth or transmission latency, preserving the deterministic timing requirements of the reaction wheel control system.

## 4. Telemetry Packet Format

The telemetry packet defines the communication contract between the embedded firmware and the host-side data acquisition software. Every telemetry transmission conforms to this fixed binary structure, allowing the host computer to decode incoming measurements without requiring knowledge of the controller implementation or the currently executing experiment. By maintaining a stable packet definition, the telemetry subsystem provides a consistent interface that remains independent of controller revisions, commutation strategies, and experimental procedures.

Rather than transmitting measurements individually, all information associated with a single sampling instant is assembled into one packed binary packet. Each packet contains a synchronization identifier, protocol version, timestamp, measured operating quantities, experiment metadata, validity information, and checksum protection. Grouping all related measurements within a single packet ensures that every transmitted sample represents a complete and internally consistent snapshot of the controller state.

Conceptually, the packet organization is illustrated below.

```text
+------------------------------------------------------------+
| Magic Identifier                                            |
+------------------------------------------------------------+
| Packet Version                                              |
+------------------------------------------------------------+
| Timestamp                                                   |
+------------------------------------------------------------+
| Electrical Measurements                                     |
|  • Bus Voltage                                              |
|  • Phase Currents                                           |
|  • Rotor Velocity                                           |
|  • Speed Command                                            |
+------------------------------------------------------------+
| Optional V2 Measurements                                    |
|  • Phase Voltages                                           |
|  • Phase Temperatures                                       |
|  • MCU Temperature                                          |
+------------------------------------------------------------+
| Experiment Metadata                                         |
|  • Test State                                               |
|  • Commutation Mode                                         |
|  • Experiment Status                                        |
|  • Fault Code                                               |
+------------------------------------------------------------+
| Measurement Validity Flags                                  |
+------------------------------------------------------------+
| Checksum                                                    |
+------------------------------------------------------------+
```

The packet begins with a fixed synchronization value and packet version number. These fields allow the host software to reliably identify the beginning of each packet while simultaneously ensuring that both the firmware and host software interpret the packet using the same protocol definition. Future revisions of the telemetry protocol can therefore introduce additional measurements without sacrificing backward compatibility.

A timestamp accompanies every transmitted packet, allowing measurements acquired by the embedded controller to be accurately aligned with external instrumentation such as the Joulescope power analyzer and LabJack data acquisition system. Since all experimental data are referenced to the controller timestamp rather than host reception time, variations in serial communication latency do not influence the temporal alignment of the recorded measurements.

The majority of the packet consists of measurements describing the instantaneous operating condition of the reaction wheel. These include the DC bus voltage, three phase currents, measured rotor velocity, commanded speed, and available thermal information. The packet structure also reserves space for additional measurements introduced by future hardware revisions, including individual phase voltages and per-phase temperature sensing, while maintaining compatibility with earlier hardware through explicit validity flags.

Beyond physical measurements, each packet includes metadata describing the current state of the firmware. Information such as the active experiment state, selected commutation strategy, experiment lifecycle status, and fault code allows the host software to interpret the accompanying measurements without maintaining additional communication channels. Consequently, every telemetry packet represents not only the physical state of the reaction wheel but also the operational state of the embedded controller.

Finally, each packet concludes with a simple checksum calculated over the packet contents. Although lightweight, this checksum provides basic protection against communication errors by allowing the host software to detect corrupted packets before they are incorporated into the recorded experimental data.

## 5. Experiment Control

In addition to transmitting physical measurements, the telemetry subsystem communicates the execution state of firmware-controlled experiments. This capability allows the host computer to synchronize with the embedded controller without relying on external timing assumptions or manually defined experiment durations.

Rather than treating telemetry solely as a stream of sensor measurements, each telemetry packet also contains metadata describing the current operating state of the firmware. These metadata include the active experiment state, selected commutation strategy, overall experiment status, and fault condition. Together, these fields allow the host software to determine not only *what* the controller is measuring, but also *what the controller is currently doing*.

The overall experiment lifecycle is represented using four well-defined states:

```text
             ┌─────────────┐
             │    IDLE     │
             └──────┬──────┘
                    │ Trigger
                    ▼
             ┌─────────────┐
             │   RUNNING   │
             └───┬─────┬───┘
                 │     │
      Complete   │     │ Fault
                 ▼     ▼
          ┌─────────┐ ┌─────────┐
          │COMPLETE │ │  FAULT  │
          └─────────┘ └─────────┘
```

Prior to the start of an experiment, the firmware reports the **IDLE** state while awaiting a trigger command from the host computer. Once the requested experiment begins, the telemetry status transitions to **RUNNING**, indicating that controller measurements should be recorded and associated with an active test. Successful completion of the experiment is reported using the **COMPLETE** status, while abnormal termination results in the **FAULT** state together with an accompanying fault code describing the reason for termination.

This firmware-controlled lifecycle significantly simplifies the host-side data acquisition software. Rather than estimating experiment duration using timers or detecting completion from changes in measured speed, the Python acquisition scripts simply monitor the experiment status field contained within each telemetry packet. Data acquisition therefore begins and ends under firmware control, ensuring that every recorded experiment precisely corresponds to the execution of the embedded controller.

Each telemetry packet also communicates the current test state and selected commutation strategy. These additional metadata allow a single telemetry protocol to support multiple experimental procedures—including power characterization, torque-speed testing, step response evaluation, and thermal soak experiments—without requiring different packet formats or experiment-specific communication protocols.

When abnormal operating conditions occur, the telemetry subsystem reports a standardized fault code together with the transition to the **FAULT** experiment status. This common fault reporting mechanism provides the host computer with immediate notification that an experiment terminated prematurely while allowing individual test implementations to map their local failure conditions into a common set of telemetry fault codes. As a result, host-side software can respond consistently to failures without requiring detailed knowledge of the internal implementation of each experiment.

## 6. Non-Blocking UART Transport

The telemetry subsystem was designed to ensure that communication never interferes with the deterministic execution of the reaction wheel controller. To achieve this objective, packet generation and UART transmission are implemented as two completely independent operations.

During execution of the 16 kHz control loop, the telemetry subsystem performs only three bounded operations:

1. Poll for a host trigger byte.
2. Construct a telemetry packet when the configured decimation interval expires.
3. Insert the completed packet into a transmission queue.

No UART transmission is performed within the real-time control loop. Instead, all communication with the UART peripheral occurs asynchronously through the Zephyr UART driver after the packet has been placed into the transmission queue.

Conceptually, the execution model is illustrated below.

```text
          16 kHz Control Loop
                 │
                 ▼
        Build Telemetry Packet
                 │
                 ▼
     Single-Producer Queue
                 │
        (Control Loop Ends)
                 │
─────────────────┼──────────────────────────
                 │
                 ▼
      UART Driver Callback
                 │
                 ▼
     Single-Consumer Queue
                 │
                 ▼
          UART Transmission
                 │
                 ▼
           Host Computer
```

The transmission queue is implemented as a fixed-size ring buffer using a single-producer, single-consumer architecture. The deterministic control loop acts as the sole producer by inserting completed telemetry packets into the queue, while the asynchronous UART callback acts as the sole consumer by removing packets after successful transmission. Because ownership of the queue is clearly divided between one producer and one consumer, queue management remains both efficient and predictable while requiring only minimal synchronization to protect shared indices.

Once a packet has been inserted into the queue, the telemetry subsystem immediately returns control to the remainder of the controller. If the UART peripheral is idle, transmission begins automatically. Otherwise, the packet simply remains in the queue until the UART driver reports that the previous transmission has completed. The callback associated with the asynchronous UART driver is therefore responsible for advancing the queue and initiating subsequent transmissions without any intervention from the real-time control loop.

This producer-consumer architecture offers several important advantages. Most importantly, the execution time of the control loop becomes independent of UART bandwidth, packet length, or transmission latency. Packet generation therefore requires a predictable amount of computation regardless of whether the UART is actively transmitting, waiting for hardware completion, or temporarily operating at reduced throughput.

To prevent communication delays from propagating into the controller, the transmission queue also serves as a temporary buffer between telemetry generation and physical communication. Under normal operating conditions, packets are removed from the queue at approximately the same rate that they are generated. Should transmission temporarily fall behind, packets accumulate within the queue until bandwidth becomes available. Only if the queue reaches capacity are additional packets discarded, with the firmware maintaining a counter that records the number of dropped packets for diagnostic purposes. This behavior preserves deterministic controller execution even under abnormal communication conditions by sacrificing telemetry continuity rather than control-loop timing.

An additional benefit of this architecture is that it cleanly separates communication transport from telemetry generation. The telemetry subsystem is responsible only for constructing valid packets, while the UART driver is responsible only for delivering those packets to the host computer. This separation improves software modularity, simplifies maintenance, and allows future communication interfaces to be introduced without modifying the deterministic telemetry generation logic.

## 7. Runtime Operation

The telemetry subsystem executes continuously alongside the reaction wheel controller, operating as a deterministic component of the normal 16 kHz control loop. During each controller iteration, telemetry performs a small and predictable amount of work before immediately returning control to the remainder of the firmware. This execution model ensures that telemetry remains synchronized with the controller while preserving the fixed execution time required for deterministic motor control.

At the beginning of each control cycle, the telemetry subsystem performs a non-blocking poll of the UART receive interface to determine whether the host computer has transmitted the experiment trigger byte. This operation requires only a lightweight UART poll and immediately returns if no data are available, allowing the controller to continue execution without delay. When the trigger byte is detected, an internal trigger event is recorded for later consumption by the experiment controller.

If telemetry transmission is currently enabled, the subsystem advances an internal decimation counter. Rather than transmitting measurements during every controller iteration, packets are generated only after a fixed number of control cycles have elapsed. This deterministic decimation mechanism reduces UART bandwidth while maintaining a constant telemetry sampling frequency that remains synchronized with the controller update rate.

Whenever the decimation interval expires, the telemetry subsystem constructs a complete telemetry packet representing the current operating state of the reaction wheel. Measurements are obtained directly from the Hardware Abstraction Layer, experiment metadata are incorporated from the telemetry state, validity flags are generated according to the active hardware configuration, and a checksum is computed over the completed packet before transmission. The resulting packet therefore represents a complete snapshot of both the physical system and controller state at a single instant in time.

Once packet construction has completed, the packet is copied into the transmission queue and the real-time control loop immediately resumes execution. No attempt is made to wait for UART availability or to transmit serial data during controller execution. If the UART peripheral is already idle, transmission begins automatically; otherwise, the packet remains buffered until the asynchronous UART callback becomes available to initiate the next transfer.

The complete runtime operation of the telemetry subsystem is summarized in Figure X.

```text
                16 kHz Controller Update
                         │
                         ▼
              Poll UART for Trigger Byte
                         │
                         ▼
                Telemetry Enabled?
                  │              │
                 No             Yes
                  │              ▼
                  │     Increment Decimation Counter
                  │              │
                  │              ▼
                  │    Decimation Interval Reached?
                  │         │               │
                  │        No              Yes
                  │         │               ▼
                  │         │      Read HAL Measurements
                  │         │               │
                  │         │               ▼
                  │         │      Build Telemetry Packet
                  │         │               │
                  │         │               ▼
                  │         │      Queue Packet
                  │         │               │
                  └─────────┴───────────────┘
                                  │
                                  ▼
                     Continue Controller Execution

             (Asynchronous UART callback transmits queued packets)
```

By limiting telemetry activity within the control loop to bounded operations consisting of trigger polling, decimation, packet construction, and queue insertion, the runtime execution time remains both predictable and independent of serial communication latency. All variable-duration operations associated with physical data transmission occur asynchronously after the control loop has completed, preserving the deterministic execution characteristics required by the reaction wheel controller.

## 8. Source Code Organization

The telemetry subsystem is implemented using a compact two-module architecture consisting of a public interface definition and a corresponding implementation. This organization separates the communication protocol from the runtime implementation while providing a stable interface to the remainder of the firmware.

The public interface is defined within **Telemetry.h**, which serves as the communication contract between the telemetry subsystem and the remainder of the firmware. This file defines the binary packet structure, packet version, synchronization constants, experiment lifecycle states, standardized fault codes, measurement validity flags, and the public application programming interface (API) used by the controller and experiment framework. By centralizing these definitions within a single header, both the firmware and host-side software operate from a consistent protocol specification.

The runtime implementation is contained within **Telemetry.c**. This module performs UART initialization, trigger detection, packet construction, checksum generation, queue management, asynchronous UART transmission, and maintenance of experiment metadata. In addition, it implements the deterministic 16 kHz update routine responsible for constructing telemetry packets while preserving the execution requirements of the control loop.

Measurements included within each telemetry packet are obtained from the Hardware Abstraction Layer rather than directly from hardware peripherals. Consequently, the telemetry subsystem remains independent of individual sensors, analog-to-digital conversion routines, and controller implementation details. Its responsibility is limited to packaging already available measurements into a standardized communication format before transferring those packets to the transport layer.

Similarly, experiment metadata originate from the controller and experiment framework through a small collection of public setter functions. The telemetry subsystem therefore maintains no knowledge of individual experiment implementations beyond the metadata explicitly supplied through its public interface. This loose coupling allows new experiments to be incorporated without requiring modifications to the telemetry transport architecture.

Overall, the telemetry subsystem exhibits high cohesion and low coupling. The public interface defines *what* information is communicated, while the implementation defines *how* that information is transmitted. Measurements, controller behavior, hardware access, and experiment execution remain the responsibility of their respective subsystems, allowing telemetry to function as an independent communication layer within the overall reaction wheel software architecture.

## 9. Engineering Decisions

Several engineering decisions guided the design of the telemetry subsystem, each intended to support deterministic controller execution while providing a robust communication interface for automated experimentation.

The most significant design decision was the complete separation of telemetry generation from UART transmission. Communication peripherals inherently operate at timescales that are several orders of magnitude slower than the 16 kHz control loop, making direct serial transmission unsuitable for deterministic motor control. By restricting the real-time control loop to packet construction and queue insertion while delegating all physical communication to an asynchronous UART driver, the execution time of the controller becomes independent of serial communication latency. This separation preserves deterministic controller timing while still allowing continuous measurement streaming throughout every experiment.

A second design decision was to define telemetry as a fixed binary communication protocol rather than a stream of formatted text. Human-readable serial output is useful during early debugging but introduces unnecessary bandwidth overhead, variable packet lengths, and increased parsing complexity. By adopting a fixed packet structure with explicitly defined fields, every telemetry transmission occupies a constant number of bytes, simplifying host-side decoding while reducing communication overhead. The addition of packet version information further allows future revisions of the protocol to remain compatible with existing host software.

The telemetry protocol was also designed to transmit experiment metadata alongside physical measurements. Rather than requiring separate communication channels for controller status and measurement data, every packet includes information describing the current experiment state, selected commutation strategy, experiment lifecycle status, and active fault condition. This approach transforms telemetry from a passive measurement stream into a self-describing communication protocol capable of coordinating fully automated experiments between the embedded firmware and the host computer.

Another important design decision was the introduction of explicit measurement validity flags. As additional sensing capabilities become available in future hardware revisions, maintaining a stable packet structure avoids unnecessary modifications to host-side software. Instead of altering the packet definition for each hardware revision, every packet communicates which measurements are valid for the current hardware configuration. This approach preserves compatibility between hardware generations while allowing the telemetry protocol to evolve without breaking existing data acquisition tools.

Finally, telemetry sampling was synchronized with the controller rather than implemented as an independent communication task. Packet generation occurs through deterministic decimation of the 16 kHz control loop, ensuring that every telemetry sample corresponds to a well-defined controller update. This guarantees consistent temporal spacing between measurements and eliminates sampling jitter that could otherwise arise from independent operating system scheduling or timer-driven communication tasks. As a result, recorded telemetry accurately reflects the behavior of the embedded controller throughout every experiment.

## 10. Chapter Summary

This chapter presented the telemetry subsystem developed for the reaction wheel controller. Unlike conventional serial debugging interfaces, the telemetry subsystem was designed as a deterministic communication architecture capable of supporting automated experimental evaluation without influencing the execution of the real-time motor controller.

A fixed binary telemetry protocol was introduced to communicate controller measurements, experiment metadata, and system status between the embedded firmware and the host computer. By defining a stable packet format incorporating version information, validity flags, and checksum protection, the telemetry subsystem provides a consistent communication interface that remains compatible across multiple hardware revisions and experimental configurations.

To preserve deterministic controller execution, telemetry generation was intentionally separated from physical UART transmission. The 16 kHz control loop performs only bounded packet construction and queue insertion, while an asynchronous UART driver independently manages packet transport through a single-producer, single-consumer transmission queue. This architecture ensures that communication latency cannot influence controller timing while still providing continuous real-time measurement streaming throughout every experiment.

The telemetry subsystem therefore serves as the communication foundation for the experimental framework presented in this thesis. By providing synchronized, self-describing measurements together with firmware-controlled experiment status, it enables reliable data acquisition, automated experiment execution, and repeatable post-processing of controller performance.

The following chapters build upon this communication framework to describe the experimental methodology, host-side data acquisition software, and performance evaluation used to characterize the reaction wheel controller.