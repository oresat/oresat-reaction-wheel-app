# Commutation

## 1. Introduction

The Controller described in the previous chapter determines the electrical effort required to achieve the commanded motor speed. The output of the Controller, however, cannot be applied directly to a brushless DC motor.

Unlike a brushed DC motor, where mechanical brushes automatically switch current between the rotor windings as the shaft rotates, a brushless DC motor contains stationary stator windings and a permanent-magnet rotor. Consequently, the inverter must actively determine which windings are energized, when they are energized, and by how much in order to continuously generate torque.

This process is known as **commutation**.

Within the reaction wheel firmware, the Commutation subsystem forms the interface between high-level control decisions and low-level inverter actuation. It receives the electrical command produced by the Controller together with the measured electrical state of the motor and converts these quantities into the pulse-width modulation (PWM) signals required to drive the three-phase inverter.

Conceptually, the relationship between the Controller and the Commutation subsystem is illustrated below.

```text
          Hardware Abstraction Layer
                    │
        Rotor Position, Current,
      Electrical Angle, Bus Voltage
                    │
                    ▼
              Controller
                    │
      Electrical Command
                    │
                    ▼
            Commutation
                    │
     Three PWM Duty Cycles
                    │
                    ▼
          Three-Phase Inverter
                    │
                    ▼
             Brushless Motor
```

The Controller determines **how much** electrical effort is required to achieve the desired operating condition. The Commutation subsystem determines **how** that electrical effort is physically applied to the motor.

This separation is a fundamental architectural feature of the firmware. By defining a common interface between the Controller and the Commutation subsystem, multiple commutation algorithms can be implemented without modifying the closed-loop controller. During experimental evaluation, the same controller therefore operates with trapezoidal commutation, sinusoidal commutation, and field-oriented control under identical operating conditions, allowing meaningful comparisons between the individual commutation strategies. This architectural boundary is reflected directly in the firmware through a common strategy interface shared by every commutation implementation.

Although each commutation strategy ultimately produces the same output—a set of three PWM duty cycles—the mathematical operations required to generate those duty cycles differ substantially. Trapezoidal commutation energizes discrete winding combinations according to the rotor electrical sector, sinusoidal commutation generates continuously varying phase voltages, and field-oriented control performs multiple coordinate transformations before synthesizing the final inverter commands. Despite these differences, all strategies share the same objective: producing the rotating magnetic field required to generate torque within the motor.

Understanding how these electrical commands are transformed into three-phase inverter actuation requires first examining the quantities used to describe the motor itself. The following section introduces the reference frames used throughout modern brushless motor control and establishes the mathematical foundation upon which the remainder of this chapter is built.

## 2. Design Objectives

The Commutation subsystem is responsible for converting the abstract electrical commands produced by the Controller into the pulse-width modulation signals required to drive the three-phase inverter. While this objective appears straightforward, the implementation must satisfy several competing design requirements.

First, the subsystem must operate deterministically within the fixed 16 kHz control period established by the firmware architecture. Every commutation strategy must complete its calculations within a single control iteration while producing repeatable timing independent of the selected algorithm.

Second, the subsystem must remain independent of the closed-loop Controller. The Controller determines the desired electrical effort but has no knowledge of how that effort is translated into inverter switching patterns. Likewise, the Commutation subsystem receives the Controller's electrical command without knowledge of the operating mode or higher-level control decisions. This separation allows the controller-to-commutation interface to remain unchanged regardless of the active commutation strategy.

Third, every commutation strategy must operate through a common software interface. Regardless of whether trapezoidal commutation, sinusoidal commutation, open-loop vector control, or field-oriented control is active, each implementation receives the same controller-generated input quantities and produces the same unified PWM command. This architectural consistency simplifies strategy selection while ensuring that experimental comparisons evaluate only the commutation algorithm rather than differences elsewhere in the firmware.

Finally, the implementation must preserve a clear separation between mathematical processing and hardware control. Coordinate transformations, voltage synthesis, and modulation are performed entirely within the Commutation subsystem, while direct interaction with the PWM peripherals and gate driver remains the responsibility of the Hardware Abstraction Layer. This separation improves modularity, simplifies software maintenance, and allows mathematical algorithms to evolve independently of the underlying hardware implementation.

Collectively, these objectives produce a Commutation subsystem that is deterministic, modular, interchangeable, and independent of both the Controller and the hardware peripherals. These design principles provide the foundation upon which each commutation strategy is implemented.

## 3. Reference Frames

The physical quantities within a brushless DC motor are continuously changing as the rotor rotates. Phase currents, phase voltages, and magnetic fields all vary with electrical position, making the motor inherently a dynamic system.

To analyze and control this behavior, it is useful to describe the motor using different coordinate systems, known as **reference frames**. Each reference frame represents the same physical motor but expresses its electrical quantities in a form that is more convenient for a particular task.

No information is created or lost when transforming between reference frames. Instead, the same physical quantities are represented using different mathematical coordinates. Choosing an appropriate reference frame can dramatically simplify both analysis and control.

Throughout this chapter, three reference frames are used:

- the three-phase **ABC** reference frame,
- the stationary **αβ (alpha-beta)** reference frame, and
- the rotating **dq (direct-quadrature)** reference frame.

Each provides a different perspective of the same electrical system.

The relationship between these reference frames is illustrated below.

```text
        Three-Phase Motor
              │
              ▼
      ABC Reference Frame
              │
      Clarke Transform
              │
              ▼
      αβ Reference Frame
              │
       Park Transform
              │
              ▼
      dq Reference Frame
```

The reverse transformations are equally important. Once the required electrical command has been calculated in the rotating reference frame, it must be transformed back into the stationary reference frame before the inverter can generate the corresponding three-phase voltages.

Consequently, every field-oriented control cycle continuously transforms electrical quantities between multiple reference frames before producing the final PWM commands. The firmware implements these mathematical transformations as reusable BLDC-specific utilities that are shared across the commutation subsystem.

The following sections examine each reference frame individually before introducing the mathematical transformations that connect them.

### 3.1 Three-Phase (ABC) Reference Frame

The three-phase, or **ABC**, reference frame is the natural coordinate system of the brushless motor because it directly corresponds to the physical stator windings.

Each phase winding carries its own voltage and current, producing three independent electrical quantities that vary continuously with rotor position. During normal operation, these quantities are approximately sinusoidal and separated by 120 electrical degrees.

Conceptually, the three stator phases may be represented as

```text
           Phase A
              ▲
             / \
            /   \
           /     \
          ●-------●
      Phase C   Phase B
```

The electrical quantities associated with these windings are commonly written as

$$
\begin{aligned}
i_A(t) &= I\cos(\theta)\\
i_B(t) &= I\cos\left(\theta-\frac{2\pi}{3}\right)\\
i_C(t) &= I\cos\left(\theta+\frac{2\pi}{3}\right)
\end{aligned}
$$

where

- $i_A,i_B,i_C$ are the three phase currents,
- $I$ is the current magnitude, and
- $\theta$ is the electrical rotor angle.

An equivalent relationship exists for the phase voltages.

These three quantities completely describe the electrical state of the stator and correspond directly to the physical currents measured by the Hardware Abstraction Layer. Likewise, the PWM duty cycles generated by the Commutation subsystem ultimately drive these same three windings.

Although the ABC reference frame closely matches the physical motor, it is not the most convenient representation for control. Every electrical quantity varies continuously with rotor position, causing even steady operating conditions to appear as three continuously oscillating waveforms.

As a result, many control algorithms become unnecessarily complicated when expressed directly in the three-phase reference frame. Rather than regulating three sinusoidal quantities simultaneously, it is preferable to transform them into a coordinate system that more naturally represents the rotating magnetic field.

The first step toward achieving this simplification is the stationary αβ reference frame introduced in the following section.

### 3.2 Stationary αβ Reference Frame

Although the three-phase reference frame accurately describes the electrical state of the motor, it contains redundant information. For a balanced three-phase system,

$$
i_A+i_B+i_C=0,
$$

meaning that only two of the three phase currents are independent. Once any two currents are known, the third is uniquely determined.

This observation allows the three-phase system to be represented using only two orthogonal components while preserving the complete electrical state of the motor.

The **Clarke Transform** performs this conversion by projecting the three phase quantities onto two stationary axes, conventionally labeled α (alpha) and β (beta). Rather than describing three separate winding currents, the transformed quantities describe a single vector within a fixed two-dimensional coordinate system.

The transformation used throughout this work is

$$
\begin{aligned}
i_\alpha &= i_A,\\
i_\beta &= \frac{1}{\sqrt{3}}\left(i_B-i_C\right).
\end{aligned}
$$

This implementation assumes balanced three-phase currents and is identical to that used throughout the firmware's BLDC mathematics library.

Rather than viewing the stator as three independent windings, the αβ reference frame represents the electrical state as a single vector,

$$
\mathbf{i}_{\alpha\beta}
=
\begin{bmatrix}
i_\alpha\\
i_\beta
\end{bmatrix},
$$

whose magnitude and direction vary continuously with the rotor position.

Conceptually, the transformation may be visualized as

```text
              β
              ▲
              │
          •   │
      ↗  │    │
   iαβ    │   │
          │   │
──────────┼──────────► α
          │
```

Instead of tracking three sinusoidal waveforms independently, the controller now observes a single vector rotating within a stationary coordinate system.

This representation provides several important advantages.

First, the mathematical complexity of the system is reduced by eliminating the redundant third phase variable while preserving the complete electrical state of the motor.

Second, the αβ reference frame provides a natural geometric interpretation of the rotating magnetic field. The direction of the vector represents the instantaneous electrical angle, while its magnitude corresponds to the overall electrical excitation.

Finally, the stationary αβ frame forms the basis for modern voltage synthesis techniques. Both open-loop vector control and field-oriented control ultimately generate voltage vectors within this coordinate system before converting those vectors into three PWM duty cycles through Space Vector Modulation (SVM). The firmware reflects this architecture by implementing the inverse Park transformation followed by SVM when synthesizing inverter commands.

Although the αβ reference frame greatly simplifies the representation of the motor, the electrical quantities remain time-varying because the coordinate system itself is stationary while the rotor continues to rotate. Consequently, even steady operating conditions still appear as continuously rotating vectors.

A further simplification can be achieved by allowing the reference frame itself to rotate with the rotor.

### 3.3 Rotating dq Reference Frame

The stationary αβ reference frame represents the motor as a rotating vector, greatly simplifying the original three-phase system. However, the vector itself continues to rotate at the electrical rotor speed.

From the perspective of the stationary reference frame, even perfectly steady motor operation appears as continuously changing sinusoidal quantities.

Suppose, however, that the observer rotates together with the rotor.

Instead of standing outside the motor and watching the magnetic field rotate, imagine sitting on the rotor and observing the electrical quantities from the rotor's own perspective.

The rotating magnetic field would no longer appear to rotate.

It would instead appear nearly stationary.

This simple change of perspective forms the basis of the **Park Transform**.

Rather than using stationary α and β axes, the Park Transform rotates the coordinate system by the electrical rotor angle,

$$
\theta_e,
$$

producing two new orthogonal axes:

- the **direct (d)** axis, aligned with the rotor magnetic field, and
- the **quadrature (q)** axis, perpendicular to the rotor magnetic field.

Conceptually,

```text
Stationary Frame               Rotating Frame

        β                           q
        ▲                           ▲
      ↗ │                           │
     ●  │                     ●─────►
        │                           d
────────┼────► α
```

The rotating reference frame therefore moves together with the rotor rather than remaining fixed in space.

As a consequence, quantities that previously appeared as rotating sinusoids become nearly constant during steady-state operation. This transformation converts an inherently time-varying control problem into one that closely resembles the regulation of two independent DC quantities.

This simplification is one of the primary reasons that field-oriented control has become the dominant control method for high-performance brushless motors. Rather than attempting to regulate three continuously varying phase currents, the controller independently regulates two approximately constant current components aligned with physically meaningful directions.

The mathematical transformation required to perform this coordinate rotation is introduced in the following section.

### 3.4 Park Transform

The Park Transform rotates the stationary αβ reference frame into the rotating dq reference frame by the instantaneous electrical rotor angle. Mathematically, this operation is simply a two-dimensional coordinate rotation.

Given the stationary current vector

$$
\mathbf{i}_{\alpha\beta}
=
\begin{bmatrix}
i_\alpha\\
i_\beta
\end{bmatrix},
$$

and the electrical rotor angle,

$$
\theta_e,
$$

the transformed current vector becomes

$$
\mathbf{i}_{dq}
=
\begin{bmatrix}
i_d\\
i_q
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_e & \sin\theta_e\\
-\sin\theta_e & \cos\theta_e
\end{bmatrix}
\begin{bmatrix}
i_\alpha\\
i_\beta
\end{bmatrix}.
$$

Expanding the matrix multiplication yields

$$
\begin{aligned}
i_d &= i_\alpha\cos\theta_e+i_\beta\sin\theta_e,\\
i_q &= -i_\alpha\sin\theta_e+i_\beta\cos\theta_e.
\end{aligned}
$$

These equations are implemented directly within the firmware's shared BLDC mathematics library, where the required sine and cosine values are first evaluated before performing the coordinate rotation.

Although the transformation itself is mathematically straightforward, its physical interpretation is significantly more important.

The **d-axis**, or direct axis, is aligned with the permanent-magnet flux generated by the rotor.

The **q-axis**, or quadrature axis, is oriented ninety electrical degrees ahead of the rotor and is therefore perpendicular to the magnetic field.

```text
                 q-axis
                   ▲
                   │
                   │
         Torque    │
         Current   │
                   │
Rotor Flux ────────●──────► d-axis
                   │
```

Because the d-axis points directly toward the rotor magnetic field, current flowing along this axis primarily influences the magnetic flux within the machine. Conversely, current flowing along the q-axis acts perpendicular to the rotor flux and therefore produces electromagnetic torque.

For surface-mounted permanent-magnet motors such as the reaction wheel motor used in this work, maximum torque is generally achieved by maintaining

$$
i_d \approx 0,
$$

while regulating

$$
i_q
$$

to produce the desired torque.

This separation represents one of the greatest advantages of the rotating reference frame. Rather than controlling three coupled sinusoidal phase currents, the controller regulates two nearly independent physical quantities:

- **d-axis current**, which primarily controls magnetic flux, and
- **q-axis current**, which primarily controls torque production.

As a result, the current regulation problem becomes mathematically similar to controlling two DC signals using conventional PI controllers.

This simplification forms the foundation of modern field-oriented control and explains why the dq reference frame is used throughout the remainder of this chapter.

### 3.5 Inverse Park Transform

Although current regulation is greatly simplified within the rotating dq reference frame, the inverter cannot directly generate d-axis or q-axis voltages.

Instead, the inverter produces three phase voltages that are physically applied to the stator windings.

Consequently, any voltage command calculated within the rotating reference frame must first be transformed back into the stationary reference frame before pulse-width modulation can be generated.

This operation is performed using the **Inverse Park Transform**.

Beginning with the commanded voltage vector

$$
\mathbf{v}_{dq}
=
\begin{bmatrix}
v_d\\
v_q
\end{bmatrix},
$$

the corresponding stationary voltage vector becomes

$$
\mathbf{v}_{\alpha\beta}
=
\begin{bmatrix}
v_\alpha\\
v_\beta
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_e & -\sin\theta_e\\
\sin\theta_e & \cos\theta_e
\end{bmatrix}
\begin{bmatrix}
v_d\\
v_q
\end{bmatrix}.
$$

Expanding the matrix multiplication gives

$$
\begin{aligned}
v_\alpha &= v_d\cos\theta_e-v_q\sin\theta_e,\\
v_\beta &= v_d\sin\theta_e+v_q\cos\theta_e.
\end{aligned}
$$

These equations are likewise implemented within the shared BLDC mathematics library and represent the inverse operation of the Park Transform.

After this transformation, the commanded voltage once again exists within the stationary αβ reference frame. Rather than representing currents aligned with the rotor, the resulting vector now describes the stationary stator voltage that must be synthesized by the inverter.

At this stage, the desired voltage has been fully determined. The remaining task is to generate PWM duty cycles capable of producing this voltage using the available DC bus.

This final conversion is performed by the pulse-width modulation algorithm.

## 4. Voltage Synthesis

The previous sections established how electrical quantities can be transformed between the ABC, αβ, and dq reference frames. After the current controller determines the required d-axis and q-axis voltages, and these voltages are transformed back into the stationary αβ reference frame, the desired stator voltage is completely defined.

The inverter, however, cannot directly generate an arbitrary voltage vector.

A conventional three-phase inverter consists of three half-bridges connected to a DC supply. Each half-bridge can connect its corresponding motor phase either to the positive supply rail or to ground. Consequently, the inverter can produce only a finite number of switching states.

The purpose of the voltage synthesis stage is therefore to approximate the continuously varying voltage vector calculated by the controller using the discrete switching states available to the inverter.

This conversion forms the final stage of the Commutation subsystem before the resulting duty cycles are transmitted to the Hardware Abstraction Layer for PWM generation.

### 4.1 Three-Phase Inverter

The reaction wheel motor is driven by a conventional three-phase voltage source inverter consisting of three complementary half-bridges.

Each motor phase may be connected to either the positive DC bus or ground depending on the switching state of the corresponding transistor pair.

Conceptually, the inverter may be represented as

```text
           +VDC
             │
      ┌──────┼──────┐
      │      │      │
     SA     SB     SC
      │      │      │
      A      B      C
       \     |     /
        \    |    /
         \   |   /
          Brushless
             Motor
         /   |   \
        /    |    \
       /     |     \
      │      │      │
     SA'    SB'    SC'
      │      │      │
      └──────┼──────┘
             │
            GND
```

Each phase leg contains two complementary switches that are never allowed to conduct simultaneously. Instead, pulse-width modulation determines the fraction of each switching period that the upper and lower switches remain active, thereby controlling the average phase voltage applied to the motor.

Since each phase has two possible switching states, the complete inverter possesses

$$
2^3=8
$$

possible switching combinations.

Six of these switching states generate non-zero voltage vectors, while the remaining two produce zero output voltage.

These eight switching states form the basis of Space Vector Modulation.

### 4.2 Space Voltage Vectors

Every inverter switching state produces a unique combination of phase voltages.

When expressed within the stationary αβ reference frame, each switching state corresponds to a voltage vector pointing in a particular direction.

Rather than thinking about six independent transistor switches, it is therefore more useful to consider the inverter as a device capable of generating a finite set of voltage vectors.

These vectors are illustrated conceptually below.

```text
                V2
                ●
             /     \
          V3         V1

        ●               ●

      V4                 V0,V7

        ●               ●

          V5         V6
             \     /
                ●
```

More commonly, these vectors are represented as a regular hexagon.

```text
                     V2
                     ●
                  /     \
             V3 ●         ● V1
                |         |
                |    •    |
                |         |
             V4 ●         ● V0
                  \     /
                     ●
                     V5
```

The six active vectors possess equal magnitude and are separated by 60 electrical degrees.

The remaining two switching states produce the zero vector located at the origin.

Together, these eight vectors completely describe every voltage that the inverter can generate instantaneously.

### 4.3 Principle of Space Vector Modulation

Although the inverter can generate only eight discrete voltage vectors, the motor requires a continuously rotating magnetic field.

Space Vector Modulation achieves this by rapidly switching between adjacent active vectors and the zero vector during each PWM period.

Rather than generating the desired voltage directly, the inverter produces an average voltage over one switching interval.

Suppose the desired voltage vector lies between two adjacent active vectors.

```text
            V2
            ●
           /|
          / |
         /  |
        ●---×
      V3    VREF
```

Instead of attempting to generate $V_{\mathrm{REF}}$ directly, the inverter applies

- the first active vector for a duration $T_1$,
- the second active vector for a duration $T_2$, and
- the zero vector for the remaining interval $T_0$.

These durations satisfy

$$
T_s=T_1+T_2+T_0,
$$

where $T_s$ is the PWM switching period.

The average voltage produced over one switching interval therefore becomes

$$
\mathbf{V}_{ref}
=
\frac{T_1}{T_s}\mathbf{V}_1
+
\frac{T_2}{T_s}\mathbf{V}_2
+
\frac{T_0}{T_s}\mathbf{V}_0.
$$

Although the inverter never generates the desired voltage instantaneously, the average voltage observed by the motor closely approximates the commanded reference vector.

Because the PWM switching frequency is significantly higher than the electrical bandwidth of the motor, the phase inductance naturally filters these switching transitions, causing the motor to respond primarily to the average voltage rather than the individual switching events.

### 4.4 PWM Duty Cycle Generation

Once the active vector durations have been determined, the remaining task is to convert these switching intervals into PWM duty cycles for the three inverter phases.

The Space Vector Modulation algorithm first determines the 60-degree sector containing the commanded voltage vector. Using the sector number together with the calculated vector durations, the algorithm computes three normalized duty cycles corresponding to phases A, B, and C.

These duty cycles represent the fraction of the PWM period during which each upper transistor remains enabled.

Finally, the resulting duty cycles are transmitted to the Hardware Abstraction Layer, where they are converted into timer compare values and applied to the PWM peripherals driving the inverter.

Within the firmware, this functionality is encapsulated by the shared Space Vector Modulation routine, allowing every vector-based commutation strategy to synthesize inverter voltages using a common implementation.

At this point, the commanded electrical voltage has been completely transformed into three synchronized PWM signals capable of producing the desired rotating magnetic field within the motor.

The mathematical framework developed throughout this chapter forms the common foundation upon which each individual commutation strategy is implemented. The following section examines how these common mathematical tools are employed by the four commutation strategies supported by the reaction wheel firmware.

# 5. Commutation Strategies

Although every commutation strategy shares the same objective—producing the rotating magnetic field required to generate motor torque—they differ significantly in the mathematical methods used to generate the inverter switching commands.

From the perspective of the firmware architecture, each strategy implements a common software interface while encapsulating its own control algorithm. Regardless of the selected strategy, the Controller provides a common set of electrical commands, and the Commutation subsystem ultimately produces three PWM duty cycles for the Hardware Abstraction Layer. This modular design allows commutation algorithms to be exchanged without affecting the remainder of the control system.

The reaction wheel firmware supports four distinct commutation strategies:

- Open-Loop Vector Control
- Trapezoidal Commutation
- Sinusoidal Commutation
- Field-Oriented Control

Each represents a progressively more sophisticated approach to generating the rotating magnetic field required by the brushless motor.

The following sections describe these strategies in increasing order of mathematical complexity.

### 5.1 Open-Loop Vector Control

Open-loop vector control was implemented primarily as a development and validation tool rather than as an operational control strategy.

Unlike closed-loop field-oriented control, open-loop vector control does not regulate motor current or estimate electromagnetic torque. Instead, the electrical angle is supplied externally while the commanded d-axis and q-axis voltages are applied directly to the motor.

Because no current feedback is required, the algorithm bypasses the current regulation stage entirely. The commanded voltage vector is transformed directly from the rotating dq reference frame into the stationary αβ frame using the Inverse Park Transform before being synthesized by the Space Vector Modulation algorithm.

The resulting control path is therefore considerably simpler than that of field-oriented control.

```text
Voltage Command
       │
       ▼
Inverse Park
       │
       ▼
 αβ Voltage
       │
       ▼
     SVM
       │
       ▼
 PWM Duty Cycles
```

Despite its simplicity, open-loop vector control proved valuable throughout firmware development. By eliminating the current control loop, encoder feedback processing, and associated compensation terms, individual mathematical components such as the coordinate transformations and Space Vector Modulation implementation could be verified independently.

Consequently, open-loop vector control served primarily as an engineering validation tool used during subsystem integration rather than as a strategy intended for reaction wheel operation.

### 5.2 Trapezoidal Commutation

Trapezoidal commutation is the simplest closed-loop commutation strategy implemented within the reaction wheel firmware and remains one of the most widely used control methods for brushless DC motors due to its computational efficiency and straightforward implementation.

Rather than continuously regulating the stator magnetic field, trapezoidal commutation energizes only two of the three stator windings at any instant while allowing the remaining phase to float.

The active winding pair changes every 60 electrical degrees according to the measured electrical rotor position, producing six distinct commutation sectors during one electrical revolution.

Conceptually, the six commutation states are

```text
Sector   Active Phases

  1         A+  B-
  2         A+  C-
  3         B+  C-
  4         B+  A-
  5         C+  A-
  6         C+  B-
```

As the rotor advances through each electrical sector, the energized winding pair changes to maintain torque production.

Unlike vector-based control methods, trapezoidal commutation operates directly within the three-phase ABC reference frame. No Clarke Transform, Park Transform, or Space Vector Modulation is required. Instead, the measured electrical angle determines the active commutation sector, and predefined switching patterns generate the corresponding inverter commands.

The principal advantages of trapezoidal commutation include its low computational complexity, deterministic execution time, and minimal mathematical requirements. These characteristics make it particularly attractive for embedded systems with limited computational resources.

However, the discrete nature of the six commutation sectors also introduces abrupt transitions in the generated magnetic field. These discontinuities produce increased torque ripple, greater acoustic noise, and reduced electrical efficiency compared with continuously varying commutation strategies.

For these reasons, trapezoidal commutation serves as an important performance baseline throughout the experimental evaluation presented later in this thesis.

### 5.3 Sinusoidal Commutation

Sinusoidal commutation represents an intermediate approach between trapezoidal commutation and full field-oriented control.

Rather than energizing only two stator windings at a time, sinusoidal commutation continuously drives all three phases using sinusoidal voltage waveforms separated by 120 electrical degrees. The resulting magnetic field rotates smoothly around the stator, significantly reducing the torque ripple associated with six-step commutation.

The commanded phase voltages may be expressed as

$$
\begin{aligned}
v_A &= V\cos(\theta_e),\\
v_B &= V\cos\left(\theta_e-\frac{2\pi}{3}\right),\\
v_C &= V\cos\left(\theta_e+\frac{2\pi}{3}\right),
\end{aligned}
$$

where $V$ represents the commanded voltage magnitude and $\theta_e$ is the measured electrical rotor angle.

Unlike field-oriented control, sinusoidal commutation does not transform measured currents into the rotating dq reference frame. Instead, the desired sinusoidal phase voltages are generated directly within the three-phase reference frame and subsequently converted into PWM duty cycles.

Because the magnetic field varies continuously rather than changing abruptly every 60 electrical degrees, sinusoidal commutation generally produces smoother torque, lower vibration, and reduced acoustic noise compared with trapezoidal commutation.

However, the strategy still lacks direct regulation of the d-axis and q-axis currents. Consequently, it cannot independently control magnetic flux and torque, limiting both dynamic performance and overall electrical efficiency.

Sinusoidal commutation therefore occupies an intermediate position between the simplicity of trapezoidal control and the sophistication of full field-oriented control.

### 5.4 Field-Oriented Control

Field-Oriented Control (FOC) is the most sophisticated commutation strategy implemented within the reaction wheel firmware and serves as the primary control method investigated throughout this work.

Unlike trapezoidal and sinusoidal commutation, which operate directly within the three-phase reference frame, FOC continuously estimates the electrical state of the motor, transforms the measured quantities into the rotating dq reference frame, independently regulates magnetic flux and torque, and synthesizes the resulting voltage vector using Space Vector Modulation.

The principal objective of FOC is to transform the inherently coupled three-phase motor into a system that behaves similarly to two independent DC control loops. This transformation allows magnetic flux and torque to be regulated separately, improving efficiency, reducing torque ripple, and significantly enhancing dynamic performance.

Conceptually, a single control iteration follows the sequence illustrated below.

```text
Phase Current Measurements
           │
           ▼
     Clarke Transform
           │
           ▼
      αβ Currents
           │
           ▼
      Park Transform
           │
           ▼
      dq Currents
           │
           ▼
    Current Regulation
           │
           ▼
     dq Voltages
           │
           ▼
  Inverse Park Transform
           │
           ▼
      αβ Voltages
           │
           ▼
Space Vector Modulation
           │
           ▼
   PWM Duty Cycles
```

Each stage performs a specific mathematical operation that progressively converts measured electrical quantities into inverter switching commands. Rather than viewing these operations as independent algorithms, it is more useful to consider them as successive stages within a single deterministic control cycle.

#### 5.4.1 Current Measurement

Every control iteration begins with measurement of the motor phase currents.

The Hardware Abstraction Layer performs synchronized analog-to-digital conversion of the motor current sensors, ensuring that all phase currents are sampled at a consistent point within the PWM cycle. Accurate synchronization is essential because the phase currents vary continuously throughout each switching period.

Under balanced operating conditions,

$$
i_A+i_B+i_C=0,
$$

allowing two independent current measurements to completely describe the electrical state of the stator.

These measured currents provide the feedback required by the current control loop and form the starting point for every subsequent mathematical transformation performed by the Field-Oriented Control algorithm.

#### 5.4.2 Coordinate Transformation

The measured phase currents initially exist within the physical three-phase reference frame.

The Clarke Transform first projects these quantities into the stationary αβ reference frame, eliminating the redundant third phase while preserving the complete electrical state of the motor.

The Park Transform then rotates the stationary current vector into the rotor reference frame using the measured electrical angle.

Following these transformations, the controller no longer observes three sinusoidal phase currents.

Instead, it observes two approximately constant quantities:

- the d-axis current, representing magnetic flux, and
- the q-axis current, representing torque-producing current.

This transformation greatly simplifies current regulation because the controller operates on nearly constant signals rather than continuously varying sinusoidal waveforms.

The firmware performs both coordinate transformations during every control iteration before entering the current regulation stage. :contentReference[oaicite:0]{index=0}

#### 5.4.3 Current Regulation

Once the measured currents have been transformed into the rotating reference frame, independent proportional-integral controllers regulate the d-axis and q-axis current components.

The d-axis controller regulates magnetic flux while the q-axis controller regulates electromagnetic torque.

For the surface-mounted permanent-magnet motor used by the reaction wheel, the desired operating point is

$$
i_d=0,
$$

allowing the entire available current to contribute toward torque production through regulation of

$$
i_q.
$$

Each controller computes a voltage correction proportional to the corresponding current error,

$$
e=i_{ref}-i_{meas},
$$

producing independent voltage commands

$$
v_d
$$

and

$$
v_q.
$$

Because the dq reference frame removes the sinusoidal behavior present in the original phase currents, conventional PI controllers may be used without requiring continuously varying controller gains or phase-dependent compensation.

The resulting voltage commands represent the electrical effort required to drive the measured currents toward their desired values.

#### 5.4.4 Feedforward and Decoupling Compensation

Although the dq reference frame largely decouples the motor dynamics, the electrical behavior of a permanent-magnet synchronous motor remains partially coupled through its rotational motion.

As rotor speed increases, additional voltage components arise due to the interaction between the rotating magnetic field and the stator inductances. If left uncompensated, these effects reduce current regulation accuracy and limit high-speed performance.

To improve controller performance, the firmware incorporates feedforward and cross-coupling compensation terms that estimate these predictable voltage components before they appear as current regulation errors.

Rather than forcing the PI controllers to compensate entirely through feedback, the controller proactively applies the expected voltage required to counteract the motor's rotational dynamics.

This approach offers several advantages.

First, transient current errors are reduced because much of the required control effort is supplied before significant feedback error develops.

Second, the proportional and integral controllers operate with reduced steady-state error, allowing smaller controller gains while maintaining equivalent dynamic performance.

Finally, compensation becomes increasingly valuable at higher electrical speeds, where cross-coupling effects become more pronounced.

The feedforward and decoupling calculations therefore improve both dynamic response and current regulation accuracy without fundamentally altering the underlying Field-Oriented Control architecture.

#### 5.4.5 Voltage Limiting and Anti-Windup

The voltage commands generated by the d-axis and q-axis current controllers represent the ideal electrical effort required to achieve the desired motor currents. In practice, however, the inverter cannot generate arbitrary voltages.

The maximum available phase voltage is fundamentally limited by the DC bus supplying the inverter. Regardless of the controller output, the magnitude of the commanded voltage vector cannot exceed the voltage that can be synthesized through pulse-width modulation.

If the controller were allowed to command voltages beyond this physical limit, the requested voltage vector would become unattainable. The inverter would saturate, while the integral terms within the PI controllers would continue to accumulate error under the assumption that additional control effort was still available. This phenomenon, commonly referred to as **integrator windup**, leads to excessive overshoot, prolonged recovery times, and degraded transient performance once the controller eventually leaves saturation.

To prevent this behavior, the firmware first evaluates the magnitude of the commanded voltage vector,

$$
V=\sqrt{v_d^2+v_q^2},
$$

and compares it with the maximum voltage that can be produced from the available DC bus.

Whenever the requested magnitude exceeds this limit, the voltage vector is uniformly scaled while preserving its direction,

$$
\begin{aligned}
v_d' &= kv_d,\\
v_q' &= kv_q,
\end{aligned}
$$

where the scaling factor $k$ is selected such that

$$
\sqrt{{v_d'}^2+{v_q'}^2}=V_{max}.
$$

Maintaining the direction of the voltage vector is essential because the vector orientation determines the balance between torque-producing and flux-producing voltage components. Scaling the vector rather than independently clipping each axis preserves the intended control action while ensuring that the commanded voltage remains physically realizable.

Following voltage limiting, the firmware applies anti-windup protection to the current controller integrators. Whenever voltage saturation occurs, the integral terms are prevented from accumulating additional error that cannot be corrected by the inverter. As a result, the controller remains responsive when the operating point returns to the controllable region.

Together, voltage limiting and anti-windup provide an important layer of robustness within the control system. They ensure that the current controllers remain stable across the full operating range of the reaction wheel while respecting the physical limitations imposed by the power electronics.

#### 5.4.6 Voltage Synthesis

After the commanded voltage vector has been validated and limited, the final stage of the Field-Oriented Control algorithm converts the rotating dq voltage commands into inverter switching signals.

The first operation is the Inverse Park Transform, which rotates the voltage vector from the dq reference frame back into the stationary αβ reference frame using the measured electrical rotor angle. At this point, the commanded voltage once again represents the desired stator excitation rather than quantities aligned with the rotor.

The resulting αβ voltage vector is then supplied to the Space Vector Modulation algorithm.

Rather than directly generating three sinusoidal phase voltages, Space Vector Modulation determines the adjacent inverter switching vectors that most closely approximate the desired voltage. By calculating the appropriate switching durations within each PWM period, the algorithm synthesizes the required average voltage while maximizing utilization of the available DC bus.

Finally, the modulation algorithm produces three normalized PWM duty cycles corresponding to phases A, B, and C. These duty cycles are passed to the Hardware Abstraction Layer, where they are converted into timer compare values and applied to the inverter hardware.

At the completion of this stage, one complete Field-Oriented Control iteration has transformed measured phase currents into updated inverter switching commands. The process repeats during every 16 kHz control cycle, continuously regulating the motor currents as the rotor rotates.

#### 5.4.7 Summary

Field-Oriented Control combines the mathematical tools introduced earlier in this chapter into a unified control algorithm capable of independently regulating magnetic flux and electromagnetic torque.

Beginning with synchronized phase current measurements, each control iteration performs a sequence of coordinate transformations, current regulation, voltage limiting, and modulation before producing updated PWM duty cycles for the inverter. Although these individual operations are mathematically distinct, together they form a single deterministic control cycle executed at the fixed 16 kHz update rate established by the firmware architecture.

Compared with trapezoidal and sinusoidal commutation, Field-Oriented Control offers several significant advantages.

Continuous current regulation reduces torque ripple, improves dynamic response, and enables efficient operation across a wide range of speeds. Independent regulation of the d-axis and q-axis currents provides direct control over the motor's electromagnetic behavior, while Space Vector Modulation maximizes utilization of the available DC bus and produces smooth three-phase excitation.

These characteristics make Field-Oriented Control particularly well suited to reaction wheel applications, where precise torque production, smooth operation, and high electrical efficiency are essential for accurate spacecraft attitude control.

For these reasons, Field-Oriented Control serves as the primary commutation strategy evaluated throughout the experimental results presented later in this thesis.

## 6. Runtime Operation

Although the previous sections described the mathematical principles underlying the supported commutation strategies, these algorithms ultimately execute as part of a deterministic real-time control loop.

During each 16 kHz control cycle, the Controller first determines the required electrical command based on the current operating state of the reaction wheel. This command, together with the measured electrical quantities provided by the Hardware Abstraction Layer, is then passed to the Commutation subsystem.

The selected commutation strategy performs the mathematical operations necessary to convert these inputs into three PWM duty cycles, which are subsequently applied to the inverter through the Hardware Abstraction Layer before the next control period begins.

Regardless of the active commutation strategy, the overall execution sequence remains unchanged.

```text
Encoder Measurement
        │
        ▼
Current Measurement
        │
        ▼
 Controller Update
        │
        ▼
Commutation Strategy
        │
        ▼
 PWM Duty Cycles
        │
        ▼
 Hardware Abstraction Layer
        │
        ▼
 Three-Phase Inverter
        │
        ▼
 Brushless Motor
```

This deterministic execution sequence ensures that every commutation strategy operates under identical timing conditions. While the mathematical complexity of the individual algorithms differs considerably, the surrounding control architecture remains unchanged. Consequently, differences observed during experimental evaluation can be attributed to the commutation strategy itself rather than variations in scheduling or software structure.

Because every strategy conforms to the same execution model, selecting a different commutation algorithm requires only changing the active strategy while leaving the remainder of the firmware unchanged. This modular organization significantly simplified both software development and the comparative testing presented later in this thesis.

## 7. Source Code Organization

The Commutation subsystem was designed around a modular strategy architecture that separates common control functionality from algorithm-specific implementations.

Rather than embedding multiple commutation algorithms within a single control routine, each strategy is implemented as an independent software module that conforms to a shared interface. This organization provides a consistent abstraction between the Controller and the individual commutation algorithms while allowing new strategies to be incorporated with minimal modification to the surrounding software.

Conceptually, the software organization is illustrated below.

```text
              Controller
                  │
                  ▼
        Commutation Interface
                  │
     ┌────────────┼────────────┐
     │            │            │
     ▼            ▼            ▼
Trapezoidal   Sinusoidal     Open Loop
     │            │            │
     └────────────┼────────────┘
                  │
                  ▼
        Field-Oriented Control
                  │
                  ▼
          PWM Duty Cycles
                  │
                  ▼
     Hardware Abstraction Layer
```

Each commutation strategy receives a common collection of controller-generated inputs, including the desired electrical command and the measured electrical state of the motor. The strategy implementation is responsible solely for determining the appropriate PWM duty cycles required to realize the requested operating condition.

This separation provides several important advantages.

First, the Controller remains entirely independent of the selected commutation algorithm. Speed regulation, state management, and supervisory control therefore remain identical regardless of whether trapezoidal, sinusoidal, open-loop vector control, or field-oriented control is active.

Second, common mathematical utilities such as the Clarke Transform, Park Transform, Inverse Park Transform, and Space Vector Modulation are implemented as shared library functions rather than duplicated across multiple strategies. This approach reduces code duplication, improves maintainability, and ensures consistent mathematical behavior throughout the firmware.

Finally, the Hardware Abstraction Layer remains isolated from the mathematical implementation of each strategy. Regardless of the internal calculations performed by the selected algorithm, the Hardware Abstraction Layer receives only the final PWM duty cycles required to drive the inverter.

## 8. Engineering Decisions

Several architectural decisions were made during the development of the Commutation subsystem to improve modularity, maintainability, and experimental repeatability.

A primary design objective was to maintain a strict separation between the Controller and the commutation algorithms. By defining a common interface between these subsystems, modifications to one component do not require changes to the other. This separation greatly simplified development while ensuring that improvements to individual commutation strategies could be evaluated without affecting the surrounding control architecture.

Similarly, mathematical operations common to multiple strategies were centralized within a shared BLDC mathematics library. Rather than implementing separate versions of the Clarke Transform, Park Transform, Inverse Park Transform, and Space Vector Modulation algorithms within each commutation strategy, these functions are implemented once and reused throughout the firmware. Besides reducing software complexity, this approach guarantees that all vector-based strategies perform identical coordinate transformations and modulation calculations.

The firmware also distinguishes between strategy-specific and strategy-independent functionality. Tasks such as state management, controller execution, and hardware interaction remain external to the individual commutation algorithms, allowing each strategy to focus exclusively on the mathematical generation of inverter commands.

From an experimental perspective, this architecture provides an important advantage. Because all strategies execute within the same deterministic control loop, utilize identical hardware interfaces, and receive the same controller-generated inputs, differences observed during testing are primarily attributable to the commutation algorithm itself. This consistency was essential to achieving the primary objective of this work: a meaningful comparison of multiple commutation strategies on a common reaction wheel platform.

Finally, the modular organization naturally supports future expansion. Additional commutation strategies, alternative modulation methods, or more advanced current regulators can be incorporated by implementing the common strategy interface while leaving the remainder of the firmware unchanged. This extensibility allows the software architecture to evolve alongside future reaction wheel hardware revisions without requiring fundamental changes to the overall control framework.

## 9. Chapter Summary

This chapter presented the design and implementation of the Commutation subsystem used by the reaction wheel controller. Beginning with the motivation for electronic commutation in brushless permanent-magnet motors, the chapter introduced the reference frames and mathematical transformations that underpin modern motor control, including the Clarke Transform, Park Transform, Inverse Park Transform, and Space Vector Modulation.

Building upon this mathematical foundation, the four commutation strategies implemented within the firmware were examined in increasing order of complexity. Open-loop vector control, trapezoidal commutation, sinusoidal commutation, and Field-Oriented Control each employ different methods of generating the rotating magnetic field required for torque production while operating through a common software interface. Particular attention was given to the implementation of Field-Oriented Control, which forms the primary commutation strategy investigated throughout this work.

The chapter concluded by describing the runtime execution model, modular software architecture, and engineering decisions that enable multiple commutation strategies to coexist within a common deterministic control framework. This architecture ensures that each strategy operates under identical conditions, providing the foundation for the comparative experimental evaluation presented later in this thesis.

With the hardware platform, software architecture, control system, and commutation subsystem now established, the following chapter describes the experimental methodology used to evaluate the performance of each commutation strategy. The test procedures, measurement instrumentation, and data acquisition techniques presented therein provide the basis for the quantitative comparison discussed in the subsequent results chapter.