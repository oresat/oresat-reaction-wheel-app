# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# The repository contains its own custom Zephyr board definition under boards/.
# BOARD_ROOT must be set before find_package(Zephyr).
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(oresat_mcxn947_reactionwheels)

# =============================================================================
# Application entry point
#
# The root Kconfig choice guarantees that exactly one of these symbols is
# enabled. Each selected source file provides main().
# =============================================================================

if(CONFIG_RW_APP_PRODUCTION)
    target_sources(app PRIVATE
        src/main.c
    )

elseif(CONFIG_RW_APP_TEST_CALIBRATION)
    target_sources(app PRIVATE
        tests/bringup/testCalibration.c
    )

elseif(CONFIG_RW_APP_TEST_CURRENT_PREPROCESS)
    target_sources(app PRIVATE
        tests/bringup/testCurrentPreprocess.c
    )

elseif(CONFIG_RW_APP_TEST_DIRECT_RAMP)
    target_sources(app PRIVATE
        tests/bringup/testDirectRamp.c
    )

elseif(CONFIG_RW_APP_TEST_ENCODER)
    target_sources(app PRIVATE
        tests/bringup/testEncoder.c
    )

elseif(CONFIG_RW_APP_TEST_HAL)
    target_sources(app PRIVATE
        tests/bringup/testHAL.c
    )

elseif(CONFIG_RW_APP_TEST_PWM)
    target_sources(app PRIVATE
        tests/bringup/testPWM.c
    )

elseif(CONFIG_RW_APP_EXPERIMENT_DYNAMIC_RESPONSE)
    target_sources(app PRIVATE
        tests/experiments/DynamicResponseTest.c
    )

elseif(CONFIG_RW_APP_EXPERIMENT_POWER_RIPPLE)
    target_sources(app PRIVATE
        tests/experiments/PowerRippleTest.c
    )

elseif(CONFIG_RW_APP_EXPERIMENT_THERMAL_SOAK)
    target_sources(app PRIVATE
        tests/experiments/ThermalSoakTest.c
    )

elseif(CONFIG_RW_APP_EXPERIMENT_TORQUE_SPEED)
    target_sources(app PRIVATE
        tests/experiments/TorqueSpeedTest.c
    )

else()
    message(FATAL_ERROR
        "No reaction-wheel application entry point was selected"
    )
endif()

# =============================================================================
# Common firmware
#
# These modules are compiled for production, bring-up tests, and experiments.
# Sources are listed explicitly instead of using recursive globbing.
# =============================================================================

target_sources(app PRIVATE
    src/app/Fsm.c

    src/calibration/Calibration.c
    src/calibration/CalibrationStore.c

    src/comms/Od.c
    src/comms/Telemetry.c

    src/commutation/Commutation_Foc.c
    src/commutation/Commutation_OpenLoop.c
    src/commutation/Commutation_Sine.c
    src/commutation/Commutation_Trap.c

    src/control/Controller.c
    src/control/CurrentPreprocess.c

    src/hal/Encoder.c
    src/hal/HalAdc.c
    src/hal/HalInputMux.c
    src/hal/HalPwm.c
    src/hal/Hal_Zephyr.c

    src/math/BldcMath.c
    src/math/MathUtil.c
)

# =============================================================================
# Optional firmware modules
# =============================================================================

if(CONFIG_RW_CAN_COMMS)
    target_sources(app PRIVATE
        src/comms/CanComms.c
    )
endif()

# =============================================================================
# Header search paths
#
# Headers remain beside their implementation files. Existing short includes
# such as #include "Controller.h" can therefore remain unchanged.
# =============================================================================

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calibration
    ${CMAKE_CURRENT_SOURCE_DIR}/src/comms
    ${CMAKE_CURRENT_SOURCE_DIR}/src/commutation
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/src/math
)