#!/usr/bin/env python3
"""
Minimal Fz + auxiliary-channel logger using pylablib's NI.NIDAQ wrapper.

Reads the ATI Nano17 (FT12876) via its 6 raw strain-gauge channels (ai0:5)
and converts them to calibrated Fz (Newtons) with the same ATI calibration
matrix record_cone_press.py uses. A 7th channel, ai16, is logged alongside it
as a raw voltage (assumed to be an unrelated analog signal, not part of the
ATI sensor).

NOTE: an ATI F/T sensor cannot report Fz from a single gauge channel - its
calibration matrix mixes all 6 raw channels into every force/torque axis
(see ConvertToFT in forceDAQ/daq/_pyATIDAQ.py), so all 6 must be read even
though only Fz is kept here.

Stop with Ctrl-C.
"""

import os
import sys
import ctypes as ct

import numpy as np
from pylablib.devices import NI

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from forceDAQ.daq import ATI_CDLL
from forceDAQ._lib.misc import find_calibration_file

# --------------------------------------------------------------------------- #
DEVICE_NAME = "Dev1"

# ATI Nano17 raw strain-gauge channels (order matches ForceData.forces_names
# after calibration: Fx,Fy,Fz,Tx,Ty,Tz -> Fz is index 2 of the converted array).
GAUGE_CHANNELS = ["ai0", "ai1", "ai2", "ai3", "ai4", "ai5"]
# DIFFERENTIAL matches the wiring used elsewhere in pyForceDAQ for this sensor
# (DAQmx_Val_Diff in forceDAQ/daq/_daq_read_analog_nidaqmx.py) - do not change
# to "rse" unless the ATI breakout box has actually been rewired single-ended.
GAUGE_TERMINAL_CFG = "diff"

EXTRA_CHANNEL = "ai16"
# Wiring for ai16 is unknown - "default" lets the board pick its native
# config. Change to "diff"/"rse"/"nrse" to match whatever is actually driving
# this channel.
EXTRA_TERMINAL_CFG = "default"

SENSOR_NAME = "FT12876"
CALIBRATION_FOLDER = "calibration"
REVERSE_FZ = True   # keep sign convention: press -> positive Fz (see record_cone_press.py)
FZ_INDEX = 2        # ForceData.forces_names = ["Fx","Fy","Fz","Tx","Ty","Tz"]

RATE_HZ = 500
BIAS_SAMPLES = 500
# --------------------------------------------------------------------------- #


def main():
    daq = NI.NIDAQ(DEVICE_NAME)
    for ch in GAUGE_CHANNELS:
        daq.add_voltage_input(ch, ch, terminal_cfg=GAUGE_TERMINAL_CFG)
    daq.add_voltage_input("extra", EXTRA_CHANNEL, terminal_cfg=EXTRA_TERMINAL_CFG)
    daq.setup_clock(RATE_HZ)

    atidaq = ATI_CDLL()
    cal_file = find_calibration_file(CALIBRATION_FOLDER, SENSOR_NAME)
    atidaq.createCalibration(cal_file, ct.c_short(1))
    atidaq.setForceUnits("N")
    atidaq.setTorqueUnits("N-m")

    reverse = [FZ_INDEX] if REVERSE_FZ else []

    print("Setting bias - DO NOT TOUCH THE SENSOR ...")
    bias_samples = daq.read(BIAS_SAMPLES)[:, :6]  # gauge columns only, ai16 not biased
    atidaq.bias(np.mean(bias_samples, axis=0))
    print("Sensor calibrated (biased).\n")

    print("Recording. Press Ctrl-C to stop.\n")
    try:
        daq.start()
        while True:
            sample = daq.read(1)[0]
            gauge_voltages, extra_voltage = sample[:6], sample[6]
            forces = atidaq.convertToFT(gauge_voltages, reverse_parameters=reverse)
            fz = forces[FZ_INDEX]
            print(f"Fz = {fz:8.4f} N | ai16 = {extra_voltage:8.4f} V")
    except KeyboardInterrupt:
        print("\nStopping ...")
    finally:
        daq.stop()
        daq.close()


if __name__ == "__main__":
    main()
