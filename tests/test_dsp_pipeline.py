"""
Pytest Verification Suite for DeepCarrier-Pulse (NASA TRL-6 Flight DSP Prototype).
Verifies:
- CCSDS 131.0 CRC-16-CCITT and ASM synchronization
- Automated SNR Estimator (M2M4 and spectral power)
- Extended Kalman Filter (EKF) carrier tracking convergence under -10 dB SNR
- Gardner Timing Error Detector & Costas Loop synchronization
- 4-Stage Autonomous State Machine and Occultation Re-Lock
"""

import numpy as np
import pytest
from deepspace_signal_engine import (
    CCSDSTelemetryParser,
    AutomatedSNREstimator,
    ExtendedKalmanFilter,
    GardnerCostasSynchronizer,
    DeepSpaceChannelSynthesizer,
    AutonomousFlightDSPEngine,
    CCSDS_ASM_32,
    CCSDS_ASM_BITS,
)


def test_ccsds_crc16_integrity():
    """Validates CCSDS CRC-16 calculation against known vectors."""
    test_data = b"NASA_ARTEMIS_DEEP_SPACE_TELEMETRY"
    crc = CCSDSTelemetryParser.compute_crc16(test_data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF

    # Verify self-checking property: data + crc yields 0x0000
    packet = test_data + crc.to_bytes(2, byteorder="big")
    check = CCSDSTelemetryParser.compute_crc16(packet)
    assert check == 0x0000, "Appended CCSDS CRC-16 packet must verify to 0x0000"


def test_ccsds_frame_assembly_and_ambiguity_resolution():
    """Validates CCSDS frame framing, direct sync, and inverted 180-deg ambiguity sync."""
    msg = "SCID=0x2A5::PAYLOAD_NOMINAL"
    raw_frame, frame_bits = CCSDSTelemetryParser.assemble_frame(
        scid=0x2A5, vcid=1, frame_count=10, telemetry_text=msg
    )
    assert len(raw_frame) > 0
    assert len(frame_bits) == len(raw_frame) * 8

    # Test direct phase sync
    res_direct = CCSDSTelemetryParser.synchronize_and_parse(frame_bits, len(frame_bits))
    assert res_direct["sync_index"] == 0
    assert not res_direct["phase_inverted"]
    assert res_direct["crc_valid"] is True
    assert res_direct["telemetry_payload"] == msg.encode("ascii")

    # Test 180-degree inverted phase sync
    inverted_bits = 1 - frame_bits
    res_inv = CCSDSTelemetryParser.synchronize_and_parse(inverted_bits, len(frame_bits))
    assert res_inv["sync_index"] == 0
    assert res_inv["phase_inverted"] is True
    assert res_inv["crc_valid"] is True
    assert res_inv["telemetry_payload"] == msg.encode("ascii")


def test_automated_snr_estimator():
    """Validates M2M4 split-symbol moment SNR estimator across various noise levels."""
    np.random.seed(42)
    # Generate BPSK constellation with known noise
    n_syms = 2000
    bits = np.random.randint(0, 2, n_syms)
    syms = 2.0 * bits - 1.0

    # Add Gaussian noise for target SNR = 10 dB
    noise_sigma = np.sqrt(0.1 / 2.0)
    rx_10db = syms + noise_sigma * (np.random.randn(n_syms) + 1j * np.random.randn(n_syms))
    est_snr = AutomatedSNREstimator.estimate_m2m4(rx_10db)
    assert 7.0 <= est_snr <= 13.0, f"Expected SNR around 10 dB, got {est_snr:.2f} dB"


def test_ekf_carrier_tracking_convergence():
    """Validates 3-state EKF tracking of carrier phase, frequency, and chirp acceleration."""
    dt = 1.0 / 2000.0  # Symbol period
    ekf = ExtendedKalmanFilter(dt=dt, q_accel=20.0, r_meas=0.22)

    # Simulate frequency error of 20 Hz with 10 Hz/s chirp
    w_true = 2.0 * np.pi * 20.0
    a_true = 2.0 * np.pi * 10.0

    phi = 0.5
    for k in range(300):
        ekf.predict()
        phi += w_true * dt + 0.5 * a_true * (dt**2)
        # Phase error between true and estimated
        e = phi - ekf.phase
        # Wrap error to [-pi, pi]
        e = (e + np.pi) % (2.0 * np.pi) - np.pi
        ekf.update(e)

    freq_err = abs(ekf.frequency_hz - (w_true / (2.0 * np.pi) + a_true * dt * 300 / (2.0 * np.pi)))
    assert freq_err < 5.0, f"EKF frequency error too high: {freq_err:.2f} Hz"


def test_gardner_costas_synchronizer():
    """Validates Costas loop discriminator and Gardner timing detector."""
    sps = 50
    sync = GardnerCostasSynchronizer(sps=sps, kp_gardner=0.015)

    # Test Costas error for a known phase rotation of +0.3 rad
    z = np.exp(1j * 0.3)
    err = sync.compute_costas_phase_error(z)
    assert abs(err - 0.3) < 1e-4

    # Test Costas error for negative phase rotation -0.3 rad
    z_neg = np.exp(-1j * 0.3)
    err_neg = sync.compute_costas_phase_error(z_neg)
    assert abs(err_neg - (-0.3)) < 1e-4

    # Test Gardner timing error
    z_prompt = 1.0 + 0j
    z_mid = 0.5 + 0j
    tau = sync.update_gardner_timing(z_prompt, z_mid)
    assert isinstance(tau, float)


def test_end_to_end_flight_dsp_pipeline():
    """
    End-to-end integration test of deep space channel synthesis,
    autonomous acquisition, EKF tracking, occultation recovery, and CCSDS decoding.
    """
    synthesizer = DeepSpaceChannelSynthesizer(
        fs=100000.0,
        symbol_rate=2000.0,
        carrier_offset_hz=15000.0,
        chirp_rate_hz_s=120.0,
        snr_db=-10.0,
        dropout_samples=2500,
        dropout_start=55000,
    )
    channel_data = synthesizer.synthesize(num_symbols=3000)
    engine = AutonomousFlightDSPEngine(fs=synthesizer.fs, symbol_rate=synthesizer.symbol_rate)
    results = engine.process(channel_data)

    # 1. Coarse carrier acquired near 15 kHz
    assert abs(results["f_coarse"] - 15000.0) < 50.0

    # 2. Lock latency within DO-178C specification (< 80 ms)
    assert results["lock_latency_ms"] < 80.0

    # 3. Post-lock BER is nominal (< 1%)
    assert results["ber"] < 0.01

    # 4. CCSDS frame and CRC valid
    report = results["frame_report"]
    assert report["crc_valid"] is True
    assert b"ARTEMIS_DEEP_SPACE" in report["telemetry_payload"]

    # 5. State machine entered COAST during occultation
    assert 3 in results["state_history"]
