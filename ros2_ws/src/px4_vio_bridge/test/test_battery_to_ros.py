from types import SimpleNamespace

from px4_vio_bridge.battery_to_ros import LEVEL_NAMES, BatteryToRos


def make_stub(**overrides):
    stub = SimpleNamespace(
        warn_percent=40.0,
        critical_percent=25.0,
        empty_percent=15.0,
        warn_cell=3.60,
        critical_cell=3.45,
        empty_cell=3.30,
        default_cell_count=3,
    )
    stub.__dict__.update(overrides)
    return stub


def level(percent, cell_v, warning=0, **overrides):
    return BatteryToRos.level_from(make_stub(**overrides), percent, cell_v, warning)


def test_healthy_battery_is_ok() -> None:
    assert level(80.0, 4.00) == 0


def test_percent_thresholds() -> None:
    assert level(41.0, 4.00) == 0
    assert level(39.0, 4.00) == 1  # LOW
    assert level(24.0, 4.00) == 2  # CRITICAL
    assert level(14.0, 4.00) == 3  # EMPTY


def test_the_2026_07_27_flight_would_have_read_critical() -> None:
    """11% SoC at 10.97 V on 3S = 3.66 V/cell -- the SoC estimate is what caught it."""
    assert level(11.0, 10.97 / 3.0) == 3
    assert LEVEL_NAMES[3] == "EMPTY"


def test_cell_voltage_can_escalate_past_an_optimistic_soc() -> None:
    # SoC says fine, cells are sagging: the worse of the two must win.
    assert level(90.0, 3.40) == 2
    assert level(90.0, 3.20) == 3


def test_px4_warning_is_never_masked() -> None:
    # Healthy percent and voltage, but PX4 itself is shouting.
    assert level(90.0, 4.00, warning=1) == 1
    assert level(90.0, 4.00, warning=2) == 2
    assert level(90.0, 4.00, warning=3) == 3
    assert level(90.0, 4.00, warning=4) == 3  # FAILED collapses into EMPTY


def test_missing_fields_do_not_fabricate_a_level() -> None:
    # PX4 marks invalid as remaining=-1 / voltage=0; those become None upstream.
    assert level(None, None) == 0
    assert level(None, 3.20) == 3
    assert level(11.0, None) == 3


def test_cell_count_falls_back_when_px4_reports_zero() -> None:
    stub = make_stub()
    assert BatteryToRos.cell_count(stub, SimpleNamespace(cell_count=0)) == 3
    assert BatteryToRos.cell_count(stub, SimpleNamespace(cell_count=4)) == 4
