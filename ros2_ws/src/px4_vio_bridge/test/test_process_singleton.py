import pytest

from px4_vio_bridge.process_singleton import ProcessSingleton


def test_singleton_rejects_second_holder_and_recovers_after_close(tmp_path):
    first = ProcessSingleton(
        "global_planner_monitor", domain_id="42", lock_directory=tmp_path
    )
    try:
        with pytest.raises(RuntimeError, match="duplicate global_planner_monitor"):
            ProcessSingleton(
                "global_planner_monitor", domain_id="42", lock_directory=tmp_path
            )
    finally:
        first.close()

    replacement = ProcessSingleton(
        "global_planner_monitor", domain_id="42", lock_directory=tmp_path
    )
    replacement.close()


def test_singletons_are_scoped_by_role_and_ros_domain(tmp_path):
    locks = [
        ProcessSingleton("global_planner_monitor", domain_id="41", lock_directory=tmp_path),
        ProcessSingleton("global_planner_monitor", domain_id="42", lock_directory=tmp_path),
        ProcessSingleton("route_follower_monitor", domain_id="42", lock_directory=tmp_path),
    ]
    for lock in locks:
        lock.close()
