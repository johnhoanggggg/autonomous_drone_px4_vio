from types import SimpleNamespace
import pytest
from px4_vio_bridge.foxglove_image_throttle import FrameGate, FoxgloveImageThrottle


@pytest.mark.parametrize('rate', [0, -1, float('nan'), float('inf')])
def test_invalid_rate(rate):
    with pytest.raises(ValueError):
        FrameGate(rate)


def test_rate_limit_drops_frames_without_catching_up():
    gate = FrameGate(5)
    assert gate.accept(0)
    assert not gate.accept(.1)
    assert gate.accept(.21)
    assert gate.accept(10)
    assert not gate.accept(10.01)


def test_forwarding_preserves_message_and_skips_without_viewers(monkeypatch):
    monkeypatch.setattr('px4_vio_bridge.foxglove_image_throttle.time.monotonic', lambda: 1.0)
    received = []
    fake = SimpleNamespace(gate=FrameGate(5), publisher=SimpleNamespace(
        get_subscription_count=lambda: 0, publish=received.append))
    message = object()
    FoxgloveImageThrottle.receive(fake, message)
    assert not received and fake.gate.last_sent is None
    fake.publisher.get_subscription_count = lambda: 1
    FoxgloveImageThrottle.receive(fake, message)
    FoxgloveImageThrottle.receive(fake, object())
    assert received == [message]
