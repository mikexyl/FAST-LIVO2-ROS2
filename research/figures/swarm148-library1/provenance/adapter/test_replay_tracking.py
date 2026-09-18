import pytest
from replay_tracking import ScanTracker


def test_delayed_ack_does_not_hide_missing_scan():
    scans = ScanTracker()
    for stamp in (100, 200, 300):
        scans.publish(stamp)
    scans.acknowledge(300)
    scans.acknowledge(100)
    assert scans.pending == {200}
    scans.acknowledge(200)
    assert not scans.pending


def test_reject_invalid_ack_and_nonmonotonic_input():
    scans = ScanTracker()
    with pytest.raises(ValueError, match='unpublished'):
        scans.acknowledge(100)
    scans.publish(100)
    with pytest.raises(ValueError, match='increase'):
        scans.publish(100)
    scans.acknowledge(100)
    with pytest.raises(ValueError, match='Duplicate'):
        scans.acknowledge(100)
