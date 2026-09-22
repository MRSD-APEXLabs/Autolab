from zedx_nano_depth.sync import PairBuffer


def test_pairs_complete_in_any_order_and_only_once():
    buffer = PairBuffer()
    assert buffer.add(0, 1, 'L1') is None
    assert buffer.add(1, 1, 'R1') == ('L1', 'R1')
    assert buffer.add(1, 2, 'R2') is None
    assert buffer.add(0, 2, 'L2') == ('L2', 'R2')
    assert buffer.add(0, 2, 'L2 again') is None   # the stamp was consumed


def test_completing_a_pair_drops_older_incomplete_stamps():
    buffer = PairBuffer()
    buffer.add(0, 1, 'L1')     # its right half never arrives
    buffer.add(0, 2, 'L2')
    assert buffer.add(1, 2, 'R2') == ('L2', 'R2')
    assert buffer.add(1, 1, 'R1') is None        # stamp 1 was discarded, R1 now waits alone
    assert buffer.add(0, 3, 'L3') is None and buffer.add(1, 3, 'R3') == ('L3', 'R3')


def test_incomplete_stamps_are_bounded():
    buffer = PairBuffer(keep=3)
    for stamp in range(10):
        buffer.add(0, stamp, stamp)
    assert len(buffer._pending) == 3 and buffer.add(1, 0, 'late') is None
    assert buffer.add(1, 9, 'R9') == (9, 'R9')
