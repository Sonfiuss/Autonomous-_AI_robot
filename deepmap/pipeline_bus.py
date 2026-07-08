#!/usr/bin/env python3
"""
pipeline_bus — minimal generic thread-pipeline infrastructure (stdlib only).

Built for the stereo_walk_map multi-thread pipeline (task
2026-07-04_pipeline-walk-speed) but deliberately generic: future workers
(voice control, object detection) plug into the same Bus without changes here.

Design rules (from the task file):
- ONE owner per hardware resource (cameras / UART / GPU model); everything
  else is message passing over queue.Queue.
- Fan-out: subscribe() returns one queue PER subscriber, so several workers
  can consume the same topic independently.
- publish() blocks in short slices re-checking stop_event, so a full queue
  can never deadlock shutdown.
- Worker catches BaseException, NOT Exception: stereo_ruler paths call
  sys.exit() -> SystemExit, which `except Exception` misses -> dead worker
  and a coordinator that hangs forever.
- Shutdown: poison pill (None) per in-queue + join_all(timeout); a worker
  exits on pill OR stop_event.

Smoke test:  python3 pipeline_bus.py
"""
import queue
import threading
import time
import traceback
from dataclasses import dataclass, field


@dataclass
class Msg:
    """One bus message. err != None marks an EXPECTED per-item failure the
    consumer may skip (as opposed to a worker crash, which sets stop_event)."""
    topic: str
    idx: int
    payload: dict = field(default_factory=dict)
    err: str = None


class Bus:
    """Topic fan-out over stdlib queues. Subscribe before starting workers."""

    def __init__(self, stop_event=None):
        self.stop_event = stop_event if stop_event is not None else threading.Event()
        self._subs = {}                       # topic -> [Queue, ...]
        self._lock = threading.Lock()

    def subscribe(self, topic, q=None, maxsize=2):
        """Register (and return) a queue that receives every publish on
        `topic`. Pass an existing `q` to receive SEVERAL topics on one queue
        (join pattern, e.g. FusionMapper consuming "golden" + "mono")."""
        if q is None:
            q = queue.Queue(maxsize=maxsize)
        with self._lock:
            self._subs.setdefault(topic, []).append(q)
        return q

    def publish(self, msg, slice_s=0.2):
        """Deliver `msg` to every subscriber of its topic. A full queue blocks
        in `slice_s` slices, re-checking stop_event each time, so shutdown can
        never deadlock on a stuck consumer. Returns False if the message was
        dropped because stop_event got set while waiting."""
        for q in self._subs.get(msg.topic, ()):
            while True:
                if self.stop_event.is_set():
                    return False
                try:
                    q.put(msg, timeout=slice_s)
                    break
                except queue.Full:
                    continue
        return True


class Worker(threading.Thread):
    """Base pipeline thread: consume one in-queue, call handle() per Msg.

    - daemon=False: a fault must end in a JOINED clean exit (resources
      released in cleanup()), never a killed thread.
    - run() exits on poison pill (None) or stop_event.
    - Any raise from handle() — including SystemExit — logs a traceback and
      sets stop_event so the whole pipeline stops; cleanup() always runs.
    - stage_ms records per-idx handle() duration for --profile.
    """

    def __init__(self, name, bus, in_queue):
        super().__init__(name=name, daemon=False)
        self.bus = bus
        self.stop_event = bus.stop_event
        self.in_queue = in_queue
        self.stage_ms = {}                    # idx -> handle() wall-time ms

    def handle(self, msg):
        raise NotImplementedError

    def cleanup(self):
        """Release owned resources (cameras, serial port...). Always called."""

    def run(self):
        try:
            while not self.stop_event.is_set():
                try:
                    msg = self.in_queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                if msg is None:               # poison pill
                    break
                t0 = time.perf_counter()
                self.handle(msg)
                self.stage_ms[msg.idx] = (time.perf_counter() - t0) * 1e3
        except BaseException:                 # NOT Exception — see module doc
            print(f'[{self.name}] FATAL — stopping pipeline', flush=True)
            traceback.print_exc()
            self.stop_event.set()
        finally:
            self.cleanup()


def join_all(workers, stop_event, timeout=30.0):
    """Shut workers down IN LIST ORDER: pill its queue, then join it.

    Pass workers in topological (upstream-first) order: because each queue is
    FIFO and the upstream worker is joined before the downstream pill goes in,
    every in-flight message is processed before its consumer exits.

    The pill is RE-offered while joining: a busy worker's queue may be full of
    real work for many seconds (e.g. DepthWorker with two pairs backlogged) —
    a single put attempt would be dropped and the worker would idle forever.
    An extra pill after a race is harmless (the worker is gone; nothing reads).

    On a join timeout stop_event is set so the stragglers bail on their next
    loop check. Returns the list of workers still alive after the deadline."""
    deadline = time.time() + timeout
    stuck = []
    for w in workers:
        pilled = False
        while w.is_alive() and time.time() < deadline:
            if not pilled:
                try:
                    w.in_queue.put_nowait(None)
                    pilled = True
                except queue.Full:
                    pass                      # backlog — retry next iteration
            w.join(0.2)
        if w.is_alive():
            stuck.append(w)
            stop_event.set()
    return stuck


# ─────────────────────────────────────────────────────────────── smoke test ──
def _smoke():
    print('[smoke 1] 2 chained workers, 5 msgs ...')

    class Doubler(Worker):
        def handle(self, msg):
            self.bus.publish(Msg('b', msg.idx, {'v': msg.payload['v'] * 2}))

    class Collector(Worker):
        def __init__(self, *a):
            super().__init__(*a)
            self.got = []

        def handle(self, msg):
            self.got.append((msg.idx, msg.payload['v']))

    bus = Bus()
    wa = Doubler('doubler', bus, bus.subscribe('a'))
    wb = Collector('collector', bus, bus.subscribe('b'))
    wa.start()
    wb.start()
    for i in range(5):
        assert bus.publish(Msg('a', i, {'v': i}))
    stuck = join_all([wa, wb], bus.stop_event, timeout=5.0)
    assert not stuck, f'stuck workers: {stuck}'
    assert sorted(wb.got) == [(i, 2 * i) for i in range(5)], wb.got
    assert not bus.stop_event.is_set()
    print('   OK — 5 msgs through both stages, clean join')

    print('[smoke 2] injected SystemExit in a worker ...')

    class Crasher(Worker):
        def handle(self, msg):
            if msg.idx == 2:
                raise SystemExit('injected crash')   # what sys.exit() raises
            self.bus.publish(Msg('b', msg.idx, {'v': msg.payload['v']}))

    bus = Bus()
    wa = Crasher('crasher', bus, bus.subscribe('a'))
    wb = Collector('collector', bus, bus.subscribe('b'))
    wa.start()
    wb.start()
    for i in range(5):
        bus.publish(Msg('a', i, {'v': i}))
    t0 = time.time()
    stuck = join_all([wa, wb], bus.stop_event, timeout=5.0)
    assert not stuck, f'stuck workers: {stuck}'
    assert bus.stop_event.is_set(), 'crash must set stop_event'
    assert time.time() - t0 < 5.0, 'join_all must not hang on a crash'
    print(f'   OK — stop_event set, joined in {time.time() - t0:.2f}s '
          f'({len(wb.got)} msgs made it through before the fault)')
    print('smoke test PASSED')


if __name__ == '__main__':
    _smoke()
