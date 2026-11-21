"""Tests for server.py — run with: cd gcs/nlp_proxy && python3 -m pytest -q"""
import hashlib
import json
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

import mix_colors
import server

PROMPT_ID = hashlib.sha256(mix_colors.SYSTEM_PROMPT.encode()).hexdigest()
BROWN_CLEAR = mix_colors.expected(("brown", "clear"))
REFUSAL = {"error": "Specify two different liquids to mix: brown, clear or red."}


class FakeMod:
    """The real prompt and validate(); generate() and api() scripted per test."""
    SYSTEM_PROMPT = mix_colors.SYSTEM_PROMPT
    validate = staticmethod(mix_colors.validate)

    def __init__(self, answer=None, error=None, delay=0.0, version_ok=True):
        self.answer, self.error, self.delay, self.version_ok = answer, error, delay, version_ok
        self.tasks = []

    def generate(self, args, task, validate_output=True):
        self.tasks.append(task)
        if self.delay:
            time.sleep(self.delay)
        if self.error:
            raise RuntimeError(self.error)
        raw = json.dumps(self.answer, separators=(",", ":"))
        return raw, json.loads(raw)

    def api(self, args, endpoint, body=None):
        if not self.version_ok:
            raise RuntimeError("<urlopen error [Errno 111] Connection refused>")
        return {"version": "0.40.16"}


def make(tmp_path, mod, seed=None, cache=None, timeout=2.0):
    seed_path = tmp_path / "seed.json"
    cache_path = tmp_path / "cache.json"
    if seed is not None:
        seed_path.write_text(json.dumps(seed))
    if cache is not None:
        cache_path.write_text(json.dumps(cache))
    planner = server.Planner(mod, str(cache_path), str(seed_path), "http://127.0.0.1:11435", "qwen3.5:9b", timeout)
    return planner, cache_path


def seed_with(task, plan, prompt=PROMPT_ID):
    return {"prompt_sha256": prompt, "created_at": "2026-11-15T00:00:00+00:00",
            "tests": [{"task": task, "raw": json.dumps(plan), "passed": True}]}


def test_live_plan_returned_and_cached(tmp_path):
    planner, cache = make(tmp_path, FakeMod(answer=BROWN_CLEAR))
    r = planner.ask("Mix brown and clear.")
    assert r["ok"] and r["source"] == "live"
    assert r["plan"]["steps"][3]["name"] == "ot2"
    assert isinstance(r["seconds"], float)
    saved = json.loads(cache.read_text())
    assert "Mix brown and clear." in saved
    assert saved["Mix brown and clear."]["prompt"] == PROMPT_ID


def test_refusal_is_a_live_answer(tmp_path):
    planner, _ = make(tmp_path, FakeMod(answer=REFUSAL))
    r = planner.ask("Mix brown and blue liquids and shake them.")
    assert r["ok"] and r["source"] == "live" and r["plan"] == REFUSAL


def test_invalid_live_plan_is_discarded_not_cached(tmp_path):
    bad = {"steps": BROWN_CLEAR["steps"][:10]}
    planner, cache = make(tmp_path, FakeMod(answer=bad))
    r = planner.ask("Mix brown and clear.")
    assert r["ok"] is False and "plan" not in r
    assert not cache.exists()


def test_ollama_down_uses_recorded_answer(tmp_path):
    planner, _ = make(tmp_path, FakeMod(error="Connection refused"), seed=seed_with("Mix brown and clear.", BROWN_CLEAR))
    r = planner.ask("Mix brown and clear.")
    assert r["ok"] and r["source"] == "recorded"
    assert r["plan"] == BROWN_CLEAR
    assert "Connection refused" in r["why"]
    assert r["recordedAt"]


def test_timeout_uses_recorded_answer(tmp_path):
    planner, _ = make(tmp_path, FakeMod(answer=BROWN_CLEAR, delay=1.0),
                      seed=seed_with("Mix brown and clear.", BROWN_CLEAR), timeout=0.2)
    r = planner.ask("Mix brown and clear.")
    assert r["source"] == "recorded"
    assert "0.2" in r["why"]


def test_down_and_nothing_saved_is_error(tmp_path):
    planner, _ = make(tmp_path, FakeMod(error="Connection refused"))
    r = planner.ask("Mix brown and clear.")
    assert r == {"ok": False, "error": "Connection refused"}


def test_seed_with_other_prompt_hash_ignored(tmp_path):
    planner, _ = make(tmp_path, FakeMod(error="down"), seed=seed_with("Mix brown and clear.", BROWN_CLEAR, prompt="old"))
    assert planner.ask("Mix brown and clear.")["ok"] is False


def test_cache_entry_with_other_prompt_hash_ignored(tmp_path):
    cache = {"Mix brown and clear.": {"raw": json.dumps(BROWN_CLEAR), "at": "x", "from": "live", "prompt": "old"}}
    planner, _ = make(tmp_path, FakeMod(error="down"), cache=cache)
    assert planner.ask("Mix brown and clear.")["ok"] is False


def test_cache_entry_with_current_prompt_is_used(tmp_path):
    cache = {"Mix brown and clear.": {"raw": json.dumps(BROWN_CLEAR), "at": "x", "from": "live", "prompt": PROMPT_ID}}
    planner, _ = make(tmp_path, FakeMod(error="down"), cache=cache)
    assert planner.ask("Mix brown and clear.")["source"] == "recorded"


def test_task_is_trimmed_before_lookup(tmp_path):
    mod = FakeMod(error="down")
    planner, _ = make(tmp_path, mod, seed=seed_with("Mix brown and clear.", BROWN_CLEAR))
    r = planner.ask("  Mix brown and clear. \n")
    assert r["source"] == "recorded"
    assert mod.tasks == ["Mix brown and clear."]


def test_busy_when_three_in_flight(tmp_path):
    planner, _ = make(tmp_path, FakeMod(answer=BROWN_CLEAR, delay=0.5), timeout=2.0)
    results = []
    threads = [threading.Thread(target=lambda: results.append(planner.ask("Mix brown and clear."))) for _ in range(3)]
    for t in threads:
        t.start()
    time.sleep(0.1)
    fourth = planner.ask("Mix brown and clear.")
    for t in threads:
        t.join()
    assert fourth == {"ok": False, "error": "the planner is busy"}
    assert all(r["source"] == "live" for r in results)


def test_status_ready_and_down(tmp_path):
    ready, _ = make(tmp_path, FakeMod())
    assert ready.status() == {"state": "ready", "detail": "", "model": "qwen3.5:9b"}
    down, _ = make(tmp_path, FakeMod(version_ok=False))
    s = down.status()
    assert s["state"] == "down" and "refused" in s["detail"]


@pytest.fixture
def http(tmp_path):
    planner, _ = make(tmp_path, FakeMod(answer=BROWN_CLEAR))
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.make_handler(planner))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield "http://127.0.0.1:%d" % httpd.server_address[1]
    httpd.shutdown()


def call(url, body=None, raw=None):
    data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
    req = urllib.request.Request(url, data, {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, resp.headers.get("Content-Type"), json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, exc.headers.get("Content-Type"), json.loads(exc.read())


def test_http_routes(http):
    code, ctype, body = call(http + "/api/nlp", {"task": "Mix brown and clear."})
    assert code == 200 and ctype == "application/json" and body["source"] == "live"
    code, _, body = call(http + "/api/nlp/status")
    assert code == 200 and body["state"] == "ready"
    code, _, body = call(http + "/elsewhere")
    assert code == 404 and body["ok"] is False


def test_empty_task_is_400(http):
    code, _, body = call(http + "/api/nlp", {"task": "   "})
    assert code == 400 and body == {"ok": False, "error": "Type a request first."}


def test_long_task_is_400(http):
    code, _, body = call(http + "/api/nlp", {"task": "x" * 501})
    assert code == 400 and body["ok"] is False


def test_bad_json_is_400(http):
    code, _, body = call(http + "/api/nlp", raw=b"{not json")
    assert code == 400 and body["ok"] is False
