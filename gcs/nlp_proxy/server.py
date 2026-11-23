#!/usr/bin/env python3
"""nlp-proxy: the language planner for the GCS routine builder (Python standard library only).

POST /api/nlp {"task": "..."} runs the planner in mix_colors.py (adapted from FVD-Presentation/NLP
for the red / yellow / blue tubes) against Ollama on this host, checks the answer with its own
validate(), and returns it. If Ollama does not answer in time, the answer saved earlier for exactly that task is
returned instead, marked "recorded" (never "live"). GET /api/nlp/status says whether Ollama answers.

It only generates plans; nothing here talks to lab hardware. Adapted from the Planner class in
FVD-Presentation/present.py, without SSH and without starting Ollama.

Environment: PORT (8090), NLP_BASE_URL (http://127.0.0.1:11435), NLP_MODEL (qwen3.5:9b),
NLP_TIMEOUT (20 s), NLP_CACHE (/data/nlp_cache.json).
"""
import argparse
import hashlib
import importlib.util
import json
import logging
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
MAX_TASK = 500
MAX_IN_FLIGHT = 3
KEEP_CACHED = 200
WARM_EVERY_S = 120
WARM_TASK = "Please combine the red liquid with the yellow liquid, then shake the mixture."

log = logging.getLogger("nlp-proxy")


def _short(exc):
    text = str(exc).strip() or exc.__class__.__name__
    return text.splitlines()[-1][:200]


class Planner:
    """One task -> a checked plan: live from Ollama, else the saved answer for that task, else an error.

    `mod` provides SYSTEM_PROMPT, generate(args, task, validate_output), validate(plan) and
    api(args, endpoint, body=None): the mix_colors module in production."""

    def __init__(self, mod, cache_path, seed_path, base_url, model, timeout):
        self.mod = mod
        self.cache_path = cache_path
        self.args = argparse.Namespace(host="local", base_url=base_url, model=model)
        self.timeout = float(timeout)
        self.lock = threading.Lock()
        self.slots = threading.BoundedSemaphore(MAX_IN_FLIGHT)
        # saved answers are only reused under the prompt that produced them
        self.prompt_id = hashlib.sha256(mod.SYSTEM_PROMPT.encode()).hexdigest()
        self.recorded = {}
        self._load(seed_path)

    def _load(self, seed_path):
        try:
            with open(seed_path) as fp:
                report = json.load(fp)
            if report.get("prompt_sha256") == self.prompt_id:
                for test in report.get("tests", []):
                    if test.get("passed") and isinstance(test.get("raw"), str) and isinstance(test.get("task"), str):
                        self.recorded[test["task"]] = {"raw": test["raw"], "at": report.get("created_at"), "from": "tests"}
        except (OSError, ValueError, AttributeError):
            pass
        try:
            with open(self.cache_path) as fp:
                for task, record in json.load(fp).items():
                    if isinstance(record, dict) and isinstance(record.get("raw"), str) and record.get("prompt") == self.prompt_id:
                        self.recorded[task] = record
        except (OSError, ValueError, AttributeError):
            pass

    def _remember(self, task, raw):
        with self.lock:
            stamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            self.recorded.pop(task, None)                      # re-insert so it counts as newest
            self.recorded[task] = {"raw": raw, "at": stamp[:-2] + ":" + stamp[-2:], "from": "live", "prompt": self.prompt_id}
            live = [(t, r) for t, r in self.recorded.items() if r.get("from") == "live"]
            try:
                folder = os.path.dirname(self.cache_path)
                if folder:
                    os.makedirs(folder, exist_ok=True)
                with open(self.cache_path + ".tmp", "w") as fp:
                    json.dump(dict(live[-KEEP_CACHED:]), fp, indent=1, ensure_ascii=False)
                os.replace(self.cache_path + ".tmp", self.cache_path)
            except OSError as exc:
                log.warning("could not save %s: %s", self.cache_path, _short(exc))

    def _check(self, plan):
        """'' when the plan passes the planner's own check, else why not."""
        try:
            self.mod.validate(plan)
            return ""
        except ValueError as exc:
            return str(exc) or "invalid routine"

    def ask(self, task):
        task = task.strip()
        box, done = {}, threading.Event()
        if not self.slots.acquire(blocking=False):
            error = "the planner is busy"
        else:
            def work():
                started = time.monotonic()
                try:
                    raw, plan = self.mod.generate(self.args, task, validate_output=False)
                    box.update(raw=raw, plan=plan, problem=self._check(plan),
                               seconds=round(time.monotonic() - started, 2))
                    if not box["problem"]:
                        self._remember(task, raw)              # also when it lands after the timeout
                except Exception as exc:                       # noqa: BLE001 - HTTP, JSON, Ollama: report it
                    box["error"] = _short(exc)
                finally:
                    self.slots.release()
                    done.set()
            threading.Thread(target=work, daemon=True, name="planner").start()
            if not done.wait(self.timeout):
                error = "the model did not answer within %g s" % self.timeout
            elif "error" in box:
                error = box["error"]
            elif box["problem"]:
                error = "the planner's answer failed its check (%s)" % box["problem"]
            else:
                return {"ok": True, "source": "live", "plan": box["plan"], "seconds": float(box["seconds"])}
        record = self.recorded.get(task)
        if record:
            try:
                plan = json.loads(record["raw"])
            except ValueError:
                plan = None
            if plan is not None and not self._check(plan):     # an answer that no longer passes does not count
                return {"ok": True, "source": "recorded", "plan": plan, "why": error, "recordedAt": record.get("at") or ""}
        return {"ok": False, "error": error}

    def status(self):
        try:
            self.mod.api(self.args, "/api/version")
            return {"state": "ready", "detail": "", "model": self.args.model}
        except Exception as exc:                               # noqa: BLE001
            return {"state": "down", "detail": _short(exc), "model": self.args.model}


def make_handler(planner):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            log.info("%s %s", self.address_string(), fmt % args)

        def _json(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.split("?")[0] == "/api/nlp/status":
                return self._json(200, planner.status())
            self._json(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            if self.path.split("?")[0] != "/api/nlp":
                return self._json(404, {"ok": False, "error": "not found"})
            try:
                length = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(min(length, 64 * 1024)) or b"{}")
                task = body.get("task") if isinstance(body, dict) else None
            except ValueError:
                return self._json(400, {"ok": False, "error": "The request body must be JSON: {\"task\": \"...\"}."})
            if not isinstance(task, str) or not task.strip():
                return self._json(400, {"ok": False, "error": "Type a request first."})
            if len(task.strip()) > MAX_TASK:
                return self._json(400, {"ok": False, "error": "Keep the request under %d characters." % MAX_TASK})
            self._json(200, planner.ask(task))

    return Handler


def _load_mix_colors():
    spec = importlib.util.spec_from_file_location("mix_colors", os.path.join(HERE, "mix_colors.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _keep_warm(planner):
    """Ask now and then so the model stays loaded: a cold start takes about 5 s."""
    while True:
        try:
            planner.ask(WARM_TASK)
        except Exception as exc:                               # noqa: BLE001
            log.debug("warm-up failed: %s", _short(exc))
        time.sleep(WARM_EVERY_S)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    planner = Planner(
        _load_mix_colors(),
        cache_path=os.environ.get("NLP_CACHE", "/data/nlp_cache.json"),
        seed_path=os.path.join(HERE, "pair_test_results.json"),
        base_url=os.environ.get("NLP_BASE_URL", "http://127.0.0.1:11435"),
        model=os.environ.get("NLP_MODEL", "qwen3.5:9b"),
        timeout=float(os.environ.get("NLP_TIMEOUT", "20")),
    )
    threading.Thread(target=_keep_warm, args=(planner,), daemon=True, name="warm").start()
    port = int(os.environ.get("PORT", "8090"))
    httpd = ThreadingHTTPServer(("0.0.0.0", port), make_handler(planner))
    log.info("nlp-proxy on :%d, model %s at %s, %d saved answers",
             port, planner.args.model, planner.args.base_url, len(planner.recorded))
    httpd.serve_forever()


if __name__ == "__main__":
    main()
