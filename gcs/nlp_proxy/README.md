# nlp-proxy

The language planner behind the routine builder's **Describe the experiment** box. It turns a
plain-English request ("mix 0.1 ml red with yellow and shake for 30 seconds") into the 11-step
routine, or a refusal with a reason. It only generates plans; nothing here talks to lab hardware.

- `mix_colors.py` started as a copy of `FVD-Presentation/NLP/mix_colors.py` and now differs from it:
  the liquids are red (tube A1), yellow (A2) and blue (A3). A3 is left empty on the deck, so any
  routine with blue aspirates nothing and the clog check flags it (the anomaly demo). Saved answers
  in `pair_test_results.json` are keyed by the sha256 of the prompt, so after a prompt change they
  are ignored until regenerated: `python3 mix_colors.py` (on the 5090, or `--host local`) rewrites
  that file.
- `server.py` (standard library only) calls Ollama on this host, checks every answer with the
  planner's own `validate()`, and saves passing live answers. If Ollama does not answer within
  `NLP_TIMEOUT`, the saved answer for exactly that request comes back marked `recorded`.
- Ollama must be running on the host (on the 5090: the isolated install in
  `~/.local/share/lab-automation-ollama`, `127.0.0.1:11435`; restart command in
  `FVD-Presentation/NLP/README.md`). The container cannot start it.

## Endpoints

| | |
|---|---|
| `POST /api/nlp` `{"task": "..."}` | `{"ok": true, "source": "live", "plan": {...}, "seconds": 1.1}`, `{"ok": true, "source": "recorded", "plan": {...}, "why": "...", "recordedAt": "..."}` or `{"ok": false, "error": "..."}`. `plan` is `{"steps": [...]}` or `{"error": "..."}` (a refusal). 400 for an empty or > 500-character request. |
| `GET /api/nlp/status` | `{"state": "ready" \| "down", "detail": "...", "model": "..."}` |

## Environment

| Variable | Default |
|---|---|
| `PORT` | `8090` |
| `NLP_BASE_URL` | `http://127.0.0.1:11435` |
| `NLP_MODEL` | `qwen3.5:9b` |
| `NLP_TIMEOUT` | `20` (seconds) |
| `NLP_CACHE` | `/data/nlp_cache.json` (the `nlp-cache` volume). The deck's `nlp_cache.json` has the same format and can be dropped in. |

## Running

In Docker: the `nlp-proxy` service in `gcs/docker/docker-compose.yaml` (profiles `hitl`, `deploy`,
host network). nginx (`ui`) and the dev server forward `/api/nlp` to it on port 8090, through
`host.docker.internal` (`host-gateway`).

It listens on all interfaces, with no authentication: anyone on the network can ask it for plans
(plans only, but they use the GPU). Keep the GCS on the lab network.

**Firewall.** The `ui` container reaches it from the `autolab_network` bridge (172.31.0.0/24). A
host firewall that drops incoming traffic by default (ufw, firewalld) blocks that, and the builder
then waits about 60 s and says "Planner unavailable". Allow it once:

```bash
sudo ufw allow from 172.31.0.0/24 to any port 8090 proto tcp
```

Check the whole chain from inside the UI container:

```bash
docker exec $(docker ps -qf name=ui) wget -qO- http://host.docker.internal:8090/api/nlp/status
```

By hand (any machine, Python 3.9+): `NLP_CACHE=/tmp/nlp_cache.json python3 gcs/nlp_proxy/server.py`.
Without Ollama it still answers the builder's demo prompts from saved answers (once
`pair_test_results.json` has been regenerated for the current prompt).

Tests: `cd gcs/nlp_proxy && python3 -m pytest -q`
