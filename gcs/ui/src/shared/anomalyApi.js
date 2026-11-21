// HTTP client for the clog-detection service (Anomaly-Detection/automation/service.py).
// The UI and the service both run on the GCS desktop; REACT_APP_ANOMALY_URL overrides.
export const ANOMALY_URL =
  process.env.REACT_APP_ANOMALY_URL || `http://${window.location.hostname}:8100`;

// JSON request to a service at base; throws with the service's {error} message on failure.
export async function requestAt(base, method, path, body) {
  const opts = { method };
  if (body !== undefined) {
    opts.headers = { 'Content-Type': 'application/json' };
    opts.body = JSON.stringify(body);
  }
  const res = await fetch(`${base}${path}`, opts);
  let data = null;
  try {
    data = await res.json();
  } catch {
    // non-JSON body
  }
  if (!res.ok) {
    const err = new Error(data?.error || `HTTP ${res.status}`);
    err.status = res.status;
    throw err;
  }
  return data;
}

const request = (method, path, body) => requestAt(ANOMALY_URL, method, path, body);

export const getResult = (clipId) => request('GET', `/result/${encodeURIComponent(clipId)}`);
export const getJobs = () => request('GET', '/jobs');
export const getHealth = () => request('GET', '/health');
export const startRecording = (body = {}) => request('POST', '/recording/start', body);
export const stopRecording = () => request('POST', '/recording/stop', {});
export const abortRecording = () => request('POST', '/recording/abort', {});
