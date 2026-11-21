// Development-server proxy: browsers connect only to GCS, not the robot LAN.
const http = require('http');
const https = require('https');

module.exports = function setupProxy(app) {
  // Language planner (gcs/nlp_proxy). In Docker, ui-dev sets NLP_UPSTREAM to the host's nlp-proxy.
  const nlp = new URL(process.env.NLP_UPSTREAM || 'http://localhost:8090');
  app.use('/api/nlp', (req, res) => {
    const proxy = http.request({
      hostname: nlp.hostname,
      port: nlp.port,
      method: req.method,
      path: req.originalUrl,
      headers: { ...req.headers, host: nlp.host },
    }, (response) => {
      res.writeHead(response.statusCode, response.headers);
      response.pipe(res);
    });
    proxy.on('error', () => {
      if (res.headersSent) return res.destroy();
      res.statusCode = 502;
      res.setHeader('Content-Type', 'application/json');
      return res.end(JSON.stringify({ ok: false, error: 'nlp-proxy unavailable' }));
    });
    req.pipe(proxy);
  });

  const upstream = new URL(process.env.CAMERA_STREAM_UPSTREAM || 'http://thor.autolab:8002');
  const transport = upstream.protocol === 'https:' ? https : http;
  app.use('/camera-stream', (req, res) => {
    const proxy = transport.request({
      protocol: upstream.protocol,
      hostname: upstream.hostname,
      port: upstream.port,
      method: req.method,
      path: req.originalUrl.replace(/^\/camera-stream(?=\/|\?|$)/, '') || '/',
      headers: { ...req.headers, host: upstream.host },
    }, (response) => {
      res.writeHead(response.statusCode, {
        ...response.headers, 'cache-control': 'no-store', 'x-accel-buffering': 'no',
      });
      response.on('error', () => res.destroy());
      response.pipe(res);
      // An MJPEG request is unbounded. Release the robot connection when the
      // viewer leaves, including during camera switches and browser refreshes.
      res.on('close', () => response.destroy());
    });
    proxy.on('error', () => {
      if (!res.headersSent) {
        res.statusCode = 502;
        res.setHeader('Content-Type', 'application/json');
        res.end(JSON.stringify({ error: 'Camera server unavailable from GCS' }));
      } else res.destroy();
    });
    res.on('close', () => proxy.destroy());
    req.on('aborted', () => proxy.destroy());
    req.pipe(proxy);
  });
};
