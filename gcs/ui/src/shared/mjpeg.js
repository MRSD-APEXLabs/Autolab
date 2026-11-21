// Incremental parser for an MJPEG (multipart/x-mixed-replace) HTTP body.
// Feed it byte chunks as they arrive; it returns the newest complete JPEG in the chunk (older ones
// are skipped, so a slow viewer drops frames instead of falling behind), or null.
// Parts are cut by their Content-Length header; parts without one fall back to the JPEG end marker.

const MAX_BUFFER = 16 * 1024 * 1024;

function indexOfHeaderEnd(buf, from) {
  for (let i = from; i + 3 < buf.length; i++) {
    if (buf[i] === 13 && buf[i + 1] === 10 && buf[i + 2] === 13 && buf[i + 3] === 10) return i;
  }
  return -1;
}

function contentLength(buf, start, end) {
  let text = '';
  for (let i = start; i < end; i++) text += String.fromCharCode(buf[i]);
  const m = /content-length:\s*(\d+)/i.exec(text);
  return m ? Number(m[1]) : null;
}

function indexOfJpegEnd(buf, from) {
  for (let i = from; i + 1 < buf.length; i++) if (buf[i] === 0xff && buf[i + 1] === 0xd9) return i + 2;
  return -1;
}

export function createMjpegParser() {
  let buf = new Uint8Array(0);
  return function parse(chunk) {
    if (chunk && chunk.length) {
      const merged = new Uint8Array(buf.length + chunk.length);
      merged.set(buf);
      merged.set(chunk, buf.length);
      buf = merged;
    }
    let pos = 0;
    let newest = null;
    for (;;) {
      const he = indexOfHeaderEnd(buf, pos);
      if (he < 0) break;
      const body = he + 4;
      const len = contentLength(buf, pos, he);
      let end;
      if (len !== null) {
        if (buf.length < body + len) break;
        end = body + len;
      } else {
        end = indexOfJpegEnd(buf, body);
        if (end < 0) break;
      }
      newest = buf.slice(body, end);
      pos = end;
    }
    if (pos) buf = buf.slice(pos);
    if (buf.length > MAX_BUFFER) buf = new Uint8Array(0);           // garbage: start over
    return newest;
  };
}
