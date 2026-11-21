import { TextEncoder } from 'util';
import { createMjpegParser } from './mjpeg';

const enc = (s) => new TextEncoder().encode(s);
const cat = (...parts) => {
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let o = 0;
  parts.forEach((p) => { out.set(p, o); o += p.length; });
  return out;
};
const jpeg = (n) => new Uint8Array([0xff, 0xd8, n, n, 0xff, 0xd9]);
const part = (bytes) => cat(enc(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${bytes.length}\r\n\r\n`), bytes, enc('\r\n'));

test('returns the newest complete frame of a chunk', () => {
  const parse = createMjpegParser();
  expect(Array.from(parse(cat(part(jpeg(1)), part(jpeg(2)))))).toEqual(Array.from(jpeg(2)));
});

test('frames split across chunks are reassembled', () => {
  const parse = createMjpegParser();
  const all = cat(part(jpeg(7)), part(jpeg(8)));
  expect(parse(all.slice(0, 20))).toBeNull();
  const firstEnd = part(jpeg(7)).length;
  expect(Array.from(parse(all.slice(20, firstEnd)))).toEqual(Array.from(jpeg(7)));
  expect(parse(all.slice(firstEnd, firstEnd + 30))).toBeNull();
  expect(Array.from(parse(all.slice(firstEnd + 30)))).toEqual(Array.from(jpeg(8)));
});

test('a frame that contains end-of-image bytes is cut by its length, not by the marker', () => {
  const parse = createMjpegParser();
  const tricky = new Uint8Array([0xff, 0xd8, 0xff, 0xd9, 5, 0xff, 0xd9]);
  expect(Array.from(parse(part(tricky)))).toEqual(Array.from(tricky));
});

test('parts without a length fall back to the JPEG end marker', () => {
  const parse = createMjpegParser();
  const noLen = cat(enc('--frame\r\nContent-Type: image/jpeg\r\n\r\n'), jpeg(3), enc('\r\n'));
  expect(Array.from(parse(noLen))).toEqual(Array.from(jpeg(3)));
});