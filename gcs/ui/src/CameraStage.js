import React, { useEffect, useMemo, useRef, useState } from 'react';
import MjpegView, { uniqueUrl } from './shared/MjpegView';
import { MqttProvider, useMqttContext } from './shared/useMqtt';
import useCameraHub from './shared/useCameraHub';
import usePerception from './shared/usePerception';
import useStreamStale from './shared/useStreamStale';
import { STATION_CAMERAS, cameraLabel, deckFetchUrl, deckStreamUrl, streamUrl } from './shared/cameras';

// 1x1 transparent GIF. Pointing a stream <img> here aborts its MJPEG request (see CameraFeed).
const BLANK_SRC = 'data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';
const RETRY_MS = 2000;      // after a stream error
const STALE_RETRY_MS = 3000; // after /health has said "no frames" this long

function Stream({ src, alt, onError }) {
  const ref = useRef(null);
  useEffect(() => {
    const img = ref.current;
    img.src = src;
    return () => { img.src = BLANK_SRC; };
  }, [src]);
  return <img ref={ref} alt={alt} onError={onError} />;
}

// A robot camera stream. image_stream_server doesn't allow cross-origin stream reads, so this
// stays an <img>, but every mount and every retry gets a URL never used before (WebKit hands an
// <img> with a reused URL the old, possibly dead stream), errors retry by themselves, and a
// stream /health reports as stale is reopened.
function RobotFeed({ src, alt, stale, offText }) {
  const [attempt, setAttempt] = useState(0);
  const [failed, setFailed] = useState(false);
  const url = useMemo(() => (src ? uniqueUrl(src) : null), [src, attempt]); // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => { setFailed(false); }, [src]);
  useEffect(() => {
    if (!failed) return undefined;
    const t = setTimeout(() => { setFailed(false); setAttempt((n) => n + 1); }, RETRY_MS);
    return () => clearTimeout(t);
  }, [failed]);
  useEffect(() => {
    if (!stale || failed) return undefined;
    const t = setTimeout(() => setAttempt((n) => n + 1), STALE_RETRY_MS);
    return () => clearTimeout(t);
  }, [stale, failed, attempt]);
  if (!src) return <div className="cs-nosig"><b>No signal</b><small>{offText}</small></div>;
  return (
    <>
      {failed ? (
        <div className="cs-nosig"><b>No signal</b><small>{offText} · retrying</small></div>
      ) : (
        <Stream key={url} src={url} alt={alt} onError={() => setFailed(true)} />
      )}
      {!failed && stale && <div className="cs-stale">No frames · reconnecting</div>}
    </>
  );
}

function DetectionLine({ camera }) {
  const { tags, plates } = usePerception(camera);
  const target = plates.find((p) => p.plannerTarget);
  return (
    <div className="cs-detect" data-testid="detection-strip">
      <span>Tags <b>{tags.length ? tags.map((t) => t.id).join(' · ') : '—'}</b></span>
      <span>Well plate <b>{target ? `${target.conf.toFixed(2)} @ ${target.distance.toFixed(2)} m` : '—'}</b></span>
    </div>
  );
}

// The run's camera block: a large view plus a strip of the other cameras. Purely a viewer: which
// robot camera streams is decided by the camera hub (debug console), never from here. Clicking a
// thumbnail only changes what this screen shows large.
function CameraStageBody({ station }) {
  const { connected } = useMqttContext();
  const hub = useCameraHub();
  const [kind, setKind] = useState('annotated');
  const [main, setMain] = useState('robot');

  const robotCam = hub.active;
  const hubDown = hub.reachable === false;
  const robotSrc = (k) => (robotCam && !hubDown ? streamUrl(k, robotCam) : null);
  const robotOff = !connected ? 'Not connected to the GCS' : hubDown ? 'Camera hub unreachable' : 'No robot camera streaming';
  const stale = useStreamStale(main === 'robot' && robotSrc(kind) ? `${kind}/${robotCam}` : null);

  const views = [
    { id: 'robot', label: 'Robot', sub: cameraLabel(robotCam) },
    ...STATION_CAMERAS.map((c) => ({ id: c.id, label: c.short, sub: c.label })),
  ];
  const mainView = views.find((v) => v.id === main) ?? views[0];
  const isRobot = main === 'robot';

  return (
    <section className="cs" aria-label="Cameras">
      <div className="cs-left">
      <div className="cs-main">
        {isRobot ? (
          <RobotFeed src={robotSrc(kind)} alt={`Robot camera ${kind}`} stale={stale} offText={robotOff} />
        ) : (
          <MjpegView url={deckStreamUrl(main)} fetchUrl={deckFetchUrl(main)} alt={`${mainView.sub} camera`} />
        )}
        <div className="cs-tag">
          <span className="cs-live" />
          <b>{mainView.label}</b>
          <span>{mainView.sub}</span>
        </div>
        <div className="cs-seg" role="radiogroup" aria-label="View">
          {['annotated', 'raw'].map((k) => (
            <button
              key={k} type="button" role="radio" aria-checked={isRobot && kind === k}
              className={isRobot && kind === k ? 'on' : ''} disabled={!isRobot}
              onClick={() => setKind(k)}
              title={isRobot ? undefined : 'Station cameras have no annotated view'}
            >
              {k === 'annotated' ? 'Annotated' : 'Raw'}
            </button>
          ))}
        </div>
      </div>
      {isRobot && robotCam && <DetectionLine camera={robotCam} />}
      </div>
      <div className="cs-strip">
        {views.filter((v) => v.id !== main).map((v) => (
          <button
            key={v.id} type="button" className={`cs-thumb ${station === v.id ? 'here' : ''}`}
            onClick={() => setMain(v.id)} aria-label={`Show ${v.sub} large`}
          >
            {v.id === 'robot' ? (
              <RobotFeed src={robotSrc('raw')} alt="Robot camera" offText={robotOff} />
            ) : (
              <MjpegView url={deckStreamUrl(v.id)} fetchUrl={deckFetchUrl(v.id)} alt={`${v.sub} camera`} />
            )}
            <span className="cs-thumb-label">{v.label}{station === v.id && <em>current step</em>}</span>
          </button>
        ))}
      </div>
    </section>
  );
}

export default function CameraStage(props) {
  return (
    <MqttProvider>
      <CameraStageBody {...props} />
    </MqttProvider>
  );
}
