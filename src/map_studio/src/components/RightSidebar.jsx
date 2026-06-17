import React, { useState } from 'react';

const RightSidebar = ({ diagnostics = {}, activeView, selectedItem }) => {
  const [activeTab, setActiveTab] = useState('state_machine');

  const {
    state = 'IDLE',
    cross_track_error = 0.0,
    last_tag = '',
    nav_vel = { linear: 0, angular: 0 },
    fault_log = [],
  } = diagnostics;

  // CTE gauge: ±50mm range
  const cteMax = 50.0;
  const cteClamp = Math.max(-cteMax, Math.min(cteMax, cross_track_error * 1000)); // m -> mm
  const ctePct = ((cteClamp + cteMax) / (2 * cteMax)) * 100; // 0-100%, 50% = center

  if (activeView === 'map_studio') {
    return (
      <aside style={{
        position: 'fixed',
        top: 'var(--header-height)',
        right: 0,
        height: 'calc(100vh - var(--header-height))',
        width: 'var(--sidebar-right-width)',
        background: 'var(--color-surface-container-low)',
        borderLeft: '2px solid var(--color-outline-variant)',
        display: 'flex',
        flexDirection: 'column',
        zIndex: 40,
      }}>
        <div style={{ padding: 'var(--space-md) var(--space-lg)', borderBottom: '1px solid var(--color-outline-variant)' }}>
          <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)' }}>PROPERTIES</div>
        </div>

        <div style={{ flex: 1, padding: 'var(--space-md) var(--space-lg)', overflowY: 'auto' }} className="custom-scroll">
          {selectedItem ? (
            <div style={{ display: 'flex', flexDirection: 'column', gap: 'var(--space-md)' }}>
              <div>
                <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', fontSize: 10, marginBottom: 4 }}>ID</div>
                <div className="font-data-mono" style={{ color: 'var(--color-on-surface)', fontSize: 14 }}>{selectedItem.id}</div>
              </div>
              <div>
                <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', fontSize: 10, marginBottom: 4 }}>TYPE</div>
                <div style={{ color: 'var(--color-on-surface)', fontSize: 14 }}>{selectedItem.type || 'WAYPOINT'}</div>
              </div>
              <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 'var(--space-sm)' }}>
                <div>
                  <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', fontSize: 10, marginBottom: 4 }}>X (m)</div>
                  <input type="number" defaultValue={selectedItem.x} style={{ width: '100%', padding: '6px', background: 'var(--color-surface-container)', border: '1px solid var(--color-outline-variant)', color: 'var(--color-on-surface)' }} />
                </div>
                <div>
                  <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', fontSize: 10, marginBottom: 4 }}>Y (m)</div>
                  <input type="number" defaultValue={selectedItem.y} style={{ width: '100%', padding: '6px', background: 'var(--color-surface-container)', border: '1px solid var(--color-outline-variant)', color: 'var(--color-on-surface)' }} />
                </div>
              </div>
              {selectedItem.tag_id && (
                <div>
                  <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', fontSize: 10, marginBottom: 4 }}>RFID TAG ID</div>
                  <input type="text" defaultValue={selectedItem.tag_id} style={{ width: '100%', padding: '6px', background: 'var(--color-surface-container)', border: '1px solid var(--color-outline-variant)', color: 'var(--color-on-surface)' }} />
                </div>
              )}
            </div>
          ) : (
            <div style={{ height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', color: 'var(--color-on-surface-variant)', fontSize: 12, textAlign: 'center' }}>
              NO ITEM SELECTED<br/>Click on a vertex or edge on the canvas to view its properties.
            </div>
          )}
        </div>
      </aside>
    );
  }

  return (
    <aside style={{
      position: 'fixed',
      top: 'var(--header-height)',
      right: 0,
      height: 'calc(100vh - var(--header-height))',
      width: 'var(--sidebar-right-width)',
      background: 'var(--color-surface-container-low)',
      borderLeft: '2px solid var(--color-outline-variant)',
      display: 'flex',
      flexDirection: 'column',
      padding: 'var(--space-panel-padding)',
      zIndex: 40,
      overflowY: 'auto',
    }} className="custom-scroll">
      {/* Title */}
      <div style={{ marginBottom: 'var(--space-lg)' }}>
        <p className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', opacity: 0.7, margin: '0 0 4px 0' }}>
          REAL-TIME
        </p>
        <p className="font-headline-sm" style={{ color: 'var(--color-primary)', margin: 0 }}>
          DIAGNOSTICS
        </p>
      </div>

      {/* Tab Bar */}
      <div className="tab-bar">
        <button
          className={`tab-item ${activeTab === 'state_machine' ? 'tab-item-active' : ''}`}
          onClick={() => setActiveTab('state_machine')}
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <circle cx="18" cy="18" r="3" /><circle cx="6" cy="6" r="3" /><path d="M13 6h3a2 2 0 012 2v7" /><path d="M6 9v12" />
          </svg>
          STATE MACHINE
        </button>
        <button
          className={`tab-item ${activeTab === 'telemetry' ? 'tab-item-active' : ''}`}
          onClick={() => setActiveTab('telemetry')}
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M22 12h-4l-3 9L9 3l-3 9H2" />
          </svg>
          TELEMETRY
        </button>
      </div>

      {/* Content */}
      <div style={{ display: 'flex', flexDirection: 'column', gap: 'var(--space-lg)', flex: 1, overflowY: 'auto' }} className="custom-scroll">
        {activeTab === 'state_machine' ? (
          <>
            {/* Current State Box */}
            <div className="panel">
              <p className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', margin: '0 0 var(--space-sm) 0' }}>
                CURRENT STATE
              </p>
              <div className="state-display">
                <span className="state-display-text">{state}</span>
              </div>
            </div>

            {/* Sensor Feed: Cross-Track Error Gauge */}
            <div className="panel" style={{ borderTop: '2px solid var(--color-secondary)' }}>
              <p className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', margin: '0 0 var(--space-md) 0' }}>
                SENSOR FEED
              </p>
              <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                <span className="font-data-mono" style={{ marginBottom: 'var(--space-sm)' }}>
                  CROSS-TRACK ERROR
                </span>

                {/* Horizontal Needle Gauge */}
                <div style={{ width: '100%', position: 'relative', height: 40, marginTop: 8 }}>
                  {/* Base Axis */}
                  <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: 0,
                    right: 0,
                    height: 2,
                    background: 'var(--color-outline-variant)',
                    transform: 'translateY(-50%)',
                  }} />

                  {/* Left Critical Zone */}
                  <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: 0,
                    width: '15%',
                    height: 4,
                    background: 'var(--color-estop)',
                    transform: 'translateY(-50%)',
                  }} />

                  {/* Right Critical Zone */}
                  <div style={{
                    position: 'absolute',
                    top: '50%',
                    right: 0,
                    width: '15%',
                    height: 4,
                    background: 'var(--color-estop)',
                    transform: 'translateY(-50%)',
                  }} />

                  {/* Left Tick */}
                  <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: 0,
                    width: 2,
                    height: 16,
                    background: 'var(--color-outline-variant)',
                    transform: 'translateY(-50%)',
                  }} />

                  {/* Center Tick */}
                  <div style={{
                    position: 'absolute',
                    top: '50%',
                    left: '50%',
                    width: 2,
                    height: 24,
                    background: 'var(--color-secondary)',
                    transform: 'translate(-50%, -50%)',
                  }} />

                  {/* Right Tick */}
                  <div style={{
                    position: 'absolute',
                    top: '50%',
                    right: 0,
                    width: 2,
                    height: 16,
                    background: 'var(--color-outline-variant)',
                    transform: 'translateY(-50%)',
                  }} />

                  {/* Needle */}
                  <div style={{
                    position: 'absolute',
                    top: 2,
                    left: `${ctePct}%`,
                    transform: 'translateX(-50%)',
                    width: 0,
                    height: 0,
                    borderLeft: '6px solid transparent',
                    borderRight: '6px solid transparent',
                    borderTop: '10px solid var(--color-primary-container)',
                    transition: 'left 0.1s ease-out',
                  }} />
                </div>

                {/* Labels */}
                <div style={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  width: '100%',
                  marginTop: 'var(--space-sm)',
                }}>
                  <span className="font-data-mono" style={{ fontSize: 10, color: 'var(--color-on-surface-variant)' }}>-50mm</span>
                  <span className="font-data-mono" style={{ fontSize: 10, color: 'var(--color-secondary)', fontWeight: 700 }}>0mm</span>
                  <span className="font-data-mono" style={{ fontSize: 10, color: 'var(--color-on-surface-variant)' }}>+50mm</span>
                </div>

                {/* Numeric Value */}
                <div className="font-data-mono" style={{
                  marginTop: 'var(--space-sm)',
                  fontSize: 'var(--fs-body-lg)',
                  fontWeight: 700,
                  color: 'var(--color-primary)',
                }}>
                  {cteClamp >= 0 ? '+' : ''}{cteClamp.toFixed(1)} mm
                </div>
              </div>
            </div>

            {/* Fault Log */}
            <div className="panel" style={{ flex: 1, display: 'flex', flexDirection: 'column', minHeight: 180 }}>
              <p className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', margin: '0 0 var(--space-sm) 0' }}>
                FAULT LOG
              </p>
              <div className="fault-log-container custom-scroll" style={{ flex: 1 }}>
                {fault_log.length > 0 ? (
                  fault_log.map((entry, idx) => (
                    <div key={idx} className="fault-log-entry">
                      <span className={
                        entry.level === 'error' ? 'fault-log-ts-error' :
                        entry.level === 'warn'  ? 'fault-log-ts-warn'  :
                        'fault-log-ts'
                      }>
                        [{entry.time || '??:??:??'}]
                      </span>
                      {' '}{entry.message}
                    </div>
                  ))
                ) : (
                  <>
                    <div className="fault-log-entry"><span className="fault-log-ts">[--:--:--]</span> SYSTEM READY</div>
                    <div className="fault-log-entry"><span className="fault-log-ts">[--:--:--]</span> AWAITING DATA...</div>
                  </>
                )}
              </div>
            </div>
          </>
        ) : (
          /* Telemetry Tab */
          <>
            {/* Kinematics */}
            <div className="panel" style={{ borderTop: '2px solid var(--color-secondary)' }}>
              <p className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', margin: '0 0 var(--space-md) 0' }}>
                KINEMATICS
              </p>
              <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 'var(--space-md)' }}>
                <div>
                  <p className="font-label-caps" style={{ fontSize: 10, color: 'var(--color-on-surface-variant)', margin: '0 0 4px 0', opacity: 0.7 }}>
                    LIN_X (m/s)
                  </p>
                  <p className="font-headline-sm" style={{ color: 'var(--color-on-surface)', margin: 0 }}>
                    {(nav_vel.linear || 0).toFixed(3)}
                  </p>
                </div>
                <div>
                  <p className="font-label-caps" style={{ fontSize: 10, color: 'var(--color-on-surface-variant)', margin: '0 0 4px 0', opacity: 0.7 }}>
                    ANG_Z (rad/s)
                  </p>
                  <p className="font-headline-sm" style={{ color: 'var(--color-on-surface)', margin: 0 }}>
                    {(nav_vel.angular || 0).toFixed(3)}
                  </p>
                </div>
              </div>
            </div>

            {/* Last Tag */}
            <div className="panel">
              <p className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', margin: '0 0 var(--space-sm) 0' }}>
                LAST TAG DETECTED
              </p>
              <div className="state-display" style={{ padding: 'var(--space-sm) var(--space-md)' }}>
                <span className="font-data-mono" style={{ color: 'var(--color-primary)', fontSize: 16 }}>
                  {last_tag || '—'}
                </span>
              </div>
            </div>

            {/* Nav Velocity Details */}
            <div className="panel">
              <p className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', margin: '0 0 var(--space-md) 0' }}>
                NAVIGATION VELOCITY
              </p>
              <div style={{ display: 'flex', flexDirection: 'column', gap: 'var(--space-sm)' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                  <span className="font-data-mono" style={{ fontSize: 12, color: 'var(--color-on-surface-variant)' }}>TARGET SPEED</span>
                  <span className="font-data-mono" style={{ color: 'var(--color-on-surface)' }}>
                    {Math.abs(nav_vel.linear || 0).toFixed(3)} m/s
                  </span>
                </div>
                <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                  <span className="font-data-mono" style={{ fontSize: 12, color: 'var(--color-on-surface-variant)' }}>TURN RATE</span>
                  <span className="font-data-mono" style={{ color: 'var(--color-on-surface)' }}>
                    {(nav_vel.angular || 0).toFixed(3)} rad/s
                  </span>
                </div>
                <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                  <span className="font-data-mono" style={{ fontSize: 12, color: 'var(--color-on-surface-variant)' }}>CTE</span>
                  <span className="font-data-mono" style={{
                    color: Math.abs(cteClamp) > 35 ? 'var(--color-estop)' :
                           Math.abs(cteClamp) > 15 ? 'var(--color-primary)' : 'var(--color-secondary)'
                  }}>
                    {cteClamp >= 0 ? '+' : ''}{cteClamp.toFixed(1)} mm
                  </span>
                </div>
              </div>
            </div>
          </>
        )}
      </div>
    </aside>
  );
};

export default RightSidebar;
