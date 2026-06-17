import React from 'react';

const NAV_ITEMS = [
  { key: 'mission_control', label: 'MISSION CONTROL', icon: 'M' },
  { key: 'map_studio',      label: 'MAP STUDIO',      icon: 'S' },
  { key: 'diagnostics',     label: 'DIAGNOSTICS',     icon: 'D' },
];

const NavIcon = ({ type }) => {
  const icons = {
    M: ( // precision_manufacturing
      <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <path d="M12 2L2 7l10 5 10-5-10-5z" /><path d="M2 17l10 5 10-5" /><path d="M2 12l10 5 10-5" />
      </svg>
    ),
    S: ( // map
      <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <polygon points="1 6 1 22 8 18 16 22 23 18 23 2 16 6 8 2 1 6" /><line x1="8" y1="2" x2="8" y2="18" /><line x1="16" y1="6" x2="16" y2="22" />
      </svg>
    ),
    D: ( // analytics
      <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round">
        <line x1="18" y1="20" x2="18" y2="10" /><line x1="12" y1="20" x2="12" y2="4" /><line x1="6" y1="20" x2="6" y2="14" />
      </svg>
    )
  };
  return icons[type] || null;
};

const LeftSidebar = ({
  activeView,
  onViewChange,
  mission,
  onNewMission,
  activeMap,
  onNavigateToVertex,
  faultCount = 0,
}) => {
  
  if (activeView === 'map_studio') {
    return (
      <nav style={{
        position: 'fixed',
        top: 'var(--header-height)',
        left: 0,
        height: 'calc(100vh - var(--header-height))',
        width: 'var(--sidebar-left-width)',
        background: 'var(--color-surface-container-low)',
        borderRight: '2px solid var(--color-outline-variant)',
        display: 'flex',
        flexDirection: 'column',
        zIndex: 40,
      }}>
        {/* Map Metadata Section */}
        <div style={{ padding: 'var(--space-md) var(--space-lg)', borderBottom: '1px solid var(--color-outline-variant)' }}>
          <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', marginBottom: 'var(--space-md)' }}>MAP METADATA</div>
          
          <div style={{ marginBottom: 'var(--space-sm)' }}>
            <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', fontSize: 10 }}>NAME</div>
            <div style={{ color: 'var(--color-on-surface)', fontSize: 14 }}>Assembly Line 1</div>
          </div>
          
          <div style={{ marginBottom: 'var(--space-sm)' }}>
            <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', fontSize: 10 }}>VERSION</div>
            <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
              <span style={{ color: 'var(--color-on-surface)', fontSize: 14 }}>v2.4.1</span>
              <span style={{ background: 'var(--color-surface-container-highest)', color: 'var(--color-on-surface)', padding: '2px 6px', fontSize: 10, borderRadius: 4, fontWeight: 600 }}>STABLE</span>
            </div>
          </div>
          
          <div>
            <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', fontSize: 10 }}>GRID RESOLUTION</div>
            <div style={{ color: 'var(--color-on-surface)', fontSize: 14 }}>0.05m / px</div>
          </div>
        </div>

        {/* Layers Section */}
        <div style={{ padding: 'var(--space-md) 0', flex: 1, overflowY: 'auto' }}>
          <div style={{ padding: '0 var(--space-lg)', display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 'var(--space-sm)' }}>
            <div className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)' }}>LAYERS</div>
            <span style={{ color: 'var(--color-on-surface-variant)', cursor: 'pointer', fontSize: 16 }}>+</span>
          </div>

          <div style={{ display: 'flex', flexDirection: 'column' }}>
            <div style={{ padding: '10px var(--space-lg)', display: 'flex', alignItems: 'center', justifyContent: 'space-between', color: 'var(--color-on-surface)' }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><rect x="3" y="3" width="18" height="18" rx="2" ry="2"></rect><line x1="3" y1="9" x2="21" y2="9"></line><line x1="3" y1="15" x2="21" y2="15"></line><line x1="9" y1="3" x2="9" y2="21"></line><line x1="15" y1="3" x2="15" y2="21"></line></svg>
                <span style={{ fontSize: 14 }}>Grid</span>
              </div>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--color-on-surface-variant)" strokeWidth="2"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"></path><circle cx="12" cy="12" r="3"></circle></svg>
            </div>
            
            <div style={{ padding: '10px var(--space-lg)', display: 'flex', alignItems: 'center', justifyContent: 'space-between', color: 'var(--color-surface)', background: 'var(--color-primary)' }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M22 12h-4l-3 9L9 3l-3 9H2"></path></svg>
                <span style={{ fontSize: 14 }}>Paths</span>
              </div>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"></path><circle cx="12" cy="12" r="3"></circle></svg>
            </div>

            <div style={{ padding: '10px var(--space-lg)', display: 'flex', alignItems: 'center', justifyContent: 'space-between', color: 'var(--color-on-surface)' }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M20.59 13.41l-7.17 7.17a2 2 0 0 1-2.83 0L2 12V2h10l8.59 8.59a2 2 0 0 1 0 2.82z"></path><line x1="7" y1="7" x2="7.01" y2="7"></line></svg>
                <span style={{ fontSize: 14 }}>Tags</span>
              </div>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--color-on-surface-variant)" strokeWidth="2"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"></path><circle cx="12" cy="12" r="3"></circle></svg>
            </div>

            <div style={{ padding: '10px var(--space-lg)', display: 'flex', alignItems: 'center', justifyContent: 'space-between', color: 'var(--color-on-surface-variant)' }}>
              <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
                <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><circle cx="12" cy="12" r="10"></circle><line x1="15" y1="9" x2="9" y2="15"></line><line x1="9" y1="9" x2="15" y2="15"></line></svg>
                <span style={{ fontSize: 14 }}>Zones (Exclusion)</span>
              </div>
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24"></path><line x1="1" y1="1" x2="23" y2="23"></line></svg>
            </div>
          </div>
        </div>

        {/* Export YAML Bottom */}
        <div style={{ padding: 'var(--space-md) var(--space-lg)', borderTop: '1px solid var(--color-outline-variant)' }}>
          <button className="btn" style={{ width: '100%', background: 'transparent', border: '1px solid var(--color-outline-variant)', color: 'var(--color-on-surface)' }}>
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" style={{ marginRight: 8 }}><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path><polyline points="7 10 12 15 17 10"></polyline><line x1="12" y1="15" x2="12" y2="3"></line></svg>
            EXPORT YAML
          </button>
        </div>
      </nav>
    );
  }

  return (
    <nav style={{
      position: 'fixed',
      top: 'var(--header-height)',
      left: 0,
      height: 'calc(100vh - var(--header-height))',
      width: 'var(--sidebar-left-width)',
      background: 'var(--color-surface-container-low)',
      borderRight: '2px solid var(--color-outline-variant)',
      display: 'flex',
      flexDirection: 'column',
      padding: 'var(--space-panel-padding) 0',
      zIndex: 40,
      overflowY: 'auto',
    }} className="custom-scroll">


      {/* New Mission Button */}
      <div style={{ padding: '0 var(--space-md)', marginBottom: 'var(--space-lg)' }}>
        <button
          className="btn btn-primary"
          onClick={onNewMission}
          style={{ width: '100%', padding: '12px 16px' }}
        >
          + NEW MISSION
        </button>
      </div>

      {/* Navigation Tabs */}
      <div style={{ marginBottom: 'var(--space-lg)' }}>
        {NAV_ITEMS.map(item => (
          <button
            key={item.key}
            className={`nav-tab ${activeView === item.key ? 'nav-tab-active' : ''}`}
            onClick={() => onViewChange(item.key)}
            style={{ padding: '10px var(--space-md)' }}
          >
            <NavIcon type={item.icon} />
            {item.label}
          </button>
        ))}
      </div>

      {/* Mission Builder (visible only in mission_control view) */}
      {activeView === 'mission_control' && (
        <div style={{ padding: '0 var(--space-md)', flex: 1, display: 'flex', flexDirection: 'column' }}>
          <div className="panel" style={{ flex: 1, display: 'flex', flexDirection: 'column' }}>
            <div className="panel-header" style={{ borderTop: '2px solid var(--color-text-muted)' }}>
              MISSION BUILDER
            </div>

            {/* Draggable Action Blocks */}
            <div style={{ display: 'flex', flexDirection: 'column', gap: 'var(--space-sm)', marginBottom: 'var(--space-lg)' }}>
              <div className="mission-block">
                <span style={{ color: 'var(--color-on-surface-variant)', fontSize: 14 }}>⠿</span>
                [MOVE TO TAG]
              </div>
              <div className="mission-block">
                <span style={{ color: 'var(--color-on-surface-variant)', fontSize: 14 }}>⠿</span>
                [DOCK]
              </div>
              <div className="mission-block">
                <span style={{ color: 'var(--color-on-surface-variant)', fontSize: 14 }}>⠿</span>
                [WAIT 30S]
              </div>
            </div>

            {/* Active Mission Queue */}
            <div style={{ marginTop: 'auto' }}>
              <p className="font-label-caps" style={{ color: 'var(--color-on-surface)', margin: '0 0 var(--space-sm) 0' }}>
                ACTIVE QUEUE
              </p>
              <div style={{
                background: 'var(--color-surface-container-lowest)',
                border: '1px solid var(--color-outline-variant)',
                borderRadius: 'var(--radius-default)',
                padding: 'var(--space-sm)',
                maxHeight: 140,
                overflowY: 'auto',
              }} className="custom-scroll">
                {mission && mission.actions && mission.actions.length > 0 ? (
                  mission.actions.map((action, idx) => (
                    <div
                      key={idx}
                      className={idx === mission.currentIndex ? 'mission-block-active' : ''}
                      style={{
                        fontFamily: 'var(--font-mono)',
                        fontSize: 12,
                        color: idx === mission.currentIndex ? 'var(--color-primary)' : 'var(--color-on-surface-variant)',
                        padding: '4px',
                        borderRadius: 'var(--radius-sm)',
                        ...(idx === mission.currentIndex ? {
                          background: 'rgba(115, 92, 0, 0.2)',
                          border: '1px solid var(--color-primary)',
                        } : {}),
                      }}
                    >
                      {'>'} Action {idx + 1}: {action.type} {action.target ? `(${action.target})` : ''}
                    </div>
                  ))
                ) : (
                  <div style={{
                    fontFamily: 'var(--font-mono)',
                    fontSize: 11,
                    color: 'var(--color-on-surface-variant)',
                    textAlign: 'center',
                    padding: 8,
                  }}>
                    NO ACTIVE MISSION
                  </div>
                )}
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Footer */}
      <div style={{
        marginTop: 'auto',
        padding: 'var(--space-lg) var(--space-md) 0',
        borderTop: '1px solid var(--color-outline-variant)',
        display: 'flex',
        flexDirection: 'column',
        gap: 'var(--space-sm)',
      }}>
        <button className="nav-tab" style={{ color: 'var(--color-on-surface-variant)', padding: '6px var(--space-md)' }}>
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8">
            <circle cx="12" cy="12" r="3" /><path d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42" />
          </svg>
          <span style={{ fontSize: 10 }}>SYSTEMS</span>
        </button>
        <button className="nav-tab" style={{ color: faultCount > 0 ? 'var(--color-error)' : 'var(--color-on-surface-variant)', padding: '6px var(--space-md)' }}>
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.8">
            <path d="M10.29 3.86L1.82 18a2 2 0 001.71 3h16.94a2 2 0 001.71-3L13.71 3.86a2 2 0 00-3.42 0z" /><line x1="12" y1="9" x2="12" y2="13" /><line x1="12" y1="17" x2="12.01" y2="17" />
          </svg>
          <span style={{ fontSize: 10 }}>FAULTS ({faultCount})</span>
        </button>
      </div>
    </nav>
  );
};

export default LeftSidebar;
