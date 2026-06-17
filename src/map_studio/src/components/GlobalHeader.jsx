import React from 'react';

const GlobalHeader = ({ activeMapName, onEstop, onMapSelect, maps = [], editMode, onToggleEdit, simStatus, onOpenSimModal }) => {
  return (
    <header style={{
      position: 'fixed',
      top: 0,
      left: 0,
      width: '100%',
      height: 'var(--header-height)',
      display: 'flex',
      justifyContent: 'space-between',
      alignItems: 'center',
      padding: '0 var(--space-lg)',
      background: 'var(--color-surface)',
      borderBottom: '2px solid var(--color-outline-variant)',
      zIndex: 50,
    }}>
      {/* Left: Branding */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 'var(--space-md)' }}>
        <span 
          className="font-headline-md" 
          onClick={() => window.dispatchEvent(new CustomEvent('navigate', { detail: 'mission_control' }))}
          style={{ color: 'var(--color-primary)', letterSpacing: '0.05em', fontWeight: 600, cursor: 'pointer' }}
        >
          AGV-OS
        </span>

        <div className="divider-v" style={{ height: 24, background: 'var(--color-outline-variant)', width: 2 }} />

        <span className="font-label-caps" style={{ color: 'var(--color-on-surface-variant)', letterSpacing: '0.1em' }}>
          MAP STUDIO
        </span>
      </div>

      {/* Right: Actions + Icons + E-Stop */}
      <div style={{ display: 'flex', alignItems: 'center', gap: 'var(--space-md)' }}>
        
        {/* Save Revision Button */}
        <button 
          className="btn btn-secondary" 
          onClick={() => console.log('Save Revision')}
          style={{ fontSize: 'var(--fs-label-caps)', display: 'flex', alignItems: 'center', gap: 8 }}
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"></path>
            <polyline points="17 21 17 13 7 13 7 21"></polyline>
            <polyline points="7 3 7 8 15 8"></polyline>
          </svg>
          SAVE REVISION
        </button>

        {/* Start / Restart Simulation Button (Replacing Deploy to Fleet) */}
        {simStatus === 'running' ? (
          <button 
            className="btn" 
            onClick={() => onOpenSimModal(true)}
            style={{ 
              fontSize: 'var(--fs-label-caps)', 
              background: 'var(--color-primary)', 
              color: 'var(--color-surface)',
              border: 'none',
              display: 'flex', alignItems: 'center', gap: 8
            }}
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M21.5 2v6h-6M21.34 15.57a10 10 0 1 1-.59-9.21l-3.23 3.63" />
            </svg>
            RESTART SIM
          </button>
        ) : (
          <button 
            className="btn" 
            onClick={() => onOpenSimModal(false)}
            style={{ 
              fontSize: 'var(--fs-label-caps)', 
              background: 'var(--color-primary)', 
              color: 'var(--color-surface)',
              border: 'none',
              display: 'flex', alignItems: 'center', gap: 8
            }}
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <polygon points="5 3 19 12 5 21 5 3"></polygon>
            </svg>
            START SIM
          </button>
        )}

        <div className="divider-v" style={{ height: 32 }} />

        {/* E-STOP */}
        <button
          className="btn btn-emergency"
          onClick={onEstop}
          style={{ padding: '8px 24px', fontSize: 14, background: 'transparent', color: 'var(--color-error)', borderColor: 'var(--color-error)' }}
        >
          ⚠ E-STOP
        </button>

        <div className="divider-v" style={{ height: 32 }} />

        {/* Notification Bell */}
        <button className="btn-icon" title="Notifications">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--color-on-surface-variant)" strokeWidth="2">
            <path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9" />
            <path d="M13.73 21a2 2 0 0 1-3.46 0" />
          </svg>
        </button>

        {/* Settings */}
        <button className="btn-icon" title="Settings">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="var(--color-on-surface-variant)" strokeWidth="2">
            <circle cx="12" cy="12" r="3" />
            <path d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42" />
          </svg>
        </button>
      </div>
    </header>
  );
};

export default GlobalHeader;
