import React, { useState } from 'react';

const SimulationModal = ({ isOpen, onClose, onStart, isRestart = false }) => {
  const [headless, setHeadless] = useState(false);

  if (!isOpen) return null;

  return (
    <div style={{
      position: 'fixed', top: 0, left: 0, right: 0, bottom: 0,
      background: 'rgba(0, 0, 0, 0.75)', zIndex: 1000,
      display: 'flex', alignItems: 'center', justifyContent: 'center'
    }}>
      <div style={{
        background: 'var(--color-surface)',
        border: '2px solid var(--color-outline-variant)',
        width: 400,
        padding: 'var(--space-lg)',
        boxShadow: '0 8px 32px rgba(0,0,0,0.5)',
      }}>
        <h2 className="font-headline-sm" style={{ color: 'var(--color-primary)', marginTop: 0, marginBottom: 16 }}>
          {isRestart ? "RESTART SIMULATION" : "START SIMULATION"}
        </h2>
        
        <p style={{ color: 'var(--color-on-surface-variant)', fontSize: 14, marginBottom: 24 }}>
          {isRestart 
            ? "Are you sure you want to restart the simulation? This will kill the current process." 
            : "Simulation is currently stopped. Would you like to launch the ROS 2 environment?"}
        </p>

        <label style={{
          display: 'flex', alignItems: 'center', gap: 12, cursor: 'pointer',
          padding: '12px', background: 'var(--color-surface-container)', 
          border: '1px solid var(--color-outline-variant)'
        }}>
          <input 
            type="checkbox" 
            checked={headless} 
            onChange={(e) => setHeadless(e.target.checked)} 
            style={{ width: 18, height: 18 }}
          />
          <div>
            <div style={{ color: 'var(--color-on-surface)', fontWeight: 500, fontSize: 14 }}>Run Headlessly</div>
            <div style={{ color: 'var(--color-on-surface-variant)', fontSize: 12, marginTop: 4 }}>
              Disables Gazebo GUI rendering to save resources.
            </div>
          </div>
        </label>

        <div style={{ display: 'flex', justifyContent: 'flex-end', gap: 12, marginTop: 32 }}>
          <button className="btn btn-secondary" onClick={onClose}>
            CANCEL
          </button>
          <button 
            className="btn" 
            style={{
              background: 'var(--color-primary)',
              color: 'var(--color-surface)',
              borderColor: 'var(--color-primary)'
            }}
            onClick={() => onStart(headless)}
          >
            {isRestart ? "RESTART SIM" : "START SIM"}
          </button>
        </div>
      </div>
    </div>
  );
};

export default SimulationModal;
