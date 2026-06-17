import React from 'react';
import { Trash } from '@phosphor-icons/react';

const ConstraintList = ({ constraints, setConstraints, dimensions, setDimensions, fixedPoints = [], setFixedPoints }) => {
  if (!constraints.length && !dimensions.length && !fixedPoints.length) return null;

  return (
    <div style={{
      position: 'absolute',
      right: 20,
      top: 60,
      width: 250,
      background: 'rgba(20, 20, 20, 0.85)',
      border: '1px solid #333',
      borderRadius: 6,
      padding: '10px',
      color: '#ddd',
      fontSize: '0.75rem',
      zIndex: 100,
      maxHeight: 400,
      overflowY: 'auto'
    }}>
      <div style={{ fontWeight: 'bold', marginBottom: 8, color: '#00e5ff', borderBottom: '1px solid #333', paddingBottom: 4 }}>
        Constraint History
      </div>
      
      {dimensions.map(d => (
        <div key={`d-${d.id}`} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 6, background: '#222', padding: '4px 6px', borderRadius: 4 }}>
          <div style={{ display: 'flex', flexDirection: 'column' }}>
            <span style={{ color: '#00e5ff', fontSize: '0.65rem', fontWeight: 'bold' }}>DIMENSION</span>
            <span>{d.label} ({String(d.v1.sketchId || '').split('-')[0]}:{d.v1.part} ↔ {String(d.v2.sketchId || '').split('-')[0]}:{d.v2.part})</span>
          </div>
          <div style={{ display: 'flex', gap: 4 }}>
            <button 
              onClick={() => {
                const val = prompt("Enter new dimension value (m):", d.value);
                if (val !== null && !isNaN(parseFloat(val))) {
                   setDimensions(prev => prev.map(x => x.id === d.id ? { ...x, value: parseFloat(val), label: `${val}m` } : x));
                }
              }}
              style={{ background: 'transparent', border: 'none', color: '#aaa', cursor: 'pointer', padding: 2 }}>
              ✏️
            </button>
            <button onClick={() => setDimensions(prev => prev.filter(x => x.id !== d.id))} style={{ background: 'transparent', border: 'none', color: '#ff5252', cursor: 'pointer', padding: 2 }}>
              <Trash size={12} weight="bold" />
            </button>
          </div>
        </div>
      ))}

      {/* Fixed Points (Anchors) */}
      {fixedPoints.map((f, idx) => (
        <div key={`f-${idx}`} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 6, background: '#2d1a00', border: '1px solid #ff9800', padding: '4px 6px', borderRadius: 4 }}>
          <div style={{ display: 'flex', flexDirection: 'column' }}>
            <span style={{ color: '#ff9800', fontSize: '0.65rem', fontWeight: 'bold' }}>ANCHOR</span>
            <span style={{ fontSize: '0.7rem' }}>
              {String(f.sketchId || '').split('-')[0]}:{f.part}
            </span>
          </div>
          <button 
            onClick={() => {
              if (setFixedPoints) {
                setFixedPoints(fixedPoints.filter((_, i) => i !== idx));
              }
            }} 
            style={{ background: 'transparent', border: 'none', color: '#ff5252', cursor: 'pointer', padding: 4 }}>
            <Trash size={12} weight="bold" />
          </button>
        </div>
      ))}
      
      {constraints.map(c => (
        <div key={`c-${c.id}`} style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 6, background: '#222', padding: '4px 6px', borderRadius: 4 }}>
          <div style={{ display: 'flex', flexDirection: 'column' }}>
            <span style={{ color: '#4CAF50', fontSize: '0.65rem', fontWeight: 'bold' }}>{c.type.toUpperCase()}</span>
            <span style={{ fontSize: '0.7rem' }}>
              {String(c.v1?.sketchId || '').split('-')[0]}:{c.v1?.part} 
              {c.v2 ? ` ↔ ${String(c.v2.sketchId || '').split('-')[0]}:${c.v2.part}` : ''}
            </span>
          </div>
          <button onClick={() => setConstraints(prev => prev.filter(x => x.id !== c.id))} style={{ background: 'transparent', border: 'none', color: '#ff5252', cursor: 'pointer', padding: 4 }}>
            <Trash size={12} weight="bold" />
          </button>
        </div>
      ))}
      
    </div>
  );
};

export default ConstraintList;
