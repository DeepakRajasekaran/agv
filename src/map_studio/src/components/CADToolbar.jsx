import React from 'react';
import {
  SelectionPlus, Hammer, LineSegment, Ruler,
  ArrowUUpLeft, Trash, GpsFix, Equals, ArrowUp, ArrowRight, Rows, VectorTwo, Anchor, ChartPieSlice, Angle, ArrowsLeftRight
} from '@phosphor-icons/react';

const CADToolbar = ({
  activeTool,
  setActiveTool,
  isConstructionMode,
  setIsConstructionMode,
  isBidirectional,
  setIsBidirectional,
  undo,
  handleClearSketch,
  onConstructionClick,
  children
}) => {
  return (
    <div style={{ display: 'flex', gap: 5, background: '#222', padding: '2px 8px', borderRadius: 6, alignItems: 'center' }}>
      <button onClick={() => setActiveTool('select')} title="Select"
        style={{ background: activeTool === 'select' ? '#1a3a5c' : 'transparent', color: 'white', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
        <SelectionPlus size={16} weight="bold" />
      </button>

      <div style={{ width: 1, height: 16, background: '#333', margin: '0 4px' }} />
      
      <button onClick={onConstructionClick ? onConstructionClick : () => setIsConstructionMode(!isConstructionMode)} title="Toggle Construction Mode"
        style={{ background: isConstructionMode ? '#5c4d1a' : 'transparent', color: isConstructionMode ? '#ff9800' : '#888', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
         <Hammer size={16} weight="bold" />
      </button>
      <button onClick={() => setIsBidirectional(!isBidirectional)} title="Toggle Bidirectional Path"
        style={{ background: isBidirectional ? '#5c1a5c' : 'transparent', color: isBidirectional ? '#e040fb' : '#888', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
         <ArrowsLeftRight size={16} weight="bold" />
      </button>
      <button onClick={() => setActiveTool('line')} title="Line"
        style={{ background: activeTool === 'line' ? '#1a3a5c' : 'transparent', color: 'white', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
        <LineSegment size={16} weight="bold" />
      </button>
      <button onClick={() => setActiveTool('sector')} title="Curve/Spline"
        style={{ background: activeTool === 'sector' ? '#1a3a5c' : 'transparent', color: 'white', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
        <ChartPieSlice size={16} weight="bold" />
      </button>
      <button onClick={() => setActiveTool('dimension')} title="Dimension"
        style={{ background: activeTool === 'dimension' ? '#1a3a5c' : 'transparent', color: 'white', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
        <Ruler size={16} weight="bold" />
      </button>
      
      <div style={{ width: 1, height: 16, background: '#444', margin: '0 2px' }} />

      <button onClick={undo} title="Undo (Ctrl+Z)"
        style={{ background: 'transparent', color: '#aaa', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
        <ArrowUUpLeft size={16} weight="bold" />
      </button>
      <button onClick={handleClearSketch} title="Clear All Sketch"
        style={{ background: 'transparent', color: '#ff5252', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
        <Trash size={16} weight="bold" />
      </button>
      
      <div style={{ width: 1, height: 16, background: '#444', margin: '0 4px' }} />

      <div style={{ display: 'flex', gap: 2 }}>
          {[
            ['coincide', GpsFix], ['equal', Equals], ['vertical', ArrowUp], 
            ['horizontal', ArrowRight], ['parallel', Rows], ['perpendicular', VectorTwo], ['anchor', Anchor]
          ].map(([t, Icon]) => (
            <button key={t} onClick={() => setActiveTool(t)} title={t.charAt(0).toUpperCase() + t.slice(1)}
              style={{ background: activeTool === t ? '#1a4a25' : 'transparent', color: '#00e5ff', border: 'none', padding: '4px', cursor: 'pointer', borderRadius: 4 }}>
              <Icon size={16} weight={t === 'coincide' ? "fill" : "bold"} />
            </button>
         ))}
      </div>

      <div style={{ width: 1, height: 16, background: '#333', margin: '0 4px' }} />

      {/* Render context-specific buttons (like Finalize) here */}
      {children}
    </div>
  );
};

export default CADToolbar;
