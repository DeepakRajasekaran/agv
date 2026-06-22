import React from 'react';
import {
  SelectionPlus, Hammer, LineSegment, Ruler,
  ArrowUUpLeft, Trash, GpsFix, Equals, ArrowUp, ArrowRight, Rows, VectorTwo, Anchor, ChartPieSlice, ArrowsLeftRight
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
    <div className="bg-surface-container border-2 border-outline-variant rounded-lg p-1 flex items-center gap-1 shadow-lg">
      <button 
        onClick={() => setActiveTool('select')} 
        title="Select"
        className={`p-2 rounded transition-colors flex items-center justify-center ${activeTool === 'select' ? 'bg-surface-container-highest text-primary' : 'text-on-surface-variant hover:bg-surface-container-high hover:text-primary'}`}
      >
        <SelectionPlus size={20} weight="bold" />
      </button>

      <div className="w-px h-6 bg-outline-variant mx-1"></div>
      
      <button 
        onClick={onConstructionClick ? onConstructionClick : () => setIsConstructionMode(!isConstructionMode)} 
        title="Toggle Construction Mode"
        className={`p-2 rounded transition-colors flex items-center justify-center ${isConstructionMode ? 'bg-[#5c4d1a] text-primary' : 'text-on-surface-variant hover:bg-surface-container-high hover:text-primary'}`}
      >
         <Hammer size={20} weight="bold" />
      </button>

      <button 
        onClick={() => setIsBidirectional(!isBidirectional)} 
        title="Toggle Bidirectional Path"
        className={`p-2 rounded transition-colors flex items-center justify-center ${isBidirectional ? 'bg-secondary-container text-on-secondary-container' : 'text-on-surface-variant hover:bg-surface-container-high hover:text-secondary'}`}
      >
         <ArrowsLeftRight size={20} weight="bold" />
      </button>

      <button 
        onClick={() => setActiveTool('line')} 
        title="Draw Path / Line"
        className={`p-2 rounded transition-colors flex items-center justify-center ${activeTool === 'line' ? 'bg-surface-container-highest text-primary' : 'text-on-surface-variant hover:bg-surface-container-high hover:text-primary'}`}
      >
        <LineSegment size={20} weight="bold" />
      </button>

      <button 
        onClick={() => setActiveTool('sector')} 
        title="Curve/Spline"
        className={`p-2 rounded transition-colors flex items-center justify-center ${activeTool === 'sector' ? 'bg-surface-container-highest text-primary' : 'text-on-surface-variant hover:bg-surface-container-high hover:text-primary'}`}
      >
        <ChartPieSlice size={20} weight="bold" />
      </button>

      <button 
        onClick={() => setActiveTool('dimension')} 
        title="Dimension"
        className={`p-2 rounded transition-colors flex items-center justify-center ${activeTool === 'dimension' ? 'bg-surface-container-highest text-primary' : 'text-on-surface-variant hover:bg-surface-container-high hover:text-primary'}`}
      >
        <Ruler size={20} weight="bold" />
      </button>
      
      <div className="w-px h-6 bg-outline-variant mx-1"></div>

      <button 
        onClick={undo} 
        title="Undo (Ctrl+Z)"
        className="p-2 rounded text-on-surface-variant hover:bg-surface-container-high hover:text-primary transition-colors flex items-center justify-center"
      >
        <ArrowUUpLeft size={20} weight="bold" />
      </button>
      <button 
        onClick={handleClearSketch} 
        title="Clear All Sketch"
        className="p-2 rounded text-error hover:bg-error-container hover:text-on-error-container transition-colors flex items-center justify-center"
      >
        <Trash size={20} weight="bold" />
      </button>
      
      <div className="w-px h-6 bg-outline-variant mx-1"></div>

      <div className="flex gap-1">
          {[
            ['coincide', GpsFix], ['equal', Equals], ['vertical', ArrowUp], 
            ['horizontal', ArrowRight], ['parallel', Rows], ['perpendicular', VectorTwo], ['anchor', Anchor]
          ].map(([t, Icon]) => (
            <button 
              key={t} 
              onClick={() => setActiveTool(t)} 
              title={t.charAt(0).toUpperCase() + t.slice(1)}
              className={`p-2 rounded transition-colors flex items-center justify-center ${activeTool === t ? 'bg-[#1a4a25] text-secondary' : 'text-secondary opacity-70 hover:bg-surface-container-high hover:opacity-100'}`}
            >
              <Icon size={18} weight={t === 'coincide' ? "fill" : "bold"} />
            </button>
         ))}
      </div>

      <div className="w-px h-6 bg-outline-variant mx-1"></div>

      {/* Render context-specific buttons (like Finalize) here */}
      {children && (
        <div className="ml-1 flex items-center">
          {children}
        </div>
      )}
    </div>
  );
};

export default CADToolbar;
