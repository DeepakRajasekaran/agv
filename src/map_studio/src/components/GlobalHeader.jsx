import React from 'react';

const GlobalHeader = ({ activeMapName, onEstop, onMapSelect, maps = [], activeView, editMode, onToggleEdit, simStatus, onOpenSimModal }) => {
  return (
    <header className="fixed top-0 left-0 w-full h-16 flex justify-between items-center px-lg z-50 bg-surface dark:bg-surface border-b-2 border-outline-variant dark:border-outline-variant">
      {/* Left Area: Branding & Context */}
      {activeView === 'mission_control' ? (
        <div className="flex items-center gap-md">
          <span className="text-headline-md font-headline-md text-primary dark:text-primary tracking-tighter cursor-pointer" onClick={() => window.dispatchEvent(new CustomEvent('navigate', { detail: 'mission_control' }))}>AGV-OS</span>
          <div className="h-8 border-l border-outline-variant mx-2"></div>
          <div className="flex items-center bg-surface-container-high border border-outline-variant rounded px-md py-xs cursor-pointer hover:border-primary transition-colors">
            <span className="font-data-mono text-data-mono text-on-surface-variant mr-2">MAP:</span>
            <span className="font-label-caps text-label-caps text-on-surface uppercase">
              {activeMapName || "NO MAP SELECTED"}
            </span>
            <span className="material-symbols-outlined ml-2 text-[16px] text-on-surface-variant" style={{ fontVariationSettings: "'FILL' 0" }}>arrow_drop_down</span>
          </div>
        </div>
      ) : (
        <div className="flex items-center gap-lg">
          <div className="text-headline-md font-headline-md text-primary dark:text-primary tracking-tighter cursor-pointer" onClick={() => window.dispatchEvent(new CustomEvent('navigate', { detail: 'mission_control' }))}>AGV-OS</div>
          <div className="h-8 w-px bg-outline-variant mx-sm"></div>
          <div className="font-headline-sm text-on-surface">MAP STUDIO</div>
        </div>
      )}

      {/* Right Area: Actions */}
      <div className="flex items-center gap-md">
        
        {/* Mission Control specific actions */}
        {activeView === 'mission_control' && (
          <button 
            className="bg-transparent border-2 border-[#94A3B8] text-[#F8FAFC] font-label-caps text-label-caps px-md py-sm rounded hover:bg-surface-container-high transition-colors flex items-center"
            onClick={() => window.dispatchEvent(new CustomEvent('navigate', { detail: 'fleet' }))}
          >
            <span className="w-2 h-2 rounded-full bg-secondary mr-2"></span>
            FLEET STATUS
          </button>
        )}

        {/* Map Studio specific actions */}
        {activeView === 'map_studio' && (
          <>
            <button 
              className="flex items-center gap-sm px-4 py-2 border-2 border-outline text-on-surface font-label-caps text-label-caps rounded hover:bg-surface-container-high transition-colors"
            >
              <span className="material-symbols-outlined" style={{ fontVariationSettings: "'FILL' 0" }}>save</span>
              SAVE REVISION
            </button>
            
            <button 
              onClick={onOpenSimModal}
              className={`flex items-center gap-sm px-4 py-2 font-label-caps text-label-caps font-bold rounded transition-colors ${
                simStatus === 'running' ? 'bg-[#EF4444] text-[#F8FAFC]' : 'bg-primary-container text-on-primary-container hover:bg-primary-fixed'
              }`}
            >
              <span className="material-symbols-outlined" style={{ fontVariationSettings: "'FILL' 1" }}>
                {simStatus === 'running' ? 'restart_alt' : 'rocket_launch'}
              </span>
              {simStatus === 'running' ? 'RESTART SIMULATION' : 'START SIMULATION'}
            </button>
          </>
        )}

        <button aria-label="notifications" className="w-10 h-10 flex items-center justify-center text-on-surface-variant hover:bg-surface-container-high rounded transition-colors ml-2">
          <span className="material-symbols-outlined">notifications</span>
        </button>
        <button aria-label="settings" className="w-10 h-10 flex items-center justify-center text-on-surface-variant hover:bg-surface-container-high rounded transition-colors">
          <span className="material-symbols-outlined">settings</span>
        </button>

        <div className="h-8 border-l border-outline-variant mx-2"></div>

        <button onClick={onEstop} className="bg-[#EF4444] text-[#F8FAFC] font-headline-sm text-headline-sm font-label-caps text-label-caps px-xl py-sm rounded font-bold pulse-estop shadow-lg hover:bg-red-600 transition-colors border-2 border-red-700 active:scale-95 flex items-center gap-xs">
          <span className="material-symbols-outlined">warning</span> E-STOP
        </button>
        
      </div>
    </header>
  );
};

export default GlobalHeader;
