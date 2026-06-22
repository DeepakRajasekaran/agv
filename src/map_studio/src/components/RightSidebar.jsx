import React from 'react';

const RightSidebar = ({ diagnostics, activeView, selectedItem }) => {
  
  if (activeView === 'map_studio') {
    return (
      <aside className="bg-surface-container-low dark:bg-surface-container-low text-primary dark:text-primary font-label-caps text-label-caps font-data-mono text-data-mono border-l-2 border-outline-variant dark:border-outline-variant flat no shadows h-full w-80 flex flex-col z-40 shrink-0">
        <div className="p-panel-padding border-b-2 border-outline-variant bg-surface-container flex items-center justify-between">
          <div>
            <h2 className="font-headline-sm text-headline-sm text-on-surface m-0">PROPERTIES</h2>
            <span className="text-on-surface-variant font-label-sm uppercase">
              {selectedItem ? (selectedItem.type === 'EDGE' ? 'EDGE SELECTED' : 'VERTEX SELECTED') : 'NO SELECTION'}
            </span>
          </div>
          <span className="material-symbols-outlined text-primary text-[32px]">point_of_sale</span>
        </div>
        
        <div className="flex-1 overflow-y-auto p-panel-padding space-y-6">
          {selectedItem ? (
            <>
              {/* Identity */}
              <div className="space-y-2">
                <label className="block text-on-surface-variant font-label-caps text-label-caps">ID</label>
                <input 
                  readOnly
                  className="w-full bg-[#0F172A] border border-outline-variant text-on-surface font-data-mono p-2 rounded focus:border-primary focus:ring-1 focus:ring-primary outline-none transition-all" 
                  type="text" 
                  value={selectedItem.type === 'EDGE' ? `E-${selectedItem.id}` : `V-${selectedItem.id}`}
                />
              </div>

              {selectedItem.type !== 'EDGE' && (
                <div className="space-y-2">
                  <label className="block text-on-surface-variant font-label-caps text-label-caps">TYPE</label>
                  <div className="flex border-2 border-outline-variant rounded overflow-hidden">
                    <button className={`flex-1 p-2 font-label-caps ${selectedItem.type !== 'charging' && selectedItem.type !== 'station' ? 'bg-primary text-[#0F172A] border-b-2 border-primary' : 'bg-surface-container-highest text-on-surface-variant hover:text-on-surface'}`}>
                      STANDARD
                    </button>
                    <button className={`flex-1 p-2 font-label-caps ${selectedItem.type === 'charging' ? 'bg-primary text-[#0F172A] border-b-2 border-primary' : 'bg-surface-container-highest text-on-surface-variant border-l border-outline-variant hover:text-on-surface'}`}>
                      CHARGING
                    </button>
                    <button className={`flex-1 p-2 font-label-caps ${selectedItem.type === 'station' ? 'bg-primary text-[#0F172A] border-b-2 border-primary' : 'bg-surface-container-highest text-on-surface-variant border-l border-outline-variant hover:text-on-surface'}`}>
                      STATION
                    </button>
                  </div>
                </div>
              )}

              {/* Coordinates / Dimensions */}
              {selectedItem.type === 'EDGE' ? (
                <div>
                  <h3 className="text-on-surface-variant font-label-caps text-label-caps mb-3 border-b border-outline-variant pb-1">EDGE PROPERTIES</h3>
                  <div className="grid grid-cols-2 gap-4">
                    <div className="space-y-1">
                      <label className="text-xs text-on-surface-variant">START VERTEX</label>
                      <input readOnly className="w-full bg-[#0F172A] border border-outline-variant text-primary font-data-mono p-2 rounded" type="text" value={`V-${selectedItem.start_vertex_id}`}/>
                    </div>
                    <div className="space-y-1">
                      <label className="text-xs text-on-surface-variant">END VERTEX</label>
                      <input readOnly className="w-full bg-[#0F172A] border border-outline-variant text-primary font-data-mono p-2 rounded" type="text" value={`V-${selectedItem.end_vertex_id}`}/>
                    </div>
                    <div className="space-y-1 col-span-2 flex items-center justify-between bg-[#0F172A] border border-outline-variant p-2 rounded">
                      <label className="text-xs text-on-surface-variant">BIDIRECTIONAL</label>
                      <span className={`font-data-mono ${selectedItem.is_bidirectional ? 'text-secondary' : 'text-error'}`}>
                        {selectedItem.is_bidirectional ? 'TRUE' : 'FALSE'}
                      </span>
                    </div>
                  </div>
                </div>
              ) : (
                <div>
                  <h3 className="text-on-surface-variant font-label-caps text-label-caps mb-3 border-b border-outline-variant pb-1">COORDINATES (m)</h3>
                  <div className="grid grid-cols-2 gap-4 mb-6">
                    <div className="space-y-1">
                      <label className="text-xs text-on-surface-variant">X</label>
                      <input readOnly className="w-full bg-[#0F172A] border border-outline-variant text-primary font-data-mono p-2 rounded" step="0.01" type="number" value={selectedItem.x?.toFixed(2) || 0}/>
                    </div>
                    <div className="space-y-1">
                      <label className="text-xs text-on-surface-variant">Y</label>
                      <input readOnly className="w-full bg-[#0F172A] border border-outline-variant text-primary font-data-mono p-2 rounded" step="0.01" type="number" value={selectedItem.y?.toFixed(2) || 0}/>
                    </div>
                    <div className="space-y-1 col-span-2">
                      <label className="text-xs text-on-surface-variant">HEADING (deg)</label>
                      <input readOnly className="w-full bg-[#0F172A] border border-outline-variant text-on-surface font-data-mono p-2 rounded" step="0.1" type="number" value="90.0"/>
                    </div>
                  </div>

                  <div>
                    <h3 className="text-on-surface-variant font-label-caps text-label-caps mb-3 border-b border-outline-variant pb-1">CONNECTED EDGES</h3>
                    <ul className="space-y-2">
                      <li className="flex justify-between items-center bg-surface p-2 rounded border border-outline-variant">
                        <span className="font-data-mono text-on-surface">E-12 (Incoming)</span>
                        <span className="material-symbols-outlined text-secondary text-[16px]">arrow_forward</span>
                      </li>
                      <li className="flex justify-between items-center bg-surface p-2 rounded border border-outline-variant border-l-2 border-l-primary">
                        <span className="font-data-mono text-primary">E-14 (Outgoing)</span>
                        <span className="material-symbols-outlined text-primary text-[16px]">trending_flat</span>
                      </li>
                    </ul>
                  </div>
                </div>
              )}
            </>
          ) : (
             <div className="text-on-surface-variant text-center mt-10">Select an element on the canvas to view its properties.</div>
          )}
        </div>

        {selectedItem && (
          <div className="p-panel-padding border-t-2 border-outline-variant bg-surface-container-lowest flex gap-2">
            <button className="flex-1 py-2 border-2 border-error text-error font-label-caps text-label-caps rounded hover:bg-error-container hover:text-on-error-container transition-colors flex items-center justify-center gap-1">
              <span className="material-symbols-outlined text-[18px]">delete</span>
              DELETE
            </button>
          </div>
        )}
      </aside>
    );
  }

  // Mission Control / Diagnostics Sidebar
  const crossTrackError = diagnostics?.cross_track_error || 0;
  // Convert generic state to the specific format expected by UI
  let displayState = diagnostics?.state || "UNKNOWN";
  if (displayState === "IDLE") displayState = "SYSTEM_READY";

  return (
    <aside className="fixed top-16 right-0 h-[calc(100vh-64px)] w-80 bg-surface-container-low dark:bg-surface-container-low border-l-2 border-outline-variant dark:border-outline-variant flex flex-col py-panel-padding z-40 hidden md:flex">
      <div className="px-md mb-lg">
        <p className="font-label-caps text-label-caps text-on-surface-variant opacity-70 mb-1">REAL-TIME</p>
        <p className="font-headline-sm text-headline-sm text-primary dark:text-primary">DIAGNOSTICS</p>
      </div>

      <div className="flex items-center gap-md px-md mb-md border-b-2 border-outline-variant pb-2">
        <div className="flex items-center text-primary dark:text-primary border-b-2 border-primary pb-2 shadow-[0_0_8px_0_rgba(242,204,15,0.5)]">
          <span className="material-symbols-outlined mr-sm text-[18px]">account_tree</span>
          <span className="font-label-caps text-label-caps font-data-mono text-data-mono">STATE MACHINE</span>
        </div>
        <div className="flex items-center text-on-surface-variant dark:text-on-surface-variant hover:text-primary dark:hover:text-primary cursor-pointer pb-2">
          <span className="material-symbols-outlined mr-sm text-[18px]">speed</span>
          <span className="font-label-caps text-label-caps font-data-mono text-data-mono opacity-50">TELEMETRY</span>
        </div>
      </div>

      <div className="flex flex-col flex-1 px-md gap-lg overflow-y-auto">
        <div className="bg-[#1E293B] border border-[#334155] rounded p-md">
          <h4 className="font-label-caps text-label-caps text-on-surface-variant mb-2">CURRENT STATE</h4>
          <div className="border-2 border-primary bg-[#735c00] bg-opacity-10 p-md rounded flex items-center justify-center shadow-[0_0_12px_0_rgba(250,204,21,0.2)]">
            <span className="font-headline-md text-headline-md text-primary font-data-mono tracking-widest">{displayState}</span>
          </div>
        </div>

        <div className="bg-[#1E293B] border border-[#334155] rounded p-md">
          <h4 className="font-label-caps text-label-caps text-on-surface-variant mb-md flex items-center border-t-2 border-secondary pt-1">
            SENSOR FEED
          </h4>
          <div className="flex flex-col items-center">
            <span className="font-data-mono text-data-mono text-on-surface mb-2">CROSS-TRACK ERROR</span>
            <div className="w-full relative h-10 flex items-center justify-center mt-2">
              <div className="absolute w-full h-[2px] bg-outline-variant top-1/2 -translate-y-1/2"></div>
              <div className="absolute w-[2px] h-4 bg-outline-variant top-1/2 -translate-y-1/2 left-0"></div>
              <div className="absolute w-[2px] h-6 bg-secondary top-1/2 -translate-y-1/2 left-1/2"></div>
              <div className="absolute w-[2px] h-4 bg-outline-variant top-1/2 -translate-y-1/2 right-0"></div>
              <div className="absolute h-[4px] bg-[#EF4444] top-1/2 -translate-y-1/2 left-0 w-[15%]"></div>
              <div className="absolute h-[4px] bg-[#EF4444] top-1/2 -translate-y-1/2 right-0 w-[15%]"></div>
              
              {/* Needle dynamically positioned */}
              <div 
                className="absolute w-0 h-0 border-l-[6px] border-l-transparent border-r-[6px] border-r-transparent border-t-[10px] border-t-primary top-0 -translate-x-1/2 transition-all duration-300"
                style={{ left: `${Math.max(0, Math.min(100, 50 + (crossTrackError * 500)))}%` }}
              ></div>
            </div>
            
            <div className="flex justify-between w-full mt-2 font-data-mono text-[10px] text-on-surface-variant">
              <span>-100mm</span>
              <span className="text-secondary font-bold">0mm</span>
              <span>+100mm</span>
            </div>
            <div className="mt-2 text-primary font-data-mono text-body-lg font-bold">
              {crossTrackError > 0 ? '+' : ''}{(crossTrackError * 1000).toFixed(1)} mm
            </div>
          </div>
        </div>

        <div className="bg-[#1E293B] border border-[#334155] rounded p-md flex-1 flex flex-col min-h-[200px]">
          <h4 className="font-label-caps text-label-caps text-on-surface-variant mb-2">FAULT LOG</h4>
          <div className="bg-surface-dim border border-outline-variant p-sm rounded flex-1 overflow-y-auto font-data-mono text-[11px] text-on-surface-variant flex flex-col gap-1">
             {diagnostics?.faults && diagnostics.faults.length > 0 ? (
               diagnostics.faults.map((f, i) => (
                 <div key={i}><span className="text-[#EF4444]">[FAULT]</span> {f}</div>
               ))
             ) : (
               <div><span className="text-secondary">[SYS]</span> NO FAULTS</div>
             )}
             <div><span className="text-secondary">[OK]</span> SYSTEM READY</div>
             <div className="opacity-50">...</div>
          </div>
        </div>
      </div>
    </aside>
  );
};

export default RightSidebar;
