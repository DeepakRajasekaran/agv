import React from 'react';

const NAV_ITEMS = [
  { key: 'mission_control', label: 'MISSION CONTROL', icon: 'precision_manufacturing' },
  { key: 'map_studio',      label: 'MAP STUDIO',      icon: 'map' },
  { key: 'diagnostics',     label: 'DIAGNOSTICS',     icon: 'analytics' },
  { key: 'fleet',           label: 'FLEET',           icon: 'group_work' }
];

const LeftSidebar = ({ activeView, activeMapName, mapVersion = "v2.4.1" }) => {
  const handleNav = (key) => {
    window.dispatchEvent(new CustomEvent('navigate', { detail: key }));
  };

  if (activeView === 'map_studio') {
    return (
      <nav className="bg-surface-container-low dark:bg-surface-container-low text-secondary dark:text-secondary font-label-caps text-label-caps font-data-mono text-data-mono border-r-2 border-outline-variant dark:border-outline-variant flat no shadows h-full w-64 flex flex-col z-40 shrink-0">
        <div className="p-panel-padding border-b-2 border-outline-variant">
          <h2 className="font-label-caps text-label-caps text-on-surface-variant mb-4 tracking-widest">MAP METADATA</h2>
          <div className="space-y-4">
            <div>
              <div className="text-xs text-on-surface-variant mb-1 font-label-sm">NAME</div>
              <div className="font-data-mono text-on-surface">{activeMapName || "Untitled Map"}</div>
            </div>
            <div>
              <div className="text-xs text-on-surface-variant mb-1 font-label-sm">VERSION</div>
              <div className="font-data-mono text-primary">{mapVersion} <span className="text-[10px] bg-primary/20 px-1 rounded ml-1">STABLE</span></div>
            </div>
            <div>
              <div className="text-xs text-on-surface-variant mb-1 font-label-sm">GRID RESOLUTION</div>
              <div className="font-data-mono text-on-surface">0.05m / px</div>
            </div>
          </div>
        </div>
        
        <div className="p-panel-padding flex-1 overflow-y-auto">
          <h2 className="font-label-caps text-label-caps text-on-surface-variant mb-4 tracking-widest flex justify-between items-center">
            LAYERS
            <button className="hover:text-primary"><span className="material-symbols-outlined text-[16px]">add</span></button>
          </h2>
          
          <div className="space-y-2">
            {/* Layer Item */}
            <div className="flex items-center justify-between p-2 hover:bg-surface-container-highest rounded border-l-2 border-transparent">
              <div className="flex items-center gap-3">
                <span className="material-symbols-outlined text-on-surface-variant">grid_on</span>
                <span className="font-data-mono text-on-surface">Grid</span>
              </div>
              <button className="text-on-surface-variant hover:text-primary"><span className="material-symbols-outlined text-[18px]">visibility</span></button>
            </div>
            {/* Layer Item (Active) */}
            <div className="flex items-center justify-between p-2 bg-secondary-container text-on-secondary-container border-l-4 border-primary rounded">
              <div className="flex items-center gap-3">
                <span className="material-symbols-outlined">route</span>
                <span className="font-data-mono font-bold">Paths</span>
              </div>
              <button className="hover:text-primary"><span className="material-symbols-outlined text-[18px]">visibility</span></button>
            </div>
            {/* Layer Item */}
            <div className="flex items-center justify-between p-2 hover:bg-surface-container-highest rounded border-l-2 border-transparent">
              <div className="flex items-center gap-3">
                <span className="material-symbols-outlined text-secondary">sell</span>
                <span className="font-data-mono text-on-surface">Tags</span>
              </div>
              <button className="text-on-surface-variant hover:text-primary"><span className="material-symbols-outlined text-[18px]">visibility</span></button>
            </div>
            {/* Layer Item */}
            <div className="flex items-center justify-between p-2 hover:bg-surface-container-highest rounded border-l-2 border-transparent">
              <div className="flex items-center gap-3">
                <span className="material-symbols-outlined text-error">dangerous</span>
                <span className="font-data-mono text-on-surface">Zones (Exclusion)</span>
              </div>
              <button className="text-on-surface-variant hover:text-primary"><span className="material-symbols-outlined text-[18px]">visibility_off</span></button>
            </div>
          </div>
        </div>

        <div className="p-panel-padding border-t-2 border-outline-variant bg-surface-container-lowest">
          <button className="w-full flex items-center justify-center gap-sm px-4 py-2 border-2 border-outline text-on-surface font-label-caps text-label-caps rounded hover:bg-surface-container-high transition-colors">
            <span className="material-symbols-outlined">file_download</span>
            EXPORT YAML
          </button>
        </div>
      </nav>
    );
  }

  // Mission Control Sidebar
  return (
    <nav className="fixed top-16 left-0 h-[calc(100vh-64px)] w-64 bg-surface-container-low dark:bg-surface-container-low border-r-2 border-outline-variant dark:border-outline-variant flex flex-col py-panel-padding z-40 overflow-y-auto hidden md:flex">
      <div className="px-md mb-lg">
        <p className="font-label-caps text-label-caps text-on-surface-variant opacity-70 mb-1">CONTROL TERMINAL</p>
        <p className="font-headline-sm text-headline-sm text-primary dark:text-primary">SECTOR-07</p>
      </div>

      <button className="mx-md mb-lg bg-primary-container text-[#0F172A] font-label-caps text-label-caps font-bold py-md rounded border border-primary-container hover:bg-yellow-300 transition-colors shadow-[0_0_8px_0_rgba(250,204,21,0.3)]">
        NEW MISSION
      </button>

      <div className="flex flex-col gap-sm px-md mb-xl">
        {NAV_ITEMS.map((item) => {
          const isActive = activeView === item.key;
          if (isActive) {
            return (
              <div key={item.key} className="flex items-center px-md py-md bg-secondary-container dark:bg-secondary-container text-on-secondary-container dark:text-on-secondary-container border-l-4 border-primary cursor-pointer active:scale-95 transition-transform rounded-r">
                <span className="material-symbols-outlined mr-md">{item.icon}</span>
                <span className="font-label-caps text-label-caps font-data-mono text-data-mono">{item.label}</span>
              </div>
            );
          }
          return (
            <div key={item.key} onClick={() => handleNav(item.key)} className="flex items-center px-md py-sm text-on-surface-variant dark:text-on-surface-variant opacity-70 hover:bg-surface-container-highest dark:hover:bg-surface-container-highest transition-colors cursor-pointer rounded">
              <span className="material-symbols-outlined mr-md text-[20px]">{item.icon}</span>
              <span className="font-label-caps text-label-caps">{item.label}</span>
            </div>
          );
        })}
      </div>

      {activeView === 'mission_control' && (
        <div className="px-md flex-1">
          <div className="bg-[#1E293B] border border-[#334155] rounded h-full flex flex-col p-md">
            <h3 className="font-label-caps text-label-caps text-on-surface bg-[#334155] p-sm rounded-sm mb-md inline-block w-full border-t-2 border-[#94A3B8]">MISSION BUILDER</h3>
            
            <div className="flex flex-col gap-sm mb-lg">
              <div className="bg-surface-dim border border-outline-variant p-sm rounded font-data-mono text-data-mono text-on-surface cursor-grab hover:border-primary transition-colors flex items-center">
                <span className="material-symbols-outlined text-[16px] mr-2 text-on-surface-variant">drag_indicator</span>
                [MOVE TO TAG]
              </div>
              <div className="bg-surface-dim border border-outline-variant p-sm rounded font-data-mono text-data-mono text-on-surface cursor-grab hover:border-primary transition-colors flex items-center">
                <span className="material-symbols-outlined text-[16px] mr-2 text-on-surface-variant">drag_indicator</span>
                [DOCK]
              </div>
              <div className="bg-surface-dim border border-outline-variant p-sm rounded font-data-mono text-data-mono text-on-surface cursor-grab hover:border-primary transition-colors flex items-center">
                <span className="material-symbols-outlined text-[16px] mr-2 text-on-surface-variant">drag_indicator</span>
                [WAIT 30S]
              </div>
            </div>

            <h3 className="font-label-caps text-label-caps text-on-surface mb-sm mt-auto">ACTIVE QUEUE</h3>
            <div className="bg-surface-container-lowest border border-outline-variant rounded p-sm flex flex-col gap-1 overflow-y-auto max-h-32">
              <div className="font-data-mono text-[12px] text-primary bg-[#735c00] bg-opacity-20 border border-primary p-1 rounded">
                &gt; Action 1: MAPS_TO (TAG_B)
              </div>
              <div className="font-data-mono text-[12px] text-on-surface-variant p-1">
                &gt; Action 2: DOCK (STATION_3)
              </div>
              <div className="font-data-mono text-[12px] text-on-surface-variant p-1">
                &gt; Action 3: IDLE
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Footer Tabs */}
      <div className="mt-auto px-md pt-lg flex flex-col gap-sm border-t border-outline-variant">
        <div className="flex items-center px-sm py-sm text-on-surface-variant dark:text-on-surface-variant opacity-70 hover:bg-surface-container-highest dark:hover:bg-surface-container-highest transition-colors cursor-pointer rounded">
          <span className="material-symbols-outlined mr-md text-[18px]">settings_input_component</span>
          <span className="font-label-caps text-label-caps text-[10px]">SYSTEMS</span>
        </div>
        <div className="flex items-center px-sm py-sm text-error opacity-90 hover:bg-surface-container-highest transition-colors cursor-pointer rounded">
          <span className="material-symbols-outlined mr-md text-[18px]">warning</span>
          <span className="font-label-caps text-label-caps text-[10px]">FAULTS (0)</span>
        </div>
      </div>
    </nav>
  );
};

export default LeftSidebar;
