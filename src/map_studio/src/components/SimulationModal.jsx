import React, { useState } from 'react';

const SimulationModal = ({ isOpen, onClose, onStart, isRestart = false }) => {
  const [headless, setHeadless] = useState(false);

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black/75 z-50 flex items-center justify-center p-4">
      <div className="bg-[#1E293B] border border-[#334155] w-[400px] p-xl rounded shadow-2xl">
        <h2 className="font-headline-sm text-headline-sm text-primary mb-md">
          {isRestart ? "RESTART SIMULATION" : "START SIMULATION"}
        </h2>
        
        <p className="text-on-surface-variant text-body-md mb-lg">
          {isRestart 
            ? "Are you sure you want to restart the simulation? This will kill the current process." 
            : "Simulation is currently stopped. Would you like to launch the ROS 2 environment?"}
        </p>

        <label className="flex items-center gap-3 cursor-pointer p-3 bg-surface-dim border border-outline-variant rounded hover:border-primary transition-colors">
          <input 
            type="checkbox" 
            checked={headless} 
            onChange={(e) => setHeadless(e.target.checked)} 
            className="w-5 h-5 accent-primary bg-surface-container border-outline-variant rounded cursor-pointer"
          />
          <div>
            <div className="text-on-surface font-bold text-[14px]">Run Headlessly</div>
            <div className="text-on-surface-variant text-[12px] mt-1">
              Disables Gazebo GUI rendering to save resources.
            </div>
          </div>
        </label>

        <div className="flex justify-end gap-3 mt-xl pt-md border-t border-outline-variant">
          <button 
            className="bg-transparent border border-outline-variant text-on-surface font-label-caps text-label-caps py-2 px-4 rounded hover:bg-surface-container-high transition-colors" 
            onClick={onClose}
          >
            CANCEL
          </button>
          <button 
            className="bg-primary text-[#0F172A] border border-primary font-label-caps text-label-caps py-2 px-4 rounded font-bold hover:bg-primary-fixed transition-colors shadow-[0_0_8px_0_rgba(250,204,21,0.3)]"
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
