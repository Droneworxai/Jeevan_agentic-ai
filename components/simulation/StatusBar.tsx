"use client";

interface StatusBarProps {
  completedSteps: number[];
  isMissionRunning?: boolean;
}

export default function StatusBar({ completedSteps, isMissionRunning = false }: StatusBarProps) {
  return (
    <div className="space-y-2">
      {isMissionRunning && (
        <div className="flex items-center justify-center gap-3 bg-gradient-to-r from-green-500 to-blue-500 text-white py-3 px-6 rounded-lg shadow-lg animate-pulse">
          <span className="text-2xl">🚀</span>
          <span className="text-lg font-bold">MISSION EXECUTING</span>
          <span className="text-2xl">🤖</span>
        </div>
      )}
      <div className="flex flex-wrap items-center justify-center gap-3 text-[10px] text-gray-600 bg-white py-1 px-3 rounded-md border border-gray-200 shadow-sm">
        <span className={completedSteps.includes(0) ? "text-green-600 font-semibold" : ""}>
          {completedSteps.includes(0) ? "✓" : "○"} Sim
        </span>
        <span className={completedSteps.includes(1) ? "text-green-600 font-semibold" : ""}>
          {completedSteps.includes(1) ? "✓" : "○"} ROS
        </span>
        <span className={completedSteps.includes(2) ? "text-green-600 font-semibold" : ""}>
          {completedSteps.includes(2) ? "✓" : "○"} Boundary
        </span>
        <span className={completedSteps.includes(3) ? "text-green-600 font-semibold" : ""}>
          {completedSteps.includes(3) ? "✓" : "○"} Mission
        </span>
        <span className={completedSteps.includes(4) ? "text-green-600 font-semibold" : ""}>
          {completedSteps.includes(4) ? "✓" : "○"} Execute
        </span>
      </div>
    </div>
  );
}
