"use client";

import { Card, CardContent } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Play, Square } from "lucide-react";

interface MissionControlButtonsProps {
  isMissionRunning: boolean;
  handleStartMission: () => void;
  handleStopMission: () => void;
  isLoading: boolean;
  rosStatus: string;
  isMissionLoaded: boolean;
}

export default function MissionControlButtons({
  isMissionRunning,
  handleStartMission,
  handleStopMission,
  isLoading,
  rosStatus,
  isMissionLoaded,
}: MissionControlButtonsProps) {
  return (
    <Card className="shadow-lg border-none bg-white/80 backdrop-blur">
      <CardContent className="p-6">
        {!isMissionRunning ? (
          <Button 
            className="w-full h-14 bg-gradient-to-r from-green-600 to-blue-600 hover:from-green-700 hover:to-blue-700 text-lg font-bold shadow-blue-200 shadow-lg gap-3 disabled:opacity-50 disabled:cursor-not-allowed" 
            onClick={handleStartMission}
            disabled={isLoading || rosStatus !== "connected" || !isMissionLoaded}
            title={!isMissionLoaded ? "Load mission first!" : "Start mission execution"}
          >
            <span className="text-2xl">🚀</span>
            <div className="flex flex-col items-center">
              <span>EXECUTE MISSION</span>
              <span className="text-xs font-normal opacity-80">
                {!isMissionLoaded ? "Load Mission First" : "Ctrl+Enter"}
              </span>
            </div>
          </Button>
        ) : (
          <Button 
            className="w-full h-14 bg-gradient-to-r from-red-600 to-orange-600 hover:from-red-700 hover:to-orange-700 text-lg font-bold shadow-red-200 shadow-lg gap-3 animate-pulse" 
            onClick={handleStopMission}
            disabled={isLoading || rosStatus !== "connected"}
          >
            <span className="text-2xl">⛔</span>
            <div className="flex flex-col items-center">
              <span>ABORT MISSION</span>
              <span className="text-xs font-normal opacity-80">Ctrl+Esc</span>
            </div>
          </Button>
        )}
      </CardContent>
    </Card>
  );
}
