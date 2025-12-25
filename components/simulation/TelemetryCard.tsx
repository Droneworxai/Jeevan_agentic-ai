"use client";

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Activity } from "lucide-react";

interface TelemetryCardProps {
  missionStatus: any;
  elapsedTime: number;
  robotPosition: [number, number] | null;
  rosStatus: string;
}

export default function TelemetryCard({
  missionStatus,
  elapsedTime,
  robotPosition,
  rosStatus,
}: TelemetryCardProps) {
  return (
    <Card className="shadow-lg border-none">
      <CardHeader className="border-b bg-gray-50/50">
        <CardTitle className="text-xl flex items-center gap-3 text-gray-800">
          <Activity className="w-6 h-6 text-green-600" /> Live Telemetry
        </CardTitle>
      </CardHeader>
      <CardContent className="p-6 space-y-4">
        <div className="flex justify-between items-center p-3 bg-gray-50 rounded-lg">
          <span className="text-gray-500 font-medium">Mission State</span>
          <span className="font-bold text-blue-600 uppercase">{missionStatus?.state || "IDLE"}</span>
        </div>
        <div className="flex justify-between items-center p-3 bg-gray-50 rounded-lg">
          <span className="text-gray-500 font-medium">Weeds Neutralized</span>
          <span className="font-bold text-green-600 text-xl">{missionStatus?.weeds_removed || 0}</span>
        </div>
        <div className="grid grid-cols-2 gap-4">
          <div className="p-3 bg-gray-50 rounded-lg">
            <div className="text-[10px] font-bold text-gray-400 uppercase">Time Elapsed</div>
            <div className="text-lg font-bold text-gray-700">
              {Math.floor(elapsedTime / 60)}:{(elapsedTime % 60).toString().padStart(2, '0')}
            </div>
          </div>
          <div className="p-3 bg-gray-50 rounded-lg">
            <div className="text-[10px] font-bold text-gray-400 uppercase">Area Covered</div>
            <div className="text-lg font-bold text-gray-700">
              {(missionStatus?.weeds_removed || 0) * 2.5} m²
            </div>
          </div>
        </div>
        <div className="space-y-2">
          <span className="text-xs font-bold text-gray-400 uppercase tracking-widest">Robot Pose (GPS)</span>
          <div className="grid grid-cols-2 gap-2">
            <div className="p-2 bg-gray-900 text-white rounded text-center">
              <div className="text-[10px] text-gray-400">LATITUDE</div>
              <div className="font-mono">{robotPosition ? robotPosition[0].toFixed(6) : "---"}</div>
            </div>
            <div className="p-2 bg-gray-900 text-white rounded text-center">
              <div className="text-[10px] text-gray-400">LONGITUDE</div>
              <div className="font-mono">{robotPosition ? robotPosition[1].toFixed(6) : "---"}</div>
            </div>
          </div>
        </div>

        <div className="pt-4 border-t">
          <span className="text-xs font-bold text-gray-400 uppercase tracking-widest">System Health</span>
          <div className="mt-2 space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-xs text-gray-600">ROS Bridge</span>
              <span className={rosStatus === "connected" ? "text-green-600 font-bold" : "text-gray-400 font-bold"}>
                {rosStatus === "connected" ? "ONLINE" : "OFFLINE"}
              </span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-xs text-gray-600">Mission Planner</span>
              <span className={missionStatus ? "text-green-600 font-bold" : "text-gray-400 font-bold"}>
                {missionStatus ? "ACTIVE" : "WAITING"}
              </span>
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
