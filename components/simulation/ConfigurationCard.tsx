"use client";

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Label } from "@/components/ui/label";
import { Settings, Map as MapIcon, Bot } from "lucide-react";

interface ConfigurationCardProps {
  farmName: string;
  setFarmName: (name: string) => void;
  planner: string;
  setPlanner: (planner: string) => void;
  handleSetBoundary: () => void;
  handleLoadMission: () => void;
  isLoading: boolean;
  rosStatus: string;
  isBoundarySet: boolean;
  isMissionLoaded: boolean;
  pathLinesCount?: number;
}

export default function ConfigurationCard({
  farmName,
  setFarmName,
  planner,
  setPlanner,
  handleSetBoundary,
  handleLoadMission,
  isLoading,
  rosStatus,
  isBoundarySet,
  isMissionLoaded,
  pathLinesCount = 0,
}: ConfigurationCardProps) {
  return (
    <Card className="shadow-lg border-none bg-white/80 backdrop-blur">
      <CardHeader className="border-b bg-gray-50/50">
        <CardTitle className="text-xl flex items-center gap-3 text-gray-800">
          <Settings className="w-6 h-6 text-blue-600" /> Configuration
        </CardTitle>
      </CardHeader>
      <CardContent className="p-6 space-y-6">
        <div className="space-y-2">
          <Label className="text-sm font-semibold text-gray-600">Active Farm</Label>
          <Select value={farmName} onValueChange={setFarmName}>
            <SelectTrigger className="h-12">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="Sandfields Farm Ltd">Sandfields Farm Ltd (UK)</SelectItem>
              <SelectItem value="Manor Farm">Manor Farm (Luddington)</SelectItem>
            </SelectContent>
          </Select>
        </div>

        <div className="space-y-2">
          <Label className="text-sm font-semibold text-gray-600">Mission Planner</Label>
          <Select value={planner} onValueChange={setPlanner}>
            <SelectTrigger className="h-12">
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="boustrophedon">Boustrophedon (Lawnmower)</SelectItem>
              <SelectItem value="spiral">Spiral Coverage</SelectItem>
              <SelectItem value="waypoints">Custom Waypoints</SelectItem>
            </SelectContent>
          </Select>
        </div>

        <div className="grid grid-cols-2 gap-4">
          <Button 
            variant="outline" 
            className={`h-12 gap-2 ${isBoundarySet ? 'bg-green-50 border-green-500 text-green-700' : ''}`}
            onClick={handleSetBoundary}
            disabled={isLoading || rosStatus !== "connected"}
          >
            <MapIcon className="w-4 h-4" /> 
            {isBoundarySet ? '✓ Boundary Set' : 'Set Boundary'}
          </Button>
          <Button 
            variant="outline" 
            className={`h-12 gap-2 ${isMissionLoaded ? 'bg-green-50 border-green-500 text-green-700' : ''}`}
            onClick={handleLoadMission}
            disabled={isLoading || rosStatus !== "connected" || !isBoundarySet}
          >
            <Bot className="w-4 h-4" /> 
            {isMissionLoaded ? '✓ Mission Loaded' : 'Load Mission'}
          </Button>
        </div>
        
        {pathLinesCount > 0 && (
          <div className="text-xs text-purple-600 bg-purple-50 p-3 rounded-lg border border-purple-200 flex items-center gap-2">
            <span className="text-lg">📍</span>
            <div>
              <strong>{pathLinesCount} Custom Path{pathLinesCount > 1 ? 's' : ''}</strong>
              <br />Robot will follow these lines slowly inside the boundary
            </div>
          </div>
        )}
        
        {!isBoundarySet && (
          <div className="text-xs text-orange-600 bg-orange-50 p-3 rounded-lg border border-orange-200">
            <strong>Step 1:</strong> Draw a boundary on the map, then click "Set Boundary"
          </div>
        )}
        
        {isBoundarySet && !isMissionLoaded && (
          <div className="text-xs text-blue-600 bg-blue-50 p-3 rounded-lg border border-blue-200">
            <strong>Step 2:</strong> Click "Load Mission" to generate the path
          </div>
        )}
        
        {isMissionLoaded && (
          <div className="text-xs text-green-600 bg-green-50 p-3 rounded-lg border border-green-200">
            <strong>Ready!</strong> Click "EXECUTE MISSION" below to start
          </div>
        )}
      </CardContent>
    </Card>
  );
}
