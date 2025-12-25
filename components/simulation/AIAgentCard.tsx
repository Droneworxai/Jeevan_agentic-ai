"use client";

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Brain, Settings, Play, Pause, Trash2 } from "lucide-react";

interface AIAgentCardProps {
  geminiApiKey: string;
  setGeminiApiKey: (key: string) => void;
  apiKeySet: boolean;
  handleSaveApiKey: () => void;
  setApiKeySet: (set: boolean) => void;
  handleKMLUpload: (e: React.ChangeEvent<HTMLInputElement>) => void;
  kmlLoaded: boolean;
  handleClearAIData: () => void;
  aiNavigationPath: [number, number][];
  isAINavigating: boolean;
  handleStartAINavigation: () => void;
  handlePauseAINavigation: () => void;
  rosStatus: string;
  aiPathIndex: number;
}

export default function AIAgentCard({
  geminiApiKey,
  setGeminiApiKey,
  apiKeySet,
  handleSaveApiKey,
  setApiKeySet,
  handleKMLUpload,
  kmlLoaded,
  handleClearAIData,
  aiNavigationPath,
  isAINavigating,
  handleStartAINavigation,
  handlePauseAINavigation,
  rosStatus,
  aiPathIndex,
}: AIAgentCardProps) {
  return (
    <Card className="shadow-lg border-none mt-4 md:mt-6">
      <CardHeader className="border-b bg-gradient-to-r from-purple-50 to-indigo-50">
        <CardTitle className="text-xl flex items-center gap-3 text-purple-900">
          <Brain className="w-6 h-6 text-purple-600" /> AI Agent Navigation
        </CardTitle>
      </CardHeader>
      <CardContent className="p-6 space-y-4">
        <div className="space-y-3">
          <div>
            <Label htmlFor="gemini-key" className="text-sm font-semibold text-gray-600">Gemini API Key</Label>
            <div className="flex gap-2 mt-1">
              <Input
                id="gemini-key"
                type="password"
                placeholder="Enter your Gemini API key"
                value={geminiApiKey}
                onChange={(e) => setGeminiApiKey(e.target.value)}
                disabled={apiKeySet}
                className="h-10"
              />
              {!apiKeySet ? (
                <Button onClick={handleSaveApiKey} className="h-10 bg-purple-600 hover:bg-purple-700">
                  <Settings className="h-4 w-4" />
                </Button>
              ) : (
                <Button
                  onClick={() => {
                    setApiKeySet(false);
                    setGeminiApiKey("");
                    localStorage.removeItem("geminiApiKey");
                  }}
                  variant="outline"
                  className="h-10"
                >
                  Change
                </Button>
              )}
            </div>
            {apiKeySet && (
              <p className="text-xs text-green-600 mt-1 flex items-center gap-1">
                ✓ API Key configured
              </p>
            )}
          </div>

          <div>
            <Label htmlFor="kml-upload" className="text-sm font-semibold text-gray-600">Upload KML File</Label>
            <div className="flex gap-2 mt-1">
              <Input
                id="kml-upload"
                type="file"
                accept=".kml"
                onChange={handleKMLUpload}
                className="h-10"
              />
              {kmlLoaded && (
                <Button
                  onClick={handleClearAIData}
                  variant="outline"
                  className="h-10"
                >
                  <Trash2 className="h-4 w-4" />
                </Button>
              )}
            </div>
            {kmlLoaded && (
              <p className="text-xs text-green-600 mt-1 flex items-center gap-1">
                ✓ {aiNavigationPath.length} waypoints loaded
              </p>
            )}
          </div>
        </div>

        <div className="grid grid-cols-2 gap-3">
          {!isAINavigating ? (
            <Button
              onClick={handleStartAINavigation}
              disabled={!apiKeySet || !kmlLoaded || rosStatus !== "connected"}
              className="h-11 bg-gradient-to-r from-purple-600 to-indigo-600 hover:from-purple-700 hover:to-indigo-700 gap-2"
            >
              <Play className="h-4 w-4" />
              Start AI Nav
            </Button>
          ) : (
            <Button
              onClick={handlePauseAINavigation}
              className="h-11 bg-orange-600 hover:bg-orange-700 gap-2"
            >
              <Pause className="h-4 w-4" />
              Pause
            </Button>
          )}
          
          <Button
            onClick={handleClearAIData}
            variant="outline"
            className="h-11 gap-2"
            disabled={!kmlLoaded && !isAINavigating}
          >
            <Trash2 className="h-4 w-4" />
            Clear All
          </Button>
        </div>

        {/* AI Status */}
        <div className="border-t pt-4 space-y-2">
          <h4 className="font-semibold text-sm text-gray-700">AI Navigation Status</h4>
          <div className="grid grid-cols-2 gap-3 text-xs">
            <div className="flex justify-between p-2 bg-gray-50 rounded">
              <span className="text-gray-600">API Key:</span>
              <span className={apiKeySet ? "text-green-600 font-semibold" : "text-gray-400"}>
                {apiKeySet ? "✓ Set" : "✗ Not Set"}
              </span>
            </div>
            <div className="flex justify-between p-2 bg-gray-50 rounded">
              <span className="text-gray-600">KML:</span>
              <span className={kmlLoaded ? "text-green-600 font-semibold" : "text-gray-400"}>
                {kmlLoaded ? "✓ Loaded" : "✗ No File"}
              </span>
            </div>
            <div className="flex justify-between p-2 bg-gray-50 rounded">
              <span className="text-gray-600">Navigation:</span>
              <span className={isAINavigating ? "text-blue-600 font-semibold" : "text-gray-400"}>
                {isAINavigating ? "▶ Active" : "⏸ Idle"}
              </span>
            </div>
            <div className="flex justify-between p-2 bg-gray-50 rounded">
              <span className="text-gray-600">Waypoints:</span>
              <span className="text-gray-700 font-semibold">
                {aiNavigationPath.length}
              </span>
            </div>
          </div>
        </div>

        {/* AI Progress Bar */}
        {isAINavigating && aiNavigationPath.length > 0 && (
          <div className="border-t pt-4">
            <h4 className="font-semibold text-sm text-gray-700 mb-2">Navigation Progress</h4>
            <div className="bg-gray-200 rounded-full h-3 mb-2 overflow-hidden">
              <div
                className="bg-gradient-to-r from-purple-600 to-indigo-600 h-3 rounded-full transition-all duration-300"
                style={{
                  width: `${(aiPathIndex / (aiNavigationPath.length - 1)) * 100}%`,
                }}
              />
            </div>
            <p className="text-xs text-gray-600">
              Waypoint {aiPathIndex + 1} of {aiNavigationPath.length}
            </p>
          </div>
        )}

        {/* AI Info */}
        <div className="bg-purple-50 border-l-4 border-purple-400 p-3 rounded">
          <p className="text-xs text-purple-900">
            <strong>ℹ️ AI Navigation:</strong> Upload a KML file with waypoints, set your Gemini API key, and let the AI guide the robot through the path autonomously.
          </p>
        </div>
      </CardContent>
    </Card>
  );
}
