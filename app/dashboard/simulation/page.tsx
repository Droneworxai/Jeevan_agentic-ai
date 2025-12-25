"use client";

import { useState, useEffect, useRef } from "react";
import dynamic from "next/dynamic";
import { Card, CardContent } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { RotateCw, Upload, Play, Pause } from "lucide-react";
import RosClient from "@/lib/ros/ros-client";
import * as ROSLIB from "roslib";

// Components
import StepWizard from "@/components/simulation/StepWizard";
import StatusBar from "@/components/simulation/StatusBar";
import ConfigurationCard from "@/components/simulation/ConfigurationCard";
import MissionControlButtons from "@/components/simulation/MissionControlButtons";
import TelemetryCard from "@/components/simulation/TelemetryCard";
import AIAgentCard from "@/components/simulation/AIAgentCard";
import InstructionsCard from "@/components/simulation/InstructionsCard";
import RosTerminal from "@/components/simulation/RosTerminal";
import SimulationLogsTerminal from "@/components/simulation/SimulationLogsTerminal";

// Hooks
import { useRosConnection } from "@/hooks/useRosConnection";
import { useAINavigation } from "@/hooks/useAINavigation";
import { useRosLogs } from "@/hooks/useRosLogs";

const MissionControlMap = dynamic(
  () => import("@/components/map/mission-control-map"),
  { ssr: false }
);

export default function SimulationPage() {
  const [farmName, setFarmName] = useState("Sandfields Farm Ltd");
  const [planner, setPlanner] = useState("boustrophedon");
  const [boundary, setBoundary] = useState<any>(null);
  const [pathLines, setPathLines] = useState<[number, number][][]>([]);
  const [isMissionRunning, setIsMissionRunning] = useState(false);
  const [isMissionLoaded, setIsMissionLoaded] = useState(false);
  const [isBoundarySet, setIsBoundarySet] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [startTime, setStartTime] = useState<number | null>(null);
  const [elapsedTime, setElapsedTime] = useState(0);
  const [currentStep, setCurrentStep] = useState(0);
  const [completedSteps, setCompletedSteps] = useState<number[]>([]);
  const [showInstructions, setShowInstructions] = useState(true);
  
  // Ref for KML file input
  const kmlFileInputRef = useRef<HTMLInputElement>(null);

  // Custom hooks
  const { rosStatus, robotPosition, missionStatus, weedStatus } = useRosConnection(farmName);
  const aiNavigation = useAINavigation();
  const { logs, addLog, clearLogs } = useRosLogs();

  // Automatically set boundary when KML is loaded
  useEffect(() => {
    if (aiNavigation.kmlBoundary && aiNavigation.kmlLoaded) {
      setBoundary(aiNavigation.kmlBoundary);
    }
  }, [aiNavigation.kmlBoundary, aiNavigation.kmlLoaded]);

  // Log robot position updates
  useEffect(() => {
    if (robotPosition && rosStatus === "connected") {
      addLog("subscribe", `Robot position: [${robotPosition[0].toFixed(6)}, ${robotPosition[1].toFixed(6)}]`, "/robot_pose");
    }
  }, [robotPosition, rosStatus, addLog]);

  // Log mission status updates
  useEffect(() => {
    if (missionStatus) {
      addLog("subscribe", `Mission status: ${JSON.stringify(missionStatus)}`, "/mission_status");
    }
  }, [missionStatus, addLog]);

  // Timer for mission elapsed time
  useEffect(() => {
    let interval: any;
    if (isMissionRunning && startTime) {
      interval = setInterval(() => {
        setElapsedTime(Math.floor((Date.now() - startTime) / 1000));
      }, 1000);
    } else if (!isMissionRunning) {
      clearInterval(interval);
    }
    return () => clearInterval(interval);
  }, [isMissionRunning, startTime]);

  // Auto-progress steps based on ROS connection
  useEffect(() => {
    if (rosStatus === "connected") {
      addLog("connection", "✓ ROS bridge connected successfully", "ws://localhost:9090");
      setCompletedSteps((prev) => {
        if (!prev.includes(0) || !prev.includes(1)) {
          return [0, 1];
        }
        return prev;
      });
      setCurrentStep((prev) => prev < 2 ? 2 : prev);
    } else if (rosStatus === "error") {
      addLog("error", "ROS bridge connection failed");
    } else if (rosStatus === "disconnected") {
      addLog("connection", "ROS bridge disconnected");
    } else if (rosStatus === "connecting") {
      addLog("connection", "Connecting to ROS bridge...", "ws://localhost:9090");
    }
  }, [rosStatus, addLog]);

  const handleSetBoundary = () => {
    if (!boundary) {
      addLog("error", "No boundary drawn on map");
      alert("Please draw a boundary on the map first.");
      return;
    }
    const ros = RosClient.getInstance().getRos();
    if (!ros || !ros.isConnected) {
      addLog("error", "ROS is not connected");
      alert("ROS is not connected.");
      return;
    }

    setIsLoading(true);
    addLog("publish", `Publishing mission config: ${farmName}, ${planner}`, "/mission/start");

    const missionTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/mission/start",
      messageType: "std_msgs/String"
    });

    missionTopic.publish({
      data: JSON.stringify({ farmName, boundary, planner })
    });

    // Add timeout for service call
    const timeoutId = setTimeout(() => {
      setIsLoading(false);
      addLog("error", "Service call timeout - ROS services not responding. Make sure mission_planner node is running!", "/set_boundary");
      alert("Service timeout! Make sure the simulation is running (./start_sim.sh). Check ROS terminal for details.");
    }, 10000); // 10 second timeout

    addLog("service", "Calling /set_boundary service (waiting for response...)", "/set_boundary");
    const setBoundarySrv = new ROSLIB.Service({
      ros: ros,
      name: "/set_boundary",
      serviceType: "std_srvs/Trigger"
    });

    setBoundarySrv.callService({}, (result: any) => {
      clearTimeout(timeoutId);
      setIsLoading(false);
      if (result.success) {
        addLog("service", "✓ Boundary set successfully!", "/set_boundary");
        alert("Boundary set successfully!");
        setIsBoundarySet(true);
        setCompletedSteps((prev) => prev.includes(2) ? prev : [...prev, 2]);
        setCurrentStep(3);
      } else {
        addLog("error", `Failed to set boundary: ${result.message}`, "/set_boundary");
        alert("Failed to set boundary: " + result.message);
      }
    }, (error: any) => {
      clearTimeout(timeoutId);
      setIsLoading(false);
      addLog("error", `Service call failed: ${error}`, "/set_boundary");
      console.error("Service call failed:", error);
      alert("Service call failed. Check ROS bridge and make sure simulation nodes are running (./start_sim.sh).");
    });
  };

  const handleLoadMission = () => {
    const ros = RosClient.getInstance().getRos();
    if (!ros || !ros.isConnected) {
      addLog("error", "ROS is not connected");
      alert("ROS is not connected.");
      return;
    }

    if (!isBoundarySet) {
      addLog("error", "Boundary must be set before loading mission");
      alert("Please set the boundary first!");
      return;
    }

    setIsLoading(true);
    addLog("service", "Calling /load_mission service (waiting for response...)", "/load_mission");

    // Add timeout for service call
    const timeoutId = setTimeout(() => {
      setIsLoading(false);
      addLog("error", "Service call timeout - ROS services not responding. Make sure mission_planner node is running!", "/load_mission");
      alert("Service timeout! Make sure the simulation is running (./start_sim.sh). Check ROS terminal for details.");
    }, 10000); // 10 second timeout

    const loadMissionSrv = new ROSLIB.Service({
      ros: ros,
      name: "/load_mission",
      serviceType: "std_srvs/Trigger"
    });

    loadMissionSrv.callService({}, (result: any) => {
      clearTimeout(timeoutId);
      setIsLoading(false);
      if (result.success) {
        addLog("service", "✓ Mission loaded successfully!", "/load_mission");
        alert("Mission loaded successfully! You can now execute the mission.");
        setIsMissionLoaded(true);
        setCompletedSteps((prev) => prev.includes(3) ? prev : [...prev, 3]);
        setCurrentStep(4);
      } else {
        addLog("error", `Failed to load mission: ${result.message}`, "/load_mission");
        alert("Failed to load mission: " + result.message);
      }
    }, (error: any) => {
      clearTimeout(timeoutId);
      setIsLoading(false);
      addLog("error", `Service call failed: ${error}`, "/load_mission");
      console.error("Service call failed:", error);
      alert("Service call failed. Check ROS bridge and make sure simulation nodes are running (./start_sim.sh).");
    });
  };

  const handleStartMission = () => {
    const ros = RosClient.getInstance().getRos();
    if (!ros || !ros.isConnected) {
      addLog("error", "ROS is not connected");
      alert("ROS is not connected.");
      return;
    }

    if (!isMissionLoaded) {
      addLog("error", "Mission must be loaded before execution");
      alert("Please load the mission first! (Click 'Load Mission' button)");
      return;
    }

    addLog("publish", "Publishing mission execute command", "/mission/execute");
    const startTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/mission/execute",
      messageType: "std_msgs/Empty"
    });
    startTopic.publish({});
    setIsMissionRunning(true);
    setStartTime(Date.now());
    setElapsedTime(0);
    setCompletedSteps((prev) => prev.includes(4) ? prev : [...prev, 4]);
    setCurrentStep(5);
  };

  const handleStopMission = () => {
    const ros = RosClient.getInstance().getRos();
    if (ros) {
      addLog("publish", "Publishing mission stop command", "/mission/stop");
      const stopTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/mission/stop",
        messageType: "std_msgs/Empty"
      });
      stopTopic.publish({});
    }
    setIsMissionRunning(false);
    setStartTime(null);
    setIsMissionLoaded(false);
    setIsBoundarySet(false);
    addLog("connection", "Mission stopped - reset state. Draw boundary and load mission again to restart.");
  };

  // Keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.ctrlKey && e.key === 'Enter' && !isMissionRunning) {
        handleStartMission();
      } else if (e.ctrlKey && e.key === 'Escape' && isMissionRunning) {
        handleStopMission();
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isMissionRunning]);

  // Define steps array
  const steps = [
    {
      title: "Start the Simulation",
      description: "Run ./start.sh in your terminal to launch Gazebo, ROS 2 nodes, and Rosbridge WebSocket server.",
      note: "Gazebo runs in headless mode (no GUI) for stability. Watch the green car icon on the map instead!",
      action: null,
      canComplete: () => true,
    },
    {
      title: "Verify ROS Connection",
      description: "Wait for the ROS bridge to connect. The status indicator should show 'Connected'.",
      note: "This happens automatically once the simulation stack is running.",
      action: null,
      canComplete: () => rosStatus === "connected",
    },
    {
      title: "Set Farm Boundary",
      description: "Use the drawing tools on the map to create a polygon boundary, then click 'Set Boundary'.",
      note: "Draw by clicking points on the map. Double-click to finish the polygon.",
      action: handleSetBoundary,
      canComplete: () => boundary !== null && rosStatus === "connected",
    },
    {
      title: "Load Mission Plan",
      description: "Click 'Load Mission' to generate the path plan using the Boustrophedon algorithm.",
      note: "This calculates the optimal coverage path for the robot.",
      action: handleLoadMission,
      canComplete: () => rosStatus === "connected",
    },
    {
      title: "Execute Mission",
      description: "Click 'Execute Mission' or press Ctrl+Enter to start the robot!",
      note: "Watch the green car icon move on the map in real-time.",
      action: handleStartMission,
      canComplete: () => rosStatus === "connected" && !isMissionRunning,
    },
    {
      title: "Mission Complete",
      description: "The robot is moving! Monitor telemetry and use 'Abort' (Ctrl+Esc) if needed.",
      note: "The blue dashed line shows the robot's path history.",
      action: null,
      canComplete: () => isMissionRunning,
    },
  ];

  const getStatusColor = () => {
    switch (rosStatus) {
      case "connected": return "bg-green-500";
      case "connecting": return "bg-yellow-500";
      case "error": return "bg-red-500";
      default: return "bg-gray-500";
    }
  };

  return (
    <div className="min-h-screen bg-gray-50 p-4 md:p-8">
      <div className="max-w-7xl mx-auto space-y-4">
        <header className="flex justify-between items-center">
          <div>
            <h1 className="text-3xl md:text-4xl font-extrabold text-gray-900 tracking-tight">Simulation Dashboard</h1>
            <p className="text-gray-500 mt-1 text-sm md:text-base">EcoWeeder Autonomous Mission Control</p>
          </div>
          <div className="flex items-center gap-4 bg-white p-3 rounded-xl shadow-sm border">
            <div className={`w-3 h-3 rounded-full ${rosStatus === "connected" ? "" : "animate-pulse"} ${getStatusColor()}`} />
            <span className="text-xs md:text-sm font-bold text-gray-700 uppercase tracking-wider">
              {rosStatus}
            </span>
            {rosStatus !== "connected" && (
              <Button 
                variant="ghost" 
                size="sm" 
                className="h-8 px-2 text-xs"
                onClick={() => RosClient.getInstance().connect()}
              >
                <RotateCw className="w-3 h-3 mr-1" /> Retry
              </Button>
            )}
          </div>
        </header>

        <StepWizard
          currentStep={currentStep}
          setCurrentStep={setCurrentStep}
          completedSteps={completedSteps}
          setCompletedSteps={setCompletedSteps}
          showInstructions={showInstructions}
          setShowInstructions={setShowInstructions}
          steps={steps}
          isLoading={isLoading}
        />

        <StatusBar completedSteps={completedSteps} isMissionRunning={isMissionRunning} />

        <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 md:gap-8">
          {/* Control Panel */}
          <div className="lg:col-span-4 space-y-4 md:space-y-6">
            <ConfigurationCard
              farmName={farmName}
              setFarmName={setFarmName}
              planner={planner}
              setPlanner={setPlanner}
              handleSetBoundary={handleSetBoundary}
              handleLoadMission={handleLoadMission}
              isLoading={isLoading}
              rosStatus={rosStatus}
              isBoundarySet={isBoundarySet}
              isMissionLoaded={isMissionLoaded}
              pathLinesCount={pathLines.length}
            />

            <MissionControlButtons
              isMissionRunning={isMissionRunning}
              handleStartMission={handleStartMission}
              handleStopMission={handleStopMission}
              isLoading={isLoading}
              rosStatus={rosStatus}
              isMissionLoaded={isMissionLoaded}
            />

            <TelemetryCard
              missionStatus={missionStatus}
              elapsedTime={elapsedTime}
              robotPosition={robotPosition}
              rosStatus={rosStatus}
            />

            <AIAgentCard
              geminiApiKey={aiNavigation.geminiApiKey}
              setGeminiApiKey={aiNavigation.setGeminiApiKey}
              apiKeySet={aiNavigation.apiKeySet}
              handleSaveApiKey={aiNavigation.handleSaveApiKey}
              setApiKeySet={aiNavigation.setApiKeySet}
              handleKMLUpload={aiNavigation.handleKMLUpload}
              kmlLoaded={aiNavigation.kmlLoaded}
              handleClearAIData={aiNavigation.handleClearAIData}
              aiNavigationPath={aiNavigation.aiNavigationPath}
              isAINavigating={aiNavigation.isAINavigating}
              handleStartAINavigation={() => aiNavigation.handleStartAINavigation(rosStatus)}
              handlePauseAINavigation={aiNavigation.handlePauseAINavigation}
              rosStatus={rosStatus}
              aiPathIndex={aiNavigation.aiPathIndex}
            />
          </div>

          {/* Map View */}
          <div className="lg:col-span-8">
            <Card className="shadow-2xl border-none overflow-hidden h-[750px]">
              <CardContent className="p-0 h-full relative">
                <MissionControlMap 
                  onBoundaryChange={setBoundary}
                  onPathLinesChange={setPathLines}
                  robotPosition={robotPosition}
                  aiWaypoints={aiNavigation.aiNavigationPath}
                  currentWaypointIndex={aiNavigation.aiPathIndex}
                />
                
                {/* AI Navigation Controls - Right Side */}
                <div className="absolute top-6 left-6 z-[1000] flex flex-col gap-2">
                  <input
                    ref={kmlFileInputRef}
                    type="file"
                    accept=".kml"
                    onChange={aiNavigation.handleKMLUpload}
                    className="hidden"
                  />
                  
                  <Button
                    onClick={() => kmlFileInputRef.current?.click()}
                    className="bg-purple-600 hover:bg-purple-700 shadow-lg gap-2 w-full"
                    size="sm"
                  >
                    <Upload className="h-4 w-4" />
                    Load KML
                  </Button>

                  {!aiNavigation.isAINavigating ? (
                    <Button
                      onClick={() => aiNavigation.handleStartAINavigation(rosStatus)}
                      className="bg-green-600 hover:bg-green-700 shadow-lg gap-2 w-full"
                      disabled={!aiNavigation.kmlLoaded || !aiNavigation.apiKeySet || rosStatus !== "connected"}
                      size="sm"
                    >
                      <Play className="h-4 w-4" />
                      Start AI Nav
                    </Button>
                  ) : (
                    <Button
                      onClick={aiNavigation.handlePauseAINavigation}
                      className="bg-orange-600 hover:bg-orange-700 shadow-lg gap-2 w-full"
                      size="sm"
                    >
                      <Pause className="h-4 w-4" />
                      Pause
                    </Button>
                  )}
                  
                  {/* Status Indicators */}
                  {aiNavigation.kmlLoaded && (
                    <div className="bg-white/95 backdrop-blur px-3 py-2 rounded-lg shadow-lg">
                      <div className="text-[10px] space-y-1">
                        <div className="flex items-center justify-between gap-2">
                          <span className="text-gray-600">KML:</span>
                          <span className="text-green-600 font-semibold">✓ Loaded</span>
                        </div>
                        <div className="flex items-center justify-between gap-2">
                          <span className="text-gray-600">Waypoints:</span>
                          <span className="text-gray-800 font-semibold">{aiNavigation.aiNavigationPath.length}</span>
                        </div>
                        {aiNavigation.isAINavigating && (
                          <div className="flex items-center justify-between gap-2">
                            <span className="text-gray-600">Progress:</span>
                            <span className="text-blue-600 font-semibold">
                              {aiNavigation.aiPathIndex + 1}/{aiNavigation.aiNavigationPath.length}
                            </span>
                          </div>
                        )}
                      </div>
                    </div>
                  )}
                </div>
                
                {/* Map Legend */}
                {aiNavigation.aiNavigationPath.length > 0 && (
                  <div className="absolute top-6 right-6 z-[1000] bg-white/95 backdrop-blur p-3 rounded-lg shadow-xl border border-gray-200">
                    <h4 className="text-xs font-bold text-gray-700 mb-2">Map Legend</h4>
                    <div className="space-y-1.5 text-[10px]">
                      <div className="flex items-center gap-2">
                        <div className="w-4 h-4 bg-green-500 rounded-full border-2 border-white shadow"></div>
                        <span className="text-gray-600">Robot Position</span>
                      </div>
                      <div className="flex items-center gap-2">
                        <div className="w-4 h-4 bg-blue-500 rounded-full border-2 border-white shadow"></div>
                        <span className="text-gray-600">Current Waypoint</span>
                      </div>
                      <div className="flex items-center gap-2">
                        <div className="w-4 h-4 bg-green-600 rounded-full border-2 border-white shadow"></div>
                        <span className="text-gray-600">Completed</span>
                      </div>
                      <div className="flex items-center gap-2">
                        <div className="w-4 h-4 bg-gray-400 rounded-full border-2 border-white shadow"></div>
                        <span className="text-gray-600">Pending</span>
                      </div>
                      <div className="flex items-center gap-2 pt-1 border-t border-gray-200">
                        <div className="w-6 h-0.5 bg-blue-400" style={{borderTop: "2px dashed"}}></div>
                        <span className="text-gray-600">Robot Path</span>
                      </div>
                      <div className="flex items-center gap-2">
                        <div className="w-6 h-0.5 bg-purple-500" style={{borderTop: "3px dashed"}}></div>
                        <span className="text-gray-600">AI Route</span>
                      </div>
                    </div>
                  </div>
                )}
                
                <div className="absolute bottom-6 left-6 z-[1000] bg-white/90 backdrop-blur p-4 rounded-2xl shadow-2xl border border-white/20">
                  <div className="flex items-center gap-4">
                    <div className="flex flex-col">
                      <span className="text-[10px] font-bold text-gray-400 uppercase">System Status</span>
                      <span className="text-sm font-bold text-gray-800">Autonomous Mode Active</span>
                    </div>
                    <div className="h-8 w-[1px] bg-gray-200" />
                    <div className="flex flex-col">
                      <span className="text-[10px] font-bold text-gray-400 uppercase">Signal Strength</span>
                      <span className="text-sm font-bold text-green-600">98% (Excellent)</span>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </div>

        {/* Simulation Logs Terminal */}
        <SimulationLogsTerminal />

        {/* Instructions and ROS Terminal Grid */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 md:gap-8">
          <InstructionsCard />
          <RosTerminal logs={logs} onClear={clearLogs} />
        </div>
      </div>
    </div>
  );
}
