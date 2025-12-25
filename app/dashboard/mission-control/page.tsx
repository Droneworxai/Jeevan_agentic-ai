"use client";

import { useState, useEffect } from "react";
import dynamic from "next/dynamic";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Play, Square, Save, Map as MapIcon, Bot, Activity } from "lucide-react";
import RosClient from "@/lib/ros/ros-client";
import ROSLIB from "roslib";

const MissionControlMap = dynamic(
  () => import("@/components/map/mission-control-map"),
  { ssr: false }
);

export default function MissionControlPage() {
  const [farmName, setFarmName] = useState("");
  const [planner, setPlanner] = useState("lawnmower");
  const [boundary, setBoundary] = useState<any>(null);
  const [robotPosition, setRobotPosition] = useState<[number, number] | null>(null);
  const [isMissionRunning, setIsMissionRunning] = useState(false);
  const [rosStatus, setRosStatus] = useState<"connected" | "disconnected" | "error">("disconnected");

  useEffect(() => {
    const ros = RosClient.getInstance().connect();

    ros.on("connection", () => setRosStatus("connected"));
    ros.on("error", () => setRosStatus("error"));
    ros.on("close", () => setRosStatus("disconnected"));

    // Subscribe to robot position (odom)
    const odomTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/odom",
      messageType: "nav_msgs/Odometry"
    });

    odomTopic.subscribe((message: any) => {
      // Convert ROS coordinates to Lat/Lon (simplified for demo)
      // In a real scenario, you'd use a transform or GPS coordinates
      const lat = 27.7172 + (message.pose.pose.position.y / 111111);
      const lon = 85.324 + (message.pose.pose.position.x / (111111 * Math.cos(27.7172 * Math.PI / 180)));
      setRobotPosition([lat, lon]);
    });

    return () => {
      odomTopic.unsubscribe();
      RosClient.getInstance().disconnect();
    };
  }, []);

  const handleStartMission = () => {
    if (!boundary) {
      alert("Please set a farm boundary first!");
      return;
    }

    const ros = RosClient.getInstance().getRos();
    if (!ros || !ros.isConnected) {
      alert("ROS is not connected. Please start the simulation and bridge.");
      return;
    }

    // Send mission to ROS
    const missionTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/mission/start",
      messageType: "std_msgs/String"
    });

    const missionData = JSON.stringify({
      planner,
      boundary,
      farmName
    });

    missionTopic.publish(new ROSLIB.Message({ data: missionData }));
    setIsMissionRunning(true);
  };

  const handleStopMission = () => {
    const ros = RosClient.getInstance().getRos();
    if (ros) {
      const stopTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/mission/stop",
        messageType: "std_msgs/Empty"
      });
      stopTopic.publish(new ROSLIB.Message({}));
    }
    setIsMissionRunning(false);
  };

  const handleSaveFarm = async () => {
    if (!farmName || !boundary) {
      alert("Please provide a farm name and draw a boundary.");
      return;
    }

    try {
      const response = await fetch("/api/farms", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: farmName, boundary })
      });

      if (response.ok) {
        alert("Farm saved successfully!");
      } else {
        alert("Failed to save farm.");
      }
    } catch (error) {
      console.error("Error saving farm:", error);
    }
  };

  return (
    <div className="p-6 space-y-6">
      <div className="flex justify-between items-center">
        <h1 className="text-3xl font-bold text-gray-900">Mission Control</h1>
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${rosStatus === "connected" ? "bg-green-500" : "bg-red-500"}`} />
          <span className="text-sm font-medium text-gray-600">
            ROS: {rosStatus.charAt(0).toUpperCase() + rosStatus.slice(1)}
          </span>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Sidebar Controls */}
        <div className="lg:col-span-1 space-y-6">
          <Card>
            <CardHeader>
              <CardTitle className="text-lg flex items-center gap-2">
                <MapIcon className="w-5 h-5" /> Farm Setup
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="space-y-2">
                <Label htmlFor="farm-name">Farm Name</Label>
                <Input 
                  id="farm-name" 
                  placeholder="Enter farm name" 
                  value={farmName}
                  onChange={(e) => setFarmName(e.target.value)}
                />
              </div>
              <Button className="w-full gap-2" onClick={handleSaveFarm}>
                <Save className="w-4 h-4" /> Save Farm
              </Button>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle className="text-lg flex items-center gap-2">
                <Bot className="w-5 h-5" /> Mission Planner
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="space-y-2">
                <Label>Select Planner</Label>
                <Select value={planner} onValueChange={setPlanner}>
                  <SelectTrigger>
                    <SelectValue placeholder="Select algorithm" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="lawnmower">Lawnmower Pattern</SelectItem>
                    <SelectItem value="spiral">Spiral Pattern</SelectItem>
                    <SelectItem value="random">Random Coverage</SelectItem>
                  </SelectContent>
                </Select>
              </div>
              
              {!isMissionRunning ? (
                <Button className="w-full bg-green-600 hover:bg-green-700 gap-2" onClick={handleStartMission}>
                  <Play className="w-4 h-4" /> Start Mission
                </Button>
              ) : (
                <Button className="w-full bg-red-600 hover:bg-red-700 gap-2" onClick={handleStopMission}>
                  <Square className="w-4 h-4" /> Stop Mission
                </Button>
              )}
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle className="text-lg flex items-center gap-2">
                <Activity className="w-5 h-5" /> Bot Status
              </CardTitle>
            </CardHeader>
            <CardContent className="text-sm space-y-2">
              <div className="flex justify-between">
                <span className="text-gray-500">Status:</span>
                <span className="font-medium">{isMissionRunning ? "Active" : "Idle"}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-gray-500">Position:</span>
                <span className="font-medium">
                  {robotPosition ? `${robotPosition[0].toFixed(4)}, ${robotPosition[1].toFixed(4)}` : "Unknown"}
                </span>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Map Area */}
        <div className="lg:col-span-3">
          <Card className="h-full">
            <CardContent className="p-0 h-[600px]">
              <MissionControlMap 
                onBoundaryChange={setBoundary} 
                robotPosition={robotPosition}
              />
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}
