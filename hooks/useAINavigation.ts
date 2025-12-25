"use client";

import { useState, useEffect } from "react";
import RosClient from "@/lib/ros/ros-client";
import * as ROSLIB from "roslib";
import { parseKMLToGeoJSON } from "@/lib/kml-parser";

export function useAINavigation() {
  const [geminiApiKey, setGeminiApiKey] = useState("");
  const [apiKeySet, setApiKeySet] = useState(false);
  const [kmlFile, setKmlFile] = useState<File | null>(null);
  const [kmlLoaded, setKmlLoaded] = useState(false);
  const [isAINavigating, setIsAINavigating] = useState(false);
  const [aiNavigationPath, setAiNavigationPath] = useState<[number, number][]>([]);
  const [aiPathIndex, setAiPathIndex] = useState(0);
  const [kmlBoundary, setKmlBoundary] = useState<any>(null);

  // Load saved Gemini API key
  useEffect(() => {
    const savedKey = localStorage.getItem("geminiApiKey");
    if (savedKey) {
      setGeminiApiKey(savedKey);
      setApiKeySet(true);
    }
  }, []);

  const handleSaveApiKey = () => {
    if (geminiApiKey.trim()) {
      localStorage.setItem("geminiApiKey", geminiApiKey);
      setApiKeySet(true);
      alert("Gemini API Key saved successfully!");
    } else {
      alert("Please enter a valid API key");
    }
  };

  const handleKMLUpload = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = async (e) => {
      try {
        const kmlText = e.target?.result as string;
        const geoJSON = parseKMLToGeoJSON(kmlText);
        
        const waypoints: [number, number][] = [];
        if (geoJSON && geoJSON.features) {
          geoJSON.features.forEach((feature: any) => {
            if (feature.geometry.type === "Point") {
              const [lng, lat] = feature.geometry.coordinates;
              waypoints.push([lat, lng]);
            } else if (feature.geometry.type === "LineString") {
              feature.geometry.coordinates.forEach((coord: number[]) => {
                waypoints.push([coord[1], coord[0]]);
              });
            } else if (feature.geometry.type === "Polygon") {
              feature.geometry.coordinates[0].forEach((coord: number[]) => {
                waypoints.push([coord[1], coord[0]]);
              });
            }
          });
        }

        if (waypoints.length > 0) {
          setAiNavigationPath(waypoints);
          setKmlFile(file);
          setKmlLoaded(true);
          setKmlBoundary(geoJSON); // Store the full geoJSON for boundary
          alert(`KML loaded successfully! Found ${waypoints.length} waypoints.`);
        } else {
          alert("No waypoints found in KML file.");
        }
      } catch (error) {
        console.error("Error parsing KML:", error);
        alert("Error loading KML file. Please check the file format.");
      }
    };
    reader.readAsText(file);
  };

  const handleStartAINavigation = (rosStatus: string) => {
    if (!apiKeySet) {
      alert("Please set your Gemini API key first!");
      return;
    }
    if (!kmlLoaded || aiNavigationPath.length === 0) {
      alert("Please load a KML file with waypoints first!");
      return;
    }
    if (rosStatus !== "connected") {
      alert("ROS must be connected to start AI navigation!");
      return;
    }

    setIsAINavigating(true);
    setAiPathIndex(0);
    alert(`Starting AI navigation through ${aiNavigationPath.length} waypoints!`);
  };

  const handlePauseAINavigation = () => {
    setIsAINavigating(false);
  };

  const handleClearAIData = () => {
    setKmlFile(null);
    setKmlLoaded(false);
    setIsAINavigating(false);
    setAiNavigationPath([]);
    setAiPathIndex(0);
    setKmlBoundary(null);
  };

  // AI Navigation Animation
  useEffect(() => {
    if (!isAINavigating || aiNavigationPath.length === 0) return;

    const interval = setInterval(() => {
      setAiPathIndex((prev) => {
        if (prev >= aiNavigationPath.length - 1) {
          setIsAINavigating(false);
          alert("AI navigation completed!");
          return prev;
        }

        const nextIndex = prev + 1;
        const nextWaypoint = aiNavigationPath[nextIndex];

        // Publish waypoint to ROS
        const ros = RosClient.getInstance().getRos();
        if (ros && ros.isConnected) {
          const waypointTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/ai_waypoint",
            messageType: "geometry_msgs/PoseStamped"
          });

          waypointTopic.publish({
            header: { frame_id: "map" },
            pose: {
              position: {
                x: (nextWaypoint[1] + 1.755) * 111111 * Math.cos(52.175 * Math.PI / 180),
                y: (nextWaypoint[0] - 52.175) * 111111,
                z: 0
              },
              orientation: { x: 0, y: 0, z: 0, w: 1 }
            }
          });
        }

        return nextIndex;
      });
    }, 2000);

    return () => clearInterval(interval);
  }, [isAINavigating, aiNavigationPath]);

  return {
    geminiApiKey,
    setGeminiApiKey,
    apiKeySet,
    setApiKeySet,
    handleSaveApiKey,
    kmlLoaded,
    handleKMLUpload,
    handleClearAIData,
    aiNavigationPath,
    isAINavigating,
    handleStartAINavigation,
    handlePauseAINavigation,
    aiPathIndex,
    kmlBoundary,
  };
}
