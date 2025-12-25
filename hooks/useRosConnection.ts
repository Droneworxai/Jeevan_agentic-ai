"use client";

import { useState, useEffect, useCallback } from "react";
import RosClient from "@/lib/ros/ros-client";
import * as ROSLIB from "roslib";

export function useRosConnection(farmName: string) {
  const [rosStatus, setRosStatus] = useState<"connected" | "disconnected" | "error" | "connecting">("disconnected");
  // Robot position starts as null, will be updated from ROS /robot_pose topic
  const [robotPosition, setRobotPosition] = useState<[number, number] | null>(null);
  const [missionStatus, setMissionStatus] = useState<any>(null);
  const [weedStatus, setWeedStatus] = useState<any>(null);

  useEffect(() => {
    const ros = RosClient.getInstance().connect();
    setRosStatus("connecting");

    const handleConnection = () => {
      setRosStatus("connected");
      // Publish selected farm on connect
      const farmTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/selected_farm",
        messageType: "std_msgs/String"
      });
      farmTopic.publish({ data: farmName });
    };

    const handleError = () => setRosStatus("error");
    const handleClose = () => setRosStatus("disconnected");

    ros.on("connection", handleConnection);
    ros.on("error", handleError);
    ros.on("close", handleClose);

    // Subscribe to robot pose (GPS coordinates from mission_monitor)
    const poseTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/robot_pose",
      messageType: "geometry_msgs/PoseStamped"
    });

    poseTopic.subscribe((message: any) => {
      // Message now contains GPS coordinates (lat, lon) directly from mission_monitor
      // pose.position.x = latitude, pose.position.y = longitude
      const lat = message.pose.position.x;
      const lon = message.pose.position.y;
      setRobotPosition([lat, lon]);
    });

    // Subscribe to mission status
    const statusTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/mission_status",
      messageType: "std_msgs/String"
    });

    statusTopic.subscribe((message: any) => {
      try {
        setMissionStatus(JSON.parse(message.data));
      } catch (e) {
        console.error("Error parsing mission status", e);
      }
    });

    // Subscribe to weed status
    const weedTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/weed_status",
      messageType: "std_msgs/String"
    });

    weedTopic.subscribe((message: any) => {
      try {
        setWeedStatus(JSON.parse(message.data));
      } catch (e) {
        console.error("Error parsing weed status", e);
      }
    });

    return () => {
      poseTopic.unsubscribe();
      statusTopic.unsubscribe();
      weedTopic.unsubscribe();
      ros.off("connection", handleConnection);
      ros.off("error", handleError);
      ros.off("close", handleClose);
    };
  }, [farmName]);

  return { rosStatus, robotPosition, missionStatus, weedStatus };
}
