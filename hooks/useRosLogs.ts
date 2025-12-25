"use client";

import { useState, useCallback } from "react";

export interface RosLog {
  timestamp: string;
  type: "publish" | "subscribe" | "service" | "connection" | "error";
  topic?: string;
  data?: any;
  message: string;
}

export function useRosLogs() {
  const [logs, setLogs] = useState<RosLog[]>([]);

  const addLog = useCallback((
    type: RosLog["type"],
    message: string,
    topic?: string,
    data?: any
  ) => {
    const timestamp = new Date().toLocaleTimeString("en-US", {
      hour12: false,
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      fractionalSecondDigits: 3
    });

    setLogs((prev) => [
      ...prev.slice(-99), // Keep last 100 logs
      { timestamp, type, message, topic, data }
    ]);
  }, []);

  const clearLogs = useCallback(() => {
    setLogs([]);
  }, []);

  return { logs, addLog, clearLogs };
}
