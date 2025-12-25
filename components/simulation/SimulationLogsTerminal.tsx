"use client";

import { useEffect, useRef, useState } from "react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Trash2, Pause, Play } from "lucide-react";

interface SimulationLogsTerminalProps {
  autoScroll?: boolean;
}

export default function SimulationLogsTerminal({ autoScroll = true }: SimulationLogsTerminalProps) {
  const [logs, setLogs] = useState<string[]>([]);
  const [isPaused, setIsPaused] = useState(false);
  const terminalRef = useRef<HTMLDivElement>(null);
  const lastLineRef = useRef<string>("");

  useEffect(() => {
    if (isPaused) return;

    const fetchLogs = async () => {
      try {
        const response = await fetch("/api/simulation-logs");
        if (response.ok) {
          const data = await response.json();
          if (data.logs && data.logs !== lastLineRef.current) {
            lastLineRef.current = data.logs;
            const logLines = data.logs.split("\n").filter((line: string) => line.trim());
            setLogs(logLines.slice(-100)); // Keep last 100 lines
          }
        }
      } catch (error) {
        console.error("Error fetching simulation logs:", error);
      }
    };

    fetchLogs();
    const interval = setInterval(fetchLogs, 1000); // Poll every second

    return () => clearInterval(interval);
  }, [isPaused]);

  useEffect(() => {
    if (autoScroll && terminalRef.current && !isPaused) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [logs, autoScroll, isPaused]);

  const handleClear = () => {
    setLogs([]);
    lastLineRef.current = "";
  };

  const getLogColor = (line: string) => {
    if (line.includes("[ERROR]") || line.includes("error") || line.includes("Error")) {
      return "text-red-400";
    }
    if (line.includes("[WARN]") || line.includes("Warning")) {
      return "text-yellow-400";
    }
    if (line.includes("[INFO]") || line.includes("INFO")) {
      return "text-blue-400";
    }
    if (line.includes("Starting") || line.includes("started")) {
      return "text-green-400";
    }
    if (line.includes("Finished") || line.includes("completed")) {
      return "text-cyan-400";
    }
    return "text-gray-300";
  };

  return (
    <Card className="h-[400px] flex flex-col">
      <CardHeader className="pb-3 flex-shrink-0">
        <div className="flex items-center justify-between">
          <CardTitle className="text-lg font-bold">🖥️ Simulation Output</CardTitle>
          <div className="flex gap-2">
            <Button
              variant="ghost"
              size="sm"
              onClick={() => setIsPaused(!isPaused)}
              className="h-8 w-8 p-0"
              title={isPaused ? "Resume" : "Pause"}
            >
              {isPaused ? <Play className="h-4 w-4" /> : <Pause className="h-4 w-4" />}
            </Button>
            <Button
              variant="ghost"
              size="sm"
              onClick={handleClear}
              className="h-8 w-8 p-0"
              title="Clear logs"
            >
              <Trash2 className="h-4 w-4" />
            </Button>
          </div>
        </div>
      </CardHeader>
      <CardContent className="flex-1 overflow-hidden p-0">
        <div
          ref={terminalRef}
          className="h-full overflow-y-auto bg-gray-950 p-4 font-mono text-xs rounded-b-lg"
          style={{ scrollBehavior: "smooth" }}
        >
          {logs.length === 0 ? (
            <div className="text-gray-500 italic">Waiting for simulation logs...</div>
          ) : (
            logs.map((log, index) => (
              <div key={index} className={`${getLogColor(log)} leading-relaxed`}>
                {log}
              </div>
            ))
          )}
        </div>
      </CardContent>
    </Card>
  );
}
