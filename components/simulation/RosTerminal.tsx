"use client";

import { Card, CardHeader, CardTitle, CardContent } from "@/components/ui/card";
import { useEffect, useRef } from "react";
import { Terminal, Trash2 } from "lucide-react";
import { Button } from "@/components/ui/button";

interface RosLog {
  timestamp: string;
  type: "publish" | "subscribe" | "service" | "connection" | "error";
  topic?: string;
  data?: any;
  message: string;
}

interface RosTerminalProps {
  logs: RosLog[];
  onClear: () => void;
}

export default function RosTerminal({ logs, onClear }: RosTerminalProps) {
  const terminalRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // Auto-scroll to bottom when new logs arrive
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [logs]);

  const getLogColor = (type: string) => {
    switch (type) {
      case "publish": return "text-blue-400";
      case "subscribe": return "text-green-400";
      case "service": return "text-purple-400";
      case "connection": return "text-cyan-400";
      case "error": return "text-red-400";
      default: return "text-gray-400";
    }
  };

  const getLogPrefix = (type: string) => {
    switch (type) {
      case "publish": return "→";
      case "subscribe": return "←";
      case "service": return "⚙";
      case "connection": return "●";
      case "error": return "✗";
      default: return "•";
    }
  };

  return (
    <Card className="h-full flex flex-col">
      <CardHeader className="pb-3">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Terminal className="h-4 w-4 text-green-500" />
            <CardTitle className="text-sm">ROS Terminal</CardTitle>
          </div>
          <Button
            variant="ghost"
            size="sm"
            onClick={onClear}
            className="h-7 px-2"
          >
            <Trash2 className="h-3 w-3" />
          </Button>
        </div>
      </CardHeader>
      <CardContent className="flex-1 overflow-hidden p-0">
        <div
          ref={terminalRef}
          className="h-full overflow-y-auto bg-gray-900 text-xs font-mono p-3 space-y-1"
        >
          {logs.length === 0 ? (
            <div className="text-gray-500 text-center py-4">
              Waiting for ROS messages...
            </div>
          ) : (
            logs.map((log, index) => (
              <div key={index} className="flex gap-2">
                <span className="text-gray-600">{log.timestamp}</span>
                <span className={getLogColor(log.type)}>
                  {getLogPrefix(log.type)}
                </span>
                <span className={getLogColor(log.type)}>{log.type.toUpperCase()}</span>
                {log.topic && (
                  <span className="text-yellow-400">{log.topic}</span>
                )}
                <span className="text-gray-300 flex-1">{log.message}</span>
              </div>
            ))
          )}
        </div>
      </CardContent>
    </Card>
  );
}
