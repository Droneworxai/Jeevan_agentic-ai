"use client";

import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Settings } from "lucide-react";

export default function InstructionsCard() {
  return (
    <Card className="mt-6 shadow-lg border-l-4 border-l-blue-600">
      <CardHeader className="bg-blue-50/50">
        <CardTitle className="text-lg flex items-center gap-2 text-blue-900">
          <Settings className="w-5 h-5" /> Mission Instructions
        </CardTitle>
      </CardHeader>
      <CardContent className="p-6 space-y-4">
        <div className="space-y-3">
          <div className="flex items-start gap-3">
            <div className="flex-shrink-0 w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center font-bold">
              1
            </div>
            <div>
              <h4 className="font-semibold text-gray-900">Start the Simulation Stack</h4>
              <p className="text-sm text-gray-600 mt-1">
                Open a terminal and run: <code className="bg-gray-100 px-2 py-1 rounded text-xs font-mono">./start.sh</code> from the project root.
                This will launch Gazebo (headless mode), ROS 2 nodes, and the Rosbridge WebSocket server.
              </p>
            </div>
          </div>

          <div className="flex items-start gap-3">
            <div className="flex-shrink-0 w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center font-bold">
              2
            </div>
            <div>
              <h4 className="font-semibold text-gray-900">Verify ROS Connection</h4>
              <p className="text-sm text-gray-600 mt-1">
                Check that the status indicator at the top shows <span className="text-green-600 font-semibold">"Connected"</span>.
                If it shows "Disconnected", wait a few seconds for the system to initialize.
              </p>
            </div>
          </div>

          <div className="flex items-start gap-3">
            <div className="flex-shrink-0 w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center font-bold">
              3
            </div>
            <div>
              <h4 className="font-semibold text-gray-900">Set Farm Boundary</h4>
              <p className="text-sm text-gray-600 mt-1">
                Click on the map to draw a polygon around the area you want the robot to cover.
                Then click <span className="font-semibold">"Set Boundary"</span> to send it to the robot.
              </p>
            </div>
          </div>

          <div className="flex items-start gap-3">
            <div className="flex-shrink-0 w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center font-bold">
              4
            </div>
            <div>
              <h4 className="font-semibold text-gray-900">Load Mission Plan</h4>
              <p className="text-sm text-gray-600 mt-1">
                Click <span className="font-semibold">"Load Mission"</span> to generate the path plan using the selected algorithm (Boustrophedon pattern).
              </p>
            </div>
          </div>

          <div className="flex items-start gap-3">
            <div className="flex-shrink-0 w-8 h-8 bg-blue-600 text-white rounded-full flex items-center justify-center font-bold">
              5
            </div>
            <div>
              <h4 className="font-semibold text-gray-900">Execute Mission</h4>
              <p className="text-sm text-gray-600 mt-1">
                Click <span className="font-semibold">"Execute Mission"</span> to start the robot. Watch it move on the map in real-time!
                Use <kbd className="px-2 py-1 bg-gray-200 rounded text-xs font-mono">Ctrl+Enter</kbd> as a shortcut.
              </p>
            </div>
          </div>

          <div className="flex items-start gap-3">
            <div className="flex-shrink-0 w-8 h-8 bg-red-600 text-white rounded-full flex items-center justify-center font-bold">
              6
            </div>
            <div>
              <h4 className="font-semibold text-gray-900">Abort Mission (If Needed)</h4>
              <p className="text-sm text-gray-600 mt-1">
                Click <span className="font-semibold">"Abort Mission"</span> to stop the robot immediately.
                Use <kbd className="px-2 py-1 bg-gray-200 rounded text-xs font-mono">Ctrl+Esc</kbd> as a shortcut.
              </p>
            </div>
          </div>
        </div>

        <div className="mt-6 p-4 bg-blue-50 border-l-4 border-blue-400 rounded mb-4">
          <h4 className="font-semibold text-blue-900 flex items-center gap-2">
            <span className="text-xl">ℹ️</span> About Gazebo Simulation
          </h4>
          <p className="mt-2 text-sm text-blue-800">
            Gazebo runs in <span className="font-semibold">headless mode</span> (no GUI) for stability and performance. 
            You won't see a 3D simulation window - instead, watch the <span className="font-semibold text-green-600">green car icon</span> move on the map above in real-time! 
            The robot's position is synchronized from Gazebo physics to this web interface via ROS 2 topics.
          </p>
        </div>

        <div className="mt-6 p-4 bg-yellow-50 border-l-4 border-yellow-400 rounded">
          <h4 className="font-semibold text-yellow-900 flex items-center gap-2">
            <span className="text-xl">⚠️</span> Troubleshooting
          </h4>
          <ul className="mt-2 space-y-1 text-sm text-yellow-800">
            <li>• <strong>No Robot Movement?</strong> Follow all 6 steps in order: Start Sim → Wait for Connection → Set Boundary → Load Mission → Execute</li>
            <li>• <strong>ROS Disconnected?</strong> Ensure the simulation stack is running with <code className="bg-yellow-100 px-1 rounded">./start.sh</code></li>
            <li>• <strong>Robot Not Visible?</strong> Look for the green car icon on the map - it appears when position data is received</li>
            <li>• Check <code className="bg-yellow-100 px-1 rounded">simulation.log</code> in the project root for any ROS errors</li>
            <li>• MongoDB connection errors are non-critical (only needed for farm persistence)</li>
            <li>• The robot path (blue dashed line) will appear after the mission starts</li>
          </ul>
        </div>
      </CardContent>
    </Card>
  );
}
