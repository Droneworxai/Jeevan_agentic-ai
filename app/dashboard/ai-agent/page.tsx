"use client";

import { useState, useEffect } from "react";
import AIAgentMapComponent from "@/components/map/ai-agent-map-component";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";

export default function AIAgentPage() {
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return (
      <div className="h-full flex items-center justify-center">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-green-600 mx-auto mb-4"></div>
          <p className="text-gray-600">Loading AI Agent...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="h-full p-6">
      <Card className="h-full shadow-lg">
        <CardHeader className="bg-purple-600 text-white">
          <CardTitle className="text-xl">
            Ecoweeder | AI Agent Navigation
          </CardTitle>
        </CardHeader>
        <CardContent className="p-0 h-[calc(100%-4rem)]">
          <AIAgentMapComponent />
        </CardContent>
      </Card>
    </div>
  );
}
