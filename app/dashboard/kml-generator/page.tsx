"use client";

import { useState, useEffect } from "react";
import MapboxDrawMapComponent from "@/components/map/mapbox-draw-map-component";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";

export default function KMLGeneratorPage() {
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return (
      <div className="h-full flex items-center justify-center">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-green-600 mx-auto mb-4"></div>
          <p className="text-gray-600">Loading map...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="h-full p-6">
      <Card className="h-full shadow-lg">
        <CardHeader className="bg-green-600 text-white">
          <CardTitle className="text-xl">
            Ecoweeder | Download KML File
          </CardTitle>
        </CardHeader>
        <CardContent className="p-0 h-[calc(100%-4rem)]">
          <MapboxDrawMapComponent />
        </CardContent>
      </Card>
    </div>
  );
}
