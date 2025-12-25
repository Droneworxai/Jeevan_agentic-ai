"use client";

import { useEffect, useRef, useState } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import "leaflet-draw/dist/leaflet.draw.css";
import "leaflet-draw";
import { Button } from "@/components/ui/button";
import { Download, Trash2 } from "lucide-react";
import { convertGeoJSONToKML } from "@/lib/kml-converter";

interface MapboxDrawMapComponentProps {
  mapRef: React.MutableRefObject<L.Map | null>;
}

export default function MapboxDrawMapComponent({ mapRef }: MapboxDrawMapComponentProps) {
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const drawnItemsRef = useRef<L.FeatureGroup | null>(null);
  const [geoJSON, setGeoJSON] = useState<any>({
    type: "FeatureCollection",
    features: [],
  });
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
  }, []);

  useEffect(() => {
    if (!mounted || !mapContainerRef.current || mapRef.current) return;

    const map = L.map(mapContainerRef.current).setView([52.175, -1.755], 16);
    mapRef.current = map;

    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      maxZoom: 19,
    }).addTo(map);

    const drawnItems = new L.FeatureGroup();
    map.addLayer(drawnItems);
    drawnItemsRef.current = drawnItems;

    const drawControl = new L.Control.Draw({
      edit: {
        featureGroup: drawnItems,
        remove: true,
      },
      draw: {
        polygon: {
          allowIntersection: false,
          showArea: true,
          drawError: {
            color: "#e74c3c",
            message: "<strong>Error:</strong> shape edges cannot cross!",
          },
          shapeOptions: {
            color: "#16a34a",
            weight: 3,
          },
        },
        rectangle: {
          shapeOptions: {
            color: "#16a34a",
            weight: 3,
          },
        },
        circle: {
          shapeOptions: {
            color: "#16a34a",
            weight: 3,
          },
        },
        circlemarker: false,
        marker: {},
        polyline: {
          shapeOptions: {
            color: "#16a34a",
            weight: 3,
          },
        },
      },
    });
    map.addControl(drawControl);

    map.on(L.Draw.Event.CREATED, (e: any) => {
      const layer = e.layer;
      drawnItems.addLayer(layer);
      updateGeoJSON();
    });

    map.on(L.Draw.Event.EDITED, () => {
      updateGeoJSON();
    });

    map.on(L.Draw.Event.DELETED, () => {
      updateGeoJSON();
    });

    const updateGeoJSON = () => {
      const data = drawnItems.toGeoJSON();
      setGeoJSON(data);
    };

    return () => {
        if (mapRef.current) {
            mapRef.current.remove();
            mapRef.current = null;
        }
    };
  }, [mounted, mapRef]);

  const handleClear = () => {
    if (drawnItemsRef.current) {
      drawnItemsRef.current.clearLayers();
      setGeoJSON({ type: "FeatureCollection", features: [] });
    }
  };

  const handleDownloadKML = () => {
    if (geoJSON.features.length === 0) {
      alert("Please draw some shapes on the map first!");
      return;
    }

    const kmlContent = convertGeoJSONToKML(geoJSON);
    const blob = new Blob([kmlContent], {
      type: "application/vnd.google-earth.kml+xml",
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `ecoweeder-${Date.now()}.kml`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  return (
    <div className="flex h-full">
      <div className="flex-1 relative">
        <div ref={mapContainerRef} className="absolute inset-0" />
      </div>
      <div className="w-96 bg-white border-l border-gray-200 p-4 overflow-auto">
        <div className="space-y-4">
          <div>
            <h3 className="text-lg font-semibold mb-2">Draw Controls</h3>
            <p className="text-sm text-gray-600 mb-4">
              Use the tools on the map to draw polygons, rectangles, circles,
              lines, and markers.
            </p>
          </div>

          <div className="space-y-2">
            <Button
              variant="outline"
              className="w-full justify-start"
              onClick={handleClear}
            >
              <Trash2 className="h-4 w-4 mr-2" />
              Clear All
            </Button>
          </div>

          <div className="border-t pt-4">
            <h3 className="text-lg font-semibold mb-2">GeoJSON Output</h3>
            <div className="bg-gray-50 p-3 rounded-md max-h-96 overflow-auto">
              <pre className="text-xs text-gray-700 whitespace-pre-wrap break-words">
                {JSON.stringify(geoJSON, null, 2)}
              </pre>
            </div>
          </div>

          <Button
            className="w-full bg-green-600 hover:bg-green-700"
            onClick={handleDownloadKML}
            disabled={geoJSON.features.length === 0}
          >
            <Download className="h-4 w-4 mr-2" />
            Download KML File
          </Button>

          <div className="text-xs text-gray-500 pt-2">
            <p>Features drawn: {geoJSON.features.length}</p>
          </div>
        </div>
      </div>
    </div>
  );
}
